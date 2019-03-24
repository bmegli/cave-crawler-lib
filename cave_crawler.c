/*
 * Cave Crawler Library implementation
 *
 * Copyright 2019 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

#include "cave_crawler.h"

#include <stdint.h> //uint8_t, int16_t, int32_t
#include <unistd.h> //read, close
#include <fcntl.h> //O_RDWR file open flag
#include <termios.h> //struct termios, tcgetattr, tcsetattr, cfsetispeed, tcflush
#include <string.h> //memcpy
#include <sys/select.h> //select
#include <malloc.h> //malloc, free
#include <errno.h> //errno
#include <endian.h> //htobe32, be32toh
#include <time.h> //time, difftime

/* TUNABLE CONSTANTS */
enum {CC_BUFFER_SIZE=2048};

// timeouts
enum {CC_INIT_TIMEOUT_MS=5000, CC_READ_TIMEOUT_MS=100};

/* NON TUNABLE CONSTANTS */

/*
## The packet structure

|          |  Start Byte   |  Size     | Type            |      Payload   | End Byte      |
| ---------|---------------|-----------|-----------------|----------------|---------------|
|   bytes  |    1          |      1    |    1            | type dependent |     1         |
|   value  | fixed 0xFB    |  0-255    | defined set     | type dependent |   fixed 0xFC  |

- CRC is already included in USB (not needed)
- Start Byte, Size, Type, End Byte can all be used for sync
- Size includes all the bytes ( payload bytes + 4)
- Payload starts with 4 bytes timestamp in microseconds
*/


// start end delimeters
enum {CC_START_OF_MESSAGE=0xFB, CC_END_OF_MESSAGE=0xFC};
// offsets
enum {CC_START_OF_MESSAGE_OFFSET=0, CC_MESSAGE_SIZE_OFFSET=1, CC_MESSAGE_TYPE_OFFSET=2, CC_MSG_PAYLOAD_OFFSET=3};

/*
## Types

Preliminary

|  Type       | Value  | Payload bytes |  Info                                                         |
| ------------|--------|---------------|-----------------------------------------|
|  ODOMETRY   | 0x01   |      28       |  encoders and IMU quaternions           |
|  XV11LIDAR  | 0x02   |      15       |  lidar data                             |
|  RPLIDARA3  | 0x03   |     137       |  compressed ultra capsules, sequence    |

*/

// types
enum {CC_ODOMETRY_TYPE=0x01, CC_XV11LIDAR_TYPE=0x02, CC_RPLIDAR_TYPE=0x03};
// sizes
enum {CC_ODOMETRY_SIZE=28+4, CC_XV11LIDAR_SIZE=15+4, CC_RPLIDAR_SIZE=137+4}; //to remove +4?
enum {CC_NON_PAYLOAD_SIZE=4}; //TO DELETE?


// validation return values
enum {CC_INVALID_MESSAGE=-1, CC_NEED_MORE_DATA=0, CC_VAlID_MESSAGE=1};

// message processing return values
enum {CC_NO_SPACE_IN_USER_ARRAY=-1, CC_MESSAGE_PROCESSED=0};

// internal library data
struct cc
{
	int fd;
	int data_pending;
	struct termios initial_termios;
	struct termios actual_termios;
	uint8_t buffer[CC_BUFFER_SIZE];
	int buffer_bytes;
};

/* Init and teardown */

struct cc *cc_init(const char *tty);
static int wait_for_input(struct cc *c);

int cc_close(struct cc *c);
static struct cc *close_free_and_return_null(struct cc *c);

/* Data reading functions */

int cc_odometry(struct cc *c, struct cc_odometry_data *data, int size);
int cc_rplidar(struct cc *c, struct cc_rplidar_data *data, int size);
int cc_xv11lidar(struct cc *c, struct cc_xv11lidar_data *data, int size);

int cc_read_all(struct cc *c, struct cc_data *data);

/* Message validation */

static int validate_message(struct cc *c, int from);
static int is_valid_message_start(uint8_t c);
static int is_valid_message_start_end(uint8_t msg_start, uint8_t msg_end);
static int is_valid_length_for_message_type(uint8_t msg_start,uint8_t msg_type, uint8_t msg_length);

/* Message processing and decoding */

static int process_message(uint8_t *msg, struct cc_data *data, struct cc_size *counters);

static void decode_message_odometry(uint8_t *msg, struct cc_odometry_data *data);
static void decode_message_rplidar(uint8_t *msg, struct cc_rplidar_data *data);
static void decode_message_xv11lidar(uint8_t *msg, struct cc_xv11lidar_data *data);

static uint16_t decode_uint16(uint8_t *encoded);
static int32_t decode_int32(uint8_t *encoded);
static uint32_t decode_uint32(uint8_t *encoded);
static float decode_float(uint8_t *encoded);

/* Stream settings functions */

/* Low level IO */
static int recv(struct cc *c);

/* ---------------------- IMPLEMENTATION ----------------------------- */

/* Init and teardown functions */

struct cc *cc_init(const char *tty)
{
	struct cc *c;

	c = (struct cc*)malloc(sizeof(struct cc));

	if( c == NULL )
		return NULL;

	c->buffer_bytes=0;
	c->data_pending=0;

	if ( (c->fd=open(tty, O_RDWR)) ==-1 )
	{
		free(c);
		return NULL;
	}

	if( tcgetattr(c->fd, &c->initial_termios) < 0 || tcgetattr(c->fd, &c->actual_termios) < 0 )
		return close_free_and_return_null(c);

	cfmakeraw(&c->actual_termios);
	c->actual_termios.c_cc[VMIN]=1;
	c->actual_termios.c_cc[VTIME]=0;

	if(tcsetattr(c->fd, TCSAFLUSH, &c->actual_termios) < 0)
		return close_free_and_return_null(c);

	// from man (TO DO)
	// Note that tcsetattr() returns success if any of the  requested  changes
	// could  be  successfully  carried  out.  Therefore, when making multiple
	// changes it may be necessary to follow this call with a further call  to
	// tcgetattr() to check that all changes have been performed successfully.
	//
	// this is still edge case to consider, some settings may have not been made
	// solution tcgetattr and check settings we made for equality

	//if device didn't produce any data in time fail
	if(wait_for_input(c) < 0)
	{
		cc_close(c);
		errno = EAGAIN;
		return NULL;
	}

	return c;
}

static int wait_for_input(struct cc *c)
{
	int ret;
	struct timeval tv={0};
	fd_set rfds;

	FD_ZERO(&rfds);
	FD_SET(c->fd, &rfds);

	tv.tv_sec = CC_INIT_TIMEOUT_MS / 1000;
	tv.tv_usec = (CC_INIT_TIMEOUT_MS % 1000) * 1000;

	if( (ret = select(c->fd+1, &rfds, NULL, NULL, &tv)) < 0 )
		return CC_ERROR;

	//we failed to get input in CC_INIT_TIMEOUT_MS time
	if (ret == 0) //timeout
	{
		errno = EAGAIN;
		return CC_ERROR;
	}

	return CC_OK;
}

int cc_close(struct cc *c)
{
	int error=0;

	if(c == NULL)
		return CC_OK;

	// Note that tcsetattr() returns success if any of the  requested  changes
	// could  be  successfully  carried  out.  Therefore, when making multiple
	// changes it may be necessary to follow this call with a further call  to
	// tcgetattr() to check that all changes have been performed successfully.

	//do not restore terminal settings, this leads to "hanging" serial for some reason
	//during next try to open
	//error |= tcsetattr(c->fd, TCSANOW, &c->initial_termios) < 0;

	error |= close(c->fd) < 0;

	free(c);

	if(error)
		return CC_ERROR;

	return CC_OK;
}

static struct cc *close_free_and_return_null(struct cc *c)
{
	close(c->fd);
	free(c);
	return NULL;
}

/* Data reading functions */

int cc_read_all(struct cc* c, struct cc_data *data)
{
	int valid, msg_process_status=CC_MESSAGE_PROCESSED, offset=0;
	struct cc_size counters={0};

	if( recv(c) == CC_ERROR )
	{
		data->size=counters;
		return CC_ERROR;
	}

	while( (valid=validate_message(c, offset)) != CC_NEED_MORE_DATA  )
	{
		if(valid == CC_INVALID_MESSAGE)
		{	//try luck starting from the next byte
			++offset;
			continue;
		}
		//otherwise CC_VALID_MESSAGE
		if( (msg_process_status=process_message(c->buffer+offset, data, &counters)) == CC_NO_SPACE_IN_USER_ARRAY)
			break;
		//otherwise CC_MESSAGE_PROCESSED
		offset+=c->buffer[offset+CC_MESSAGE_SIZE_OFFSET]; //TO DO - check if it is the right size
	}

	memmove(c->buffer, c->buffer+offset, c->buffer_bytes-offset);
	c->buffer_bytes -= offset;

	data->size = counters;

	if(msg_process_status == CC_NO_SPACE_IN_USER_ARRAY)
	{
		c->data_pending=1;
		return CC_DATA_PENDING;
	}
	//otherwise valid == CC_NEED_MORE_DATA
	c->data_pending=0;
	return CC_OK;
}

/* Message validation */

// returns CC_INVALID_MESSAGE or CC_NEED_MORE_DATA or CC_VALID_MESSAGE
static int validate_message(struct cc *c, int from)
{
	int pending_bytes=c->buffer_bytes-from;
	uint8_t msg_start, msg_size=UINT8_MAX, msg_end, msg_type;

	if(pending_bytes == 0)
		return CC_NEED_MORE_DATA;

	if(pending_bytes >= 1)
	{
		msg_start=c->buffer[from];
		if(!is_valid_message_start(msg_start))
			return CC_INVALID_MESSAGE;
	}

	if(pending_bytes >= 3)
	{
		msg_size=c->buffer[from+1];
		msg_type=c->buffer[from+2];

		if(!is_valid_length_for_message_type(msg_start, msg_type, msg_size))
			return CC_INVALID_MESSAGE;
	}

	if(pending_bytes >= msg_size ) //start, length, type/reserved, end so we have end of message
	{
		msg_end=c->buffer[from + msg_size -1];

		if(!is_valid_message_start_end(msg_start, msg_end))
			return CC_INVALID_MESSAGE;

		// if we got that far:
		// - message has correct start
		// - message has valid length for start/type
		// - message end delimter matches start delimeter
		// we conclude that it is a valid message
		return CC_VAlID_MESSAGE;
	}

	return CC_NEED_MORE_DATA;
}

static int is_valid_message_start(uint8_t c)
{
	return c == CC_START_OF_MESSAGE;
}

static int is_valid_message_start_end(uint8_t msg_start, uint8_t msg_end)
{
	return  (msg_start == CC_START_OF_MESSAGE && msg_end == CC_END_OF_MESSAGE);
}

static int is_valid_length_for_message_type(uint8_t msg_start,uint8_t msg_type, uint8_t msg_length)
{
	return msg_start == CC_START_OF_MESSAGE &&
	(  (msg_type == CC_ODOMETRY_TYPE && msg_length == CC_ODOMETRY_SIZE ) ||
		(msg_type == CC_RPLIDAR_TYPE && msg_length == CC_RPLIDAR_SIZE) ||
		(msg_type == CC_XV11LIDAR_TYPE && msg_length == CC_XV11LIDAR_SIZE) );
}

/* Message processing and decoding */

//returns CC_MESSAGE_PROCESSED or CC_NO_SPACE_IN_USER_ARRAY
static int process_message(uint8_t *msg, struct cc_data *data, struct cc_size *counters)
{
	const uint8_t msg_type=msg[CC_MESSAGE_TYPE_OFFSET];

	switch(msg_type)
	{
		case CC_ODOMETRY_TYPE:
			if(data->size.odometry==0)
				return CC_MESSAGE_PROCESSED;
			if(counters->odometry >= data->size.odometry)
				return CC_NO_SPACE_IN_USER_ARRAY;

			decode_message_odometry(msg, data->odometry + counters->odometry);

			++counters->odometry;
			break;
		case CC_RPLIDAR_TYPE:
			if(data->size.rplidar==0)
				return CC_MESSAGE_PROCESSED;
			if(counters->rplidar >= data->size.rplidar)
				return CC_NO_SPACE_IN_USER_ARRAY;

			decode_message_rplidar(msg, data->rplidar + counters->rplidar);

			++counters->rplidar;
			break;
		case CC_XV11LIDAR_TYPE:
			if(data->size.xv11lidar==0)
				return CC_MESSAGE_PROCESSED;
			if(counters->xv11lidar >= data->size.xv11lidar)
				return CC_NO_SPACE_IN_USER_ARRAY;

			decode_message_xv11lidar(msg, data->xv11lidar + counters->xv11lidar);

			++counters->xv11lidar;
			break;
		default:
			;//fprintf(stderr, "unsupported message type: %c\n", msg_type);
	}

	//if we got that far the message was processed or is of unsupported type
	return CC_MESSAGE_PROCESSED;
}

/* Message level decoding */


/*
### ODOMETRY

|           | Timestamp | Left encoder | Right encoder |    QW      |     QX     |    QY      |     QZ     |
| ----------|-----------|--------------|---------------|------------|------------|------------|------------|
|   bytes   |    4      |      4       |      4        |     4      |      4     |     4      |     4      |
|   type    | uint32    |   int32      |    int32      |   float    |   float    |   float    |   float    |
|   unit    |   us      |   counts     |    counts     | quaternion | quaternion | quaternion | quaternion |
*/

static void decode_message_odometry(uint8_t *msg, struct cc_odometry_data *data)
{
	uint8_t *payload=msg+CC_MSG_PAYLOAD_OFFSET;

	data->timestamp_us = decode_uint32(payload);

	data->left_encoder_counts = decode_int32(payload+4);
	data->right_encoder_counts = decode_int32(payload+8);

	data->qw = decode_float(payload+12);
	data->qx = decode_float(payload+16);
	data->qy = decode_float(payload+20);
	data->qz = decode_float(payload+24);
}

/*
### RPLIDARA3

|          | Timestamp | Sequence | Data               |
| ---------|-------- --|----------|--------------------|
|   bytes  |     4     |    1     |  132               |
|   type   |   uint32  |  uint8   | ultra_capsules     |
|   unit   |    us     |  counts  | RPLidarA3 internal |

- data is not decoded on MCU due to complexity
- sequence if for checking if data angles follow one another
- data corresponds to [rplidar_response_ultra_capsule_measurement_nodes_t](https://github.com/Slamtec/rplidar_sdk/blob/8291e232af614842447a634b6dbd725b81f24713/sdk/sdk/include/rplidar_cmd.h#L197) in [rplidar_sdk](https://github.com/Slamtec/rplidar_sdk)
- decoding should do the same as [_ultraCapsuleToNormal](https://github.com/Slamtec/rplidar_sdk/blob/master/sdk/sdk/src/rplidar_driver.cpp#L1071) in [rplidar_sdk](https://github.com/Slamtec/rplidar_sdk)
*/

static void decode_message_rplidar(uint8_t *msg, struct cc_rplidar_data *data)
{  //add some constants
	uint8_t *payload=msg+CC_MSG_PAYLOAD_OFFSET;

	data->timestamp_us = decode_uint32(payload);

	data->sequence = payload[4];

	memcpy( &data->capsule, payload+5, sizeof(rplidar_response_ultra_capsule_measurement_nodes_t) );
}

/*
### XV11LIDAR

|          | Timestamp | Angle quad            |  Speed64       | Distances x 4 [mm]            |
| ---------|-----------|-----------------------|----------------|-------------------------------|
|   bytes  |    4      |      1                |    2           |        8                      |
|   type   | uint32    |   uint8_t             |  uint16_t      | uint16_t  x 4 (array)         |
|   unit   |   us      | 0,89 for 0-3,356-359  | rpm = Speed/64 | flag, distances or error_code |

- the packet is raw XV11Lidar packet with timestamp, without signal strength, CRC
- Distances are 14 bits mm distances or error code, 1 bit strength warning, 1 bit invalid_data
    - if invalid_data bit is set, the field carries error code
    - otherwise it is distance in mm
    - strength warning when power received is lower than expected for the distance

 */
 /*
 struct cc_xv11lidar_data
{
	uint32_t timestamp_us;
	uint8_t angle_quad; //0-89 for readings 0-3 356-359
	uint16_t speed64;	//divide by 64 for speed in rpm
	uint16_t distances[4]; //flags and distance or error code
};
 */

static void decode_message_xv11lidar(uint8_t *msg, struct cc_xv11lidar_data *data)
{
	uint8_t *payload=msg+CC_MSG_PAYLOAD_OFFSET;

	data->timestamp_us = decode_uint32(payload);

	data->angle_quad = payload[4];
	data->speed64 = decode_uint16(payload+5);
	//TO DO
	//decode distances
}

/* Data type level decoding */

// note - the communication is currently simple little endian
// so the functions just copy memory
// TO DO - consider using network order/consider host byte order

static uint16_t decode_uint16(uint8_t *encoded)
{
	uint16_t temp;
	memcpy(&temp, encoded, sizeof(temp));
	return temp;
}
static uint32_t decode_uint32(uint8_t *encoded)
{
	uint32_t temp;
	memcpy(&temp, encoded, sizeof(temp));
	return temp;
}
static int32_t decode_int32(uint8_t *encoded)
{
	int32_t temp;
	memcpy(&temp, encoded, sizeof(temp));
	return temp;
}
static float decode_float(uint8_t *encoded)
{
	float tempf;
	memcpy(&tempf, encoded, sizeof(tempf));
	return tempf;
}

/* Low level IO */

static int recv(struct cc *c)
{
	int ret;
	struct timeval tv={0};
	fd_set rfds;

	if(c->data_pending)
		return CC_OK;

	FD_ZERO(&rfds);
	FD_SET(c->fd, &rfds);

	tv.tv_usec = CC_READ_TIMEOUT_MS*1000;

	if( (ret = select(c->fd+1, &rfds, NULL, NULL, &tv)) < 0 )
		return CC_ERROR;

	if (ret == 0) //timeout
	{
		errno = EAGAIN;
		return CC_ERROR;
	}

	if( (ret = read(c->fd, c->buffer+c->buffer_bytes, CC_BUFFER_SIZE-c->buffer_bytes )) < 0 )
		return CC_ERROR;
	if( ret == 0 )
	{ //EOF - device unplugged
		errno = ENODEV;
		return CC_ERROR;
	}

	c->buffer_bytes += ret;

	return CC_OK;
}

int cc_fd(struct cc *c)
{
	return c->fd;
}

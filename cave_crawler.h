/*
 * Cave Crawler Library header
 *
 * Copyright 2019 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

/**
 ******************************************************************************
 *
 *	\mainpage cave-crawler-lib documentation
 *	\see https://github.com/bmegli/cave-crawler-lib
 *
 *	\copyright	Copyright (C) 2019 Bartosz Meglicki
 *	\file		 cave_crawler.h
 *	\brief	 Library public interface header
 *
 ******************************************************************************
 */

#ifndef CAVE_CRAWLER_LIB_H_
#define CAVE_CRAWLER_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** \addtogroup interface Public interface
 *	@{
 */

/**
 * @struct cc
 * @brief Internal library data passed around by the user.
 * @see cc_init, cc_close
 */
struct cc;

/**
 * @struct cc_odometry_data
 * @brief Data streamed for odometry and IMU.
 * @see cc_read_all
 */
struct cc_odometry_data
{
	uint32_t timestamp_us;
	int32_t left_encoder_counts;
	int32_t right_encoder_counts;
	float qw,qx,qy,qz; //orientation quaternion
};

enum {RPLidarPacketDataSize=132};

/**
 * @struct cc_rplidar_data
 * @brief Data streamed for RPLidar A3
 * @see cc_read_all
 */
struct cc_rplidar_data
{
	uint32_t timestamp_us; //timestamp in microseconds
	uint8_t sequence;		 //0-255, wrap-around for checking if packets are consecutive
	uint8_t data[RPLidarPacketDataSize]; //raw measurement packet data
};

/**
 * @struct cc_xv11lidar_data
 * @brief Data streamed for XV11 Lidar
 * @see cc_read_all
 */
struct cc_xv11lidar_data
{
	uint32_t timestamp_us;
	uint8_t angle_quad; //0-89 for readings 0-3 356-359
	uint16_t speed64;	//divide by 64 for speed in rpm
	uint16_t distances[4]; //flags and distance or error code
};

/**
 * @struct cc_size
 * @brief Array sizes for \p cc_data arrays
 *
 * @see cc_data
 */
struct cc_size
{
	int odometry;
	int rplidar;
	int xv11lidar;
};

/**
 * @struct cc_data
 * @brief Structure with multiple types of data returned from the device.
 *
 * Supply only arrays of data your are interested in. Set arrays sizes in \p size member.
 *
 * @see cc_read_all
 */
struct cc_data
{
	struct cc_odometry_data *odometry;
	struct cc_rplidar_data *rplidar;
	struct cc_xv11lidar_data *xv11lidar;

	struct cc_size size; //array sizes
};

/**
	* @brief Constants returned by most of library functions
	*/
enum cc_retval_enum {
	CC_ERROR=-1, //!< error occured with errno set
	CC_OK=0, //!< succesfull execution
	CC_DATA_PENDING=1 //!< succesfull execution and more data pending without blocking
	};

/** @name Init and teardown
 */
///@{

/**
 * @brief initialize internal library data.
 * @param tty device like "/dev/ttyACM0"
 * @return
 * - pointer to internal library data
 * - NULL on error with errno set
 *
 * @see cc_close
 *
 * Example:
 * @code
 * struct cc *c=cc_init("/dev/ttyACM0");
 * @endcode
 */
struct cc *cc_init(const char *tty);

/**
 * @brief free library resources
 *
 * Frees memory and restores terminal settings.
 *
 * May be safely called with NULL argument.
 *
 * @param c pointer to internal library data
 * @return
 * - CC_OK on success
 * - CC_ERROR on error, query errno for the details
 *
 * Example:
 * @code
 * cc_close(v);
 * @endcode
 */
int cc_close(struct cc *c);

///@}

/** @name Convinient read
 *	Simplified read functions for single type of data (e.g. only odometry).
 *	If you need to read multiple types of data use \p cc_read_all instead.
 *
 *	Functions will block waiting for data unless last call returned value > \p size.
 *	Timeout with return value CC_ERROR and errno EAGAIN indicates device is not sending data type for some reason.
 *
 *	Other data types are discarded silently without parsing.
 *
 * @param c pointer to internal library data
 * @param data user supplied array
 * @param size user supplied array size
 * @return
 * - value > \p size indicates user array was filled and more data is pending (without blocking)
 * - value <= \p size indicates number of data points returned in array (next call will block)
 * - CC_ERROR indicates error, query errno for the details
 *
 * @see cc_read_all
 *
 * Example:
 * @code
 * int status, i;
 * struct cc_odometry_data odo[10];
 *
 * while( (status=cc_odometry(c, odo, 10)) != CC_ERROR	)
 * {
 *	for(i=0; i<status && i<10; ++i)
 *		 printf("[%lu ms] left=%d right=%d qw=%f qx=%f qy=%f qz=%f\n",
 *		 odo[i].timestamp_us,
 *		 odo[i].left_encoder_counts, odo[i].right_encoder_counts,
 *		 odo[i].qw, odo[i].qx, odo[i].qy, odo[i].qz)
 * }
 * @endcode
 */
///@{
/** @brief Read only odometry data. */
int cc_odometry(struct cc *c, struct cc_odometry_data *data, int size);
/** @brief Read only RPLidar A3 data */
int cc_rplidar(struct cc *c, struct cc_rplidar_data *data, int size);
/** @brief Read only XV11 lidar data. */
int cc_xv11lidar(struct cc *c, struct cc_xv11lidar_data *data, int size);
///@}

/**
 * @brief Read multiple types of data simultanously.
 *
 * Use this function if you need to read multiple types of data (e.g. odometry and rplidar).
 *
 * If you care only about single data type use one of convinience functions instead.
 *
 * Function will block waiting for data unless last call returned CC_DATA_PENDING.
 * Timeout with return value CC_ERROR and errno EAGAIN indicates device is not sending data types for some reason.
 *
 * Data types with 0 size in \p data parameter are discarded silently without parsing.
 *
 * @param c pointer to internal library data
 * @param data user supplied arrays with sizes
 * @return
 * - CC_OK indicates user arrays in \p data parameter were filled
 * - CC_DATA_PENDING indicates at least one array in \p data parameter was filled completely and more data is pending (without blocking)
 * - CC_ERROR indicates error, query errno for the details
 *
 * @see cc_odometry, cc_rplidar, cc_xv11lidar
 */
int cc_read_all(struct cc *c, struct cc_data *data);

/**
 * @brief Get file descriptor used for serial communication with the device
 *
 * Library user should not directly read or write from or to descriptor.
 * This function is intended to be used in synchronous I/O multiplexing (select, poll).
 *
 * @param c pointer to internal library data
 * @return file descriptor
 */
int cc_fd(struct cc *c);

/** @}*/


#ifdef __cplusplus
}
#endif

#endif //CAVE_CRAWLER_LIB_H_

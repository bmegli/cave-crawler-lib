# cave-crawler-lib

C library for communication with [cave-crawler-mcu](https://github.com/bmegli/cave-crawler-mcu)

## Platforms 

Library works on Unix platforms (e.g. Linux).

## Hardware

Library was written for [cave-crawler-mcu](https://github.com/bmegli/cave-crawler-mcu)

## Scope

Library was designed to retrieve readings history (not only the latest reading).

Library [documentation](https://bmegli.github.io/cave-crawler-lib/group__interface.html)

## Building Instructions

``` bash
# compilers, make and git
sudo apt-get update
sudo apt-get install build-essential git

# get cmake - we need to specify libcurl4 for Ubuntu 18.04 dependencies problem
sudo apt-get install libcurl4 cmake

# clone the repository
git clone https://github.com/bmegli/cave-crawler-lib

# finally build the library and examples
cd cave-crawler-lib
mkdir build
cd build
cmake
make
```

## Udev Rules

For Teensy add udev rule:

```bash
wget https://www.pjrc.com/teensy/49-teensy.rules
sudo mv 49-teensy.rules /etc/udev/rules.d/
```

## Testing

Plug [cave-crawler-mcu]((https://github.com/bmegli/cave-crawler-mcu)) and run `cc-read-all` with your device, e.g.: 

```bash
./cc-read-all /dev/ttyACM0
```

## Using

See examples directory for more complete examples with error handling. (TODO)

Library [documentation](https://bmegli.github.io/cave-crawler-lib/group__interface.html).


```C
	struct cc *c=NULL;
	struct cc_odometry_data odometry[DATA_SIZE];
	struct cc_rplidar_data rplidar[DATA_SIZE];
	struct cc_xv11lidar_data xv11lidar[DATA_SIZE];

	struct cc_data data={0};
	struct cc_size size={0};

	size.odometry = size.rplidar = size.xv11lidar = DATA_SIZE;	

	data.odometry = odometry;
	data.rplidar = rplidar;
	data.xv11lidar = xv11lidar;
	data.size = size;
		
	const char *tty_device = "/dev/ttyACM0";
	int ret, reads=0;
	
	c = cc_init(tty_device);

	while( (ret=cc_read_all(c, &data)) != CC_ERROR )
	{
		for(int i=0;i<data.size.odometry;++i)
			printf("[odo] t=%u i=%d left=%d right=%d qw=%f qx=%f qy=%f qz=%f\n",
			data.odometry[i].timestamp_us, i, data.odometry[i].left_encoder_counts,
			data.odometry[i].right_encoder_counts, data.odometry[i].qw,
			data.odometry[i].qx, data.odometry[i].qy, data.odometry[i].qz);

		for(int i=0;i<data.size.rplidar;++i)
			printf("[rp ] t=%u id=%d seq=%d\n",
			data.rplidar[i].timestamp_us, data.rplidar[i].device_id, data.rplidar[i].sequence);

		for(int i=0;i<data.size.xv11lidar;++i)
			printf("[xv11] t=%u aq=%d s=%d d=?\n", data.xv11lidar[i].timestamp_us,
			data.xv11lidar[i].angle_quad, data.xv11lidar[i].speed64/64);
					
		data.size=size;
	}
			
	cc_close(c);
```

## Compiling your code

### IDE (recommended)

Simply copy `cave_crawler.h` and `cave_crawler.c` to your project.

### CMake

Hopefully you know what you are doing.

### Manually

C
``` bash
gcc cave_crawler.c your_program.c -o your-program
```

C++
``` bash
gcc -c cave_crawler.c
g++ -c your_program.cpp
g++ cave_crawler.o your_program.o -o your-program
```

## License

Library is licensed under Mozilla Public License, v. 2.0

This is similiar to LGPL but more permissive:

- you can use it as LGPL in prioprietrary software
- unlike LGPL you may compile it statically with your code

Like in LGPL, if you modify this library, you have to make your changes available. Making a github fork of the library with your changes satisfies those requirements perfectly.

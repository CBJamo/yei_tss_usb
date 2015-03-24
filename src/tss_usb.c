/***************************************************************************//**
* \file tss_usb.c
*
* \brief Standalone C Driver for the YEI Corp 3-Space Sensor USB
* \author Scott K Logan
* \date January 07, 2013
*
* This is a standolone C driver for the YEI Corp 3-Space Sensor family of IMUs.
*
* \section license License (BSD-3)
* Copyright (c) 2013, Scott K Logan\n
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of Willow Garage, Inc. nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "yei_tss_usb/tss_usb.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

/*!
 * \brief Maximum number of simultaneously open device handles.
 */
#define MAX_HANDLES 256

#define CMD_HEADER 0xF7
#define CMD_HEADER_WITH_RESPONSE 0xF9

#define NO_FLUSH_BUFFER 1

#define HEADER_SUCCESS 0x1
#define HEADER_TIMESTAMP 0x2
#define HEADER_COMMAND_ECHO 0x4
#define HEADER_ADDITIVE_CHECKSUM 0x8
#define HEADER_LOGICAL_ID 0x10
#define HEADER_SERIAL_NUMBER 0x20
#define HEADER_DATA_LENGTH 0x40 

enum tss_usb_commands
{
	//Orientation Commands
        CMD_READ_FILTERED_TARED_ORIENTATION_QUATERNION = 0x00,
        CMD_READ_FILTERED_TARED_ORIENTATION_EULER = 0x01,
        CMD_READ_FILTERED_TARED_ORIENTATION_ROTATION_MATRIX = 0x02,
        CMD_READ_FILTERED_TARED_ORIENTATION_AXIS_ANGLE = 0x03,
        CMD_READ_FILTERED_TARED_ORIENTATION_TWO_VECTOR = 0x04,
        CMD_READ_DIFFERENCE_QUATERNION = 0x05,
        CMD_READ_FILTERED_UNTARED_ORIENTATION_QUATERNION = 0x06,
        CMD_READ_FILTERED_UNTARED_ORIENTATION_EULER = 0x07,
        CMD_READ_FILTERED_UNTARED_ORIENTATION_ROTATION_MATRIX = 0x08,
        CMD_READ_FILTERED_UNTARED_ORIENTATION_AXIS_ANGLE = 0x09,
        CMD_READ_FILTERED_UNTARED_ORIENTATION_TWO_VECTOR = 0x0A,
        CMD_READ_FILTERED_TARED_ORIENTATION_TWO_VECTOR_SENSOR_FRAME = 0x0B,
        CMD_READ_FILTERED_UNTARED_ORIENTATION_TWO_VECTOR_SENSOR_FRAME = 0x0C,
	//Nomalized Data Commands
        CMD_READ_NORMALIZED_COMPONENT_SENSOR_DATA = 0x20,
        CMD_READ_NORMALIZED_GYROS = 0x21,
        CMD_READ_NORMALIZED_ACCELEROMETER = 0x22,
        CMD_READ_NORMALIZED_COMPASS = 0x23,
	//Corrected Data Commands
        CMD_READ_CORRECTED_COMPONENT_SENSOR_DATA = 0x25,
        CMD_READ_CORRECTED_GYRO = 0x26,
        CMD_READ_CORRECTED_ACCELEROMETER = 0x27,
        CMD_READ_CORRECTED_COMPASS = 0x28,
        CMD_READ_CORRECTED_ACCELERATION_GLOBAL_SPACE = 0x29,
        CMD_CORRECT_RAW_GYRO = 0x30,
        CMD_CORRECT_RAW_ACCELEROMETER = 0x31,
        CMD_CORRECT_RAW_COMPASS = 0x32,
	//Other Data Commands
        CMD_GET_TEMPERATURE_C = 0x2B,
        CMD_GET_TEMPERATURE_F = 0x2C,
        CMD_GET_CONFIDENCE_FACTOR = 0x2D,
	//Raw Data Commands
        CMD_READ_RAW_COMPONENT_SENSOR_DATA = 0x40,
        CMD_READ_RAW_GYROS = 0x41,
        CMD_READ_RAW_ACCELEROMETER = 0x42,
        CMD_READ_RAW_COMPASS = 0x43,
	//Streaming Commands
	CMD_SET_STREAMING_SLOTS = 0x50,
	CMD_GET_STREAMING_SLOTS = 0x51,
	CMD_SET_STREAMING_TIMING = 0x52,
	CMD_GET_STREAMING_TIMING = 0x53,
	CMD_GET_STREAMING_BATCH = 0x54,
	CMD_START_STREAMING = 0x55,
	CMD_STOP_STREAMING = 0x56,
	CMD_UPDATE_TIMESTAMP = 0x5F,
	//Configuration Write Commands
	CMD_SET_EULER_DECOMPOSITION_ORDER = 0x10,
	CMD_OFFSET_CURRENT_ORIENTATION = 0x13,
	CMD_RESET_BASE_OFFSET = 0x14,
	CMD_OFFSET_WITH_QUATERNION = 0x15,
	CMD_BASE_OFFSET_CURRENT_ORIENTATION = 0x16,
        CMD_TARE_CURRENT_ORIENTATION = 0x60,
	CMD_TARE_QUATERNION = 0x61,
	CMD_TARE_ROTATION_MATRIX = 0x62,
	CMD_SET_STATIC_ACCELEROMETER_TRUST_VALUE = 0x63,
	CMD_SET_CONFIDENCE_ACCELEROMETER_TRUST_VALUES = 0x64,
	CMD_SET_STATIC_COMPASS_TRUST_VALUE = 0x65,
	CMD_SET_CONFIDENCE_COMPASS_TRUST_VALUES = 0x66,
        CMD_SET_MULTI_REFERENCE_VECTORS_WITH_CURRENT_ORIENTATION = 0x68,
        CMD_SET_REFERENCE_VECTOR_MODE = 0x69,
	CMD_SET_OVERSAMPLE_RATE = 0x6A,
	CMD_SET_GYROSCOPE_ENABLED = 0x6B,
	CMD_SET_ACCELEROMETE_ENABLED = 0x6C,
	CMD_SET_COMPASS_ENABLED = 0x6D,
	CMD_SET_MI_MODE_ENABLED = 0x70,
	CMD_SET_MI_MODE_PARAMETERS = 0x71,
	CMD_BEGIN_MI_MODE_CALIBRATION = 0x72,
        CMD_SET_AXIS_DIRECTIONS = 0x74,
	CMD_SET_RUNNING_AVERAGE_PERCENT = 0x75,
	CMD_SET_COMPASS_REFERENCE_VECTOR = 0x76,
	CMD_SET_ACCELEROMETER_REFERENCE_VECTOR = 0x77,
        CMD_RESET_FILTER = 0x78,
	CMD_SET_ACCELEROMETER_RANGE = 0x79,
	CMD_SET_FILTER_MODE = 0x7B,
	CMD_SET_RUNNING_AVERAGE_MODE = 0x7C,
	CMD_SET_GYROSCOPE_RANGE = 0x7D,
	CMD_SET_COMPASS_RANGE = 0x7E,
	//Configuration Read Commands
	CMD_GET_TARE_QUATERNION = 0x80,
	CMD_GET_TARE_ROTATION_MATRIX = 0x81,
	CMD_GET_ACCELEROMETER_TRUST_VALUES = 0x82,
	CMD_GET_COMPASS_TRUST_VALUES = 0x83,
	CMD_GET_UPDATE_RATE = 0x84,
	CMD_GET_COMPASS_REFERENCE_VECTOR = 0x85,
	CMD_GET_ACCELEROMETER_REFERENCE_VECTOR = 0x86,
        CMD_GET_REFERENCE_VECTOR_MODE = 0x87,
	CMD_GET_MI_MODE_ENABLED = 0x88,
	CMD_GET_MI_MODE_PARAMETERS = 0x89,
	CMD_GET_GYROSCOPE_ENABLED = 0x8C,
	CMD_GET_ACCELEROMETE_ENABLED = 0x8D,
	CMD_GET_COMPASS_ENABLED = 0x8E,
        CMD_GET_AXIS_DIRECTIONS = 0x8F,
	CMD_GET_OVERSAMPLE_RATE = 0x90,
	CMD_GET_RUNNING_AVERAGE_PERCENT = 0x91,
	CMD_READ_KALMAN_FILTERS_COVARIANCE_MATRIX = 0x93,
	CMD_GET_ACCELEROMETER_RANGE = 0x94,
	CMD_GET_FILTER_MODE = 0x98,
	CMD_GET_RUNNING_AVERAGE_MODE = 0x99,
	CMD_GET_GYROSCOPE_RANGE = 0x9A,
	CMD_GET_COMPASS_RANGE = 0x9B,
	CMD_GET_EULER_DECOMPOSITION_ORDER = 0x9C,
	CMD_GET_OFFSET_ORIENTATION_QUATERNION = 0x9F,

	//Calibration Commands
	CMD_SET_COMPASS_CALIBRATION_COEFFIECIENTS = 0xA0,
	CMD_SET_ACCELEROMETER_CALIBRATION_COEFFIECIENTS = 0xA1,
	CMD_GET_COMPASS_CALIBRATION_COEFFIECIENTS = 0xA2,
	CMD_GET_ACCELEROMETER_CALIBRATION_COEFFIECIENTS = 0xA3,
	CMD_GET_GYROSCOPE_CALIBRATION_COEFFIECIENTS = 0xA4,
	CMD_BEGIN_GYROSCOPE_AUTO_CALIBRATION = 0xA5,
	CMD_SET_GYROSCOPE_CALIBRATION_COEFFIECIENTS = 0xA6,
	CMD_SET_CALIBRATION_MODE = 0xA9,
	CMD_GET_CALIBRATION_MODE = 0xAA,

	//System Commands
	CMD_SET_LED_MODE = 0xC4,
	CMD_GET_LED_MODE = 0xC8,
	CMD_SET_RESPONSE_HEADER_BITFIELD = 0xDD,
	CMD_GET_RESPONSE_HEADER_BITFIELD = 0xDE,
        CMD_READ_VERSION_EXTENDED = 0xDF,
        CMD_RESTORE_FACTORY_SETTINGS = 0xE0,
        CMD_COMMIT_SETTINGS = 0xE1,
        CMD_SOFTWARE_RESET = 0xE2,
	CMD_SET_SLEEP_MODE = 0xE3,
	CMD_GET_SLEEP_MODE = 0xE4,
	CMD_ENTER_BOOTLOADER_MODE = 0xE5,
        CMD_GET_VERSION = 0xE6,
        CMD_SET_UART_BAUD_RATE = 0xE7,
        CMD_GET_UART_BAUD_RATE = 0xE8,
	CMD_SET_USB_MODE = 0xE9,
	CMD_GET_USB_MODE = 0xEA,
	CMD_GET_SERIAL_NUMBER = 0xED,
        CMD_SET_LED_COLOR = 0xEE,
        CMD_GET_LED_COLOR = 0xEF
};

struct tss_usb_priv
{
	int fd;
	int baud;
	char *port;
	char *version;
	char *version_extended;
	unsigned char header;
};

struct tss_usb_return_header
{
	unsigned char retval;
	unsigned int timestamp;
	unsigned char cmd;
	unsigned char checksum;
	// unsigned char logical_id;
	// unsigned int sernum;
	unsigned char payload_len;
} __attribute__((packed));

/*!
 * \brief List of communication handles.
 */
static struct tss_usb_priv * tss_usb_list[MAX_HANDLES] = { NULL };

/*!
 * \brief Grabs the next available device handle slot.
 *
 * Iterates through the ::MAX_HANDLES slots for the lowest available index.
 *
 * \returns Open slot index between 0 and ::MAX_HANDLES
 * \retval -1 No available slots
 */
static int next_available_handle( )
{
	unsigned short int i;
	for( i = 0; i < MAX_HANDLES; i++ )
	{
		if( !tss_usb_list[i] )
			return i;
	}
	return -1;
}

//This is a convenience function to calc the last byte in the packet
// commands without parameters can use the same number as the command
static unsigned char create_checksum( const unsigned char *command_bytes,
	const unsigned int num_bytes )
{
	unsigned int chkSum = 0;
	unsigned int i;
	for( i = 0; i < num_bytes; i++ )
		chkSum += command_bytes[i];
	return (unsigned char)(chkSum % 256);
}

//The 3-Space sensors are Big Endian and x86 is Little Endian
//So the bytes need be swapped around, this function can convert from and
// to big endian
static inline void endian_swap( unsigned int *x )
{
	*x = ( *x >> 24 ) |
		( ( *x << 8 ) & 0x00FF0000 ) |
		( ( *x >> 8 ) & 0x0000FF00 ) |
		( *x << 24 );
}

static int baud2term( int *baud )
{
	switch( *baud )
	{
	case 1200:
		return B1200;
		break;
	case 2400:
		return B2400;
		break;
	case 4800:
		return B4800;
		break;
	case 9600:
		return B9600;
		break;
	case 19200:
		return B19200;
		break;
	case 38400:
		return B38400;
		break;
	case 57600:
		return B57600;
		break;
	case 115200:
		return B115200;
		break;
	case 230400:
		return B230400;
		break;
	case 460800:
		return B460800;
		break;
	case 921600:
		return B921600;
		break;
	default:
		return B0;
		break;
	}
}

static int send_cmd( const int fd, const unsigned char *data, const size_t num_bytes )
{
	int bytes_sent = write( fd, data, num_bytes );
	if( bytes_sent < 0 )
		return TSS_USB_ERROR_IO;
	else if( bytes_sent == 0 )
		return TSS_USB_ERROR_TIMEOUT;
	else if( (unsigned)bytes_sent != num_bytes )
		return TSS_USB_ERROR_IO;
	return TSS_USB_SUCCESS;
}

static int read_data( const int fd, unsigned char *data, size_t num_bytes )
{
	int bytes_recv;
	while( num_bytes )
	{
		bytes_recv = read( fd, data, num_bytes );
		if( bytes_recv < 0 )
			return TSS_USB_ERROR_IO;
		if( bytes_recv == 0 )
			return TSS_USB_ERROR_TIMEOUT;
		num_bytes -= bytes_recv;
	}
	return TSS_USB_SUCCESS;
}

int tss_usb_open( const char *port )
{
	/* Step 1: Make sure the device opens OK */
	int fd = open( port, O_RDWR | O_NOCTTY | O_NDELAY );
	if( fd < 0 )
		return TSS_USB_ERROR_NO_DEVICE;

	fcntl( fd, F_SETFL, 0 );

	struct termios options;
	cfmakeraw( &options );
	if( cfsetispeed( &options, B115200 ) < 0 )
	{
		close( fd );
		return TSS_USB_ERROR_IO;
	}
	if( cfsetospeed( &options, B115200 ) < 0 )
	{
		close( fd );
		return TSS_USB_ERROR_IO;
	}
	options.c_cflag &= ~HUPCL;
	options.c_lflag &= ~ICANON;
	options.c_cc[VTIME] = 2;
	options.c_cc[VMIN] = 0;
	if( tcsetattr( fd, TCSANOW, &options ) < 0 )
	{
		close( fd );
		return TSS_USB_ERROR_IO;
	}

	/* Step 2: Set our baud rate to match the device, if necessary */

	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( fd, TCIOFLUSH );
	#endif

	unsigned char buf[7] = { CMD_HEADER, CMD_GET_UART_BAUD_RATE, CMD_GET_UART_BAUD_RATE };
	int ret;
	if( ( ret = send_cmd( fd, buf, 3 * sizeof( unsigned char ) ) ) < 0 )
	{
		close( fd );
		return ret;
	}
	int dev_baud;
	if( ( ret = read_data( fd, (unsigned char *)&dev_baud, sizeof( int ) ) ) < 0 )
	{
		close( fd );
		return ret;
	}
	endian_swap( (unsigned int *)&dev_baud );
	int new_baud = baud2term( &dev_baud );
	if( new_baud != 115200 )
	{
		if( cfsetispeed( &options, new_baud ) == -1 )
		{
			close( fd );
			return TSS_USB_ERROR_IO;
		}
		if( cfsetospeed( &options, new_baud ) == -1 )
		{
			close( fd );
			return TSS_USB_ERROR_IO;
		}
		if( tcsetattr( fd, TCSANOW, &options ) == -1 )
		{
			close( fd );
			return TSS_USB_ERROR_IO;
		}
	}

	/* Step 3: Setup response header */
	
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	buf[0] = CMD_HEADER;
	buf[1] = CMD_SET_RESPONSE_HEADER_BITFIELD;
	*((unsigned int *)&buf[2]) = ( HEADER_SUCCESS | HEADER_TIMESTAMP | HEADER_COMMAND_ECHO | HEADER_ADDITIVE_CHECKSUM | HEADER_DATA_LENGTH );
	endian_swap( (unsigned int *)&buf[2] );
	buf[6] = create_checksum( &buf[1], sizeof( buf ) - 2 );

	if( ( ret = send_cmd( fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;


	/* Step 4: Allocate a private struct */
	int mydev = next_available_handle( );
	if( mydev < 0 )
	{
		close( fd );
		return TSS_USB_ERROR_NO_MEM;
	}

	tss_usb_list[mydev] = malloc( sizeof( struct tss_usb_priv ) );
	if( !tss_usb_list[mydev] )
	{
		close( fd );
		return TSS_USB_ERROR_NO_MEM;
	}

	memset( tss_usb_list[mydev], 0, sizeof( struct tss_usb_priv ) );

	tss_usb_list[mydev]->port = malloc( strlen( port ) + 1 );
	if( !tss_usb_list[mydev]->port )
	{
		free( tss_usb_list[mydev] );
		close( fd );
		return TSS_USB_ERROR_NO_MEM;
	}
	memcpy( tss_usb_list[mydev]->port, port, strlen( port ) + 1 );
	tss_usb_list[mydev]->fd = fd;
	tss_usb_list[mydev]->baud = dev_baud;
	tss_usb_list[mydev]->header = CMD_HEADER;

	return TSS_USB_SUCCESS;
}

void tss_usb_close( const int tssd )
{
	if( tssd < 0 || tssd > MAX_HANDLES || !tss_usb_list[tssd] )
		return;

	close( tss_usb_list[tssd]->fd );

	free( tss_usb_list[tssd]->port );
	free( tss_usb_list[tssd]->version );
	free( tss_usb_list[tssd]->version_extended );

	free( tss_usb_list[tssd] );
	tss_usb_list[tssd] = NULL;
}

int tss_set_header_enabled( const int tssd, char enable )
{
	tss_usb_list[tssd]->header = enable ? CMD_HEADER_WITH_RESPONSE : CMD_HEADER;

	return TSS_USB_SUCCESS;
}

int tss_get_led( const int tssd, float vals[3] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_GET_LED_COLOR, CMD_GET_LED_COLOR };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* If Headers Are Enabled Read Header And Response */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;
		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 3 * sizeof( float ) )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;
	
		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)vals, 3 * sizeof( float ) ) != hdr.checksum )
			return TSS_USB_ERROR_CHECKSUM;
	}
	/* Else Read Only Response */
	else
	{
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;
	}

	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );

	return TSS_USB_SUCCESS;
}

int tss_set_led( const int tssd, const float vals[3] )
{
	/* Construct Packet Payload */
	unsigned char buf[2 + 3 * sizeof( float ) + 1] = { tss_usb_list[tssd]->header, CMD_SET_LED_COLOR };
	int ret;
	memcpy( &buf[2], &vals[0], sizeof( float ) );
	endian_swap( (unsigned int *)&buf[2] );
	memcpy( &buf[2 + sizeof( float )], &vals[1], sizeof( float ) );
	endian_swap( (unsigned int *)&buf[2 + sizeof( float )] );
	memcpy( &buf[2 + 2 * sizeof( float )], &vals[2], sizeof( float ) );
	endian_swap( (unsigned int *)&buf[2 + 2 * sizeof( float )] );
	buf[2 + 3 * sizeof( float )] = create_checksum( &buf[1], sizeof( buf ) - 2 );

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) > 0  )
		return ret;

	/* If Headers Are Enabled Read Header */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;

		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 0 )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	}
	
	return TSS_USB_SUCCESS;
}

int tss_get_orientation_quaternion( const int tssd, float vals[4] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_READ_FILTERED_TARED_ORIENTATION_QUATERNION, CMD_READ_FILTERED_TARED_ORIENTATION_QUATERNION };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* If Headers Are Enabled Read Header And Response */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;
		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 4 * sizeof( float ) )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;

		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 4 * sizeof( float ) ) ) < 0 )
			return ret;

		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)vals, 4 * sizeof( float ) ) != hdr.checksum )
			return TSS_USB_ERROR_CHECKSUM;
	}
	/* Else Read Only Response */
	else
	{
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 4 * sizeof( float ) ) ) < 0 )
			return ret;
	}

	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );
	endian_swap( (unsigned int *)&vals[3] );

	return TSS_USB_SUCCESS;
}

int tss_get_filtered_gyro( const int tssd, float vals[3] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_READ_NORMALIZED_GYROS, CMD_READ_NORMALIZED_GYROS };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* If Headers Are Enabled Read Header And Response */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;
		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 3 * sizeof( float ) )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;

		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;

		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)vals, 3 * sizeof( float ) ) != hdr.checksum )
			return TSS_USB_ERROR_CHECKSUM;
	}
	/* Else Read Only Response */
	else
	{
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;
	}


	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );

	return TSS_USB_SUCCESS;
}

int tss_get_normalized_accel( const int tssd, float vals[3] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_READ_NORMALIZED_ACCELEROMETER, CMD_READ_NORMALIZED_ACCELEROMETER };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* If Headers Are Enabled Read Header And Response */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;
		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 3 * sizeof( float ) )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;

		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;

		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)vals, 3 * sizeof( float ) ) != hdr.checksum )
			return TSS_USB_ERROR_CHECKSUM;
	}
	/* Else Read Only Response */
	else
	{
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;
	}


	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );

	return TSS_USB_SUCCESS;
}

int tss_get_accel( const int tssd, float vals[3] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_READ_CORRECTED_ACCELEROMETER, CMD_READ_CORRECTED_ACCELEROMETER };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* If Headers Are Enabled Read Header And Response */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;
		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 3 * sizeof( float ) )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;

		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;

		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)vals, 3 * sizeof( float ) ) != hdr.checksum )
			return TSS_USB_ERROR_CHECKSUM;
	}
	/* Else Read Only Response */
	else
	{
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;
	}


	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );

	return TSS_USB_SUCCESS;
}

int tss_get_covariance( const int tssd, float vals[16] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_READ_KALMAN_FILTERS_COVARIANCE_MATRIX, CMD_READ_KALMAN_FILTERS_COVARIANCE_MATRIX };
	int ret;
	unsigned char i;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* If Headers Are Enabled Read Header And Response */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;
		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 16 * sizeof( float ) )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;

		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 16 * sizeof( float ) ) ) < 0 )
			return ret;

		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)vals, 16 * sizeof( float ) ) != hdr.checksum )
			return TSS_USB_ERROR_CHECKSUM;
	}
	/* Else Read Only Response */
	else
	{
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 16 * sizeof( float ) ) ) < 0 )
			return ret;
	}


	for( i = 0; i < 16; i++ )
		endian_swap( (unsigned int *)&vals[i] );

	return TSS_USB_SUCCESS;
}

int tss_set_axis_directions( const int tssd, const unsigned char val )
{
	/* Construct Packet Payload */
	unsigned char buf[2 + sizeof( unsigned char ) + 1] = { tss_usb_list[tssd]->header, CMD_SET_AXIS_DIRECTIONS };
	int ret;
	memcpy( &buf[2], &val, sizeof( unsigned char ) );
	buf[2 + sizeof( unsigned char )] = create_checksum( &buf[1], sizeof( buf ) - 2 );

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0)
		return ret;

	/* If Headers Are Enabled Read Header */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;

		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 0 )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	}

	return TSS_USB_SUCCESS;
}

int tss_get_version( const int tssd, char vals[33] )
{
	int ret = TSS_USB_SUCCESS;

	if( !tss_usb_list[tssd]->version )
	{
		#ifndef NO_FLUSH_BUFFER
		/* Clear Response Buffer */
		tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
		#endif

		/* Construct Packet Payload */
		unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_GET_VERSION, CMD_GET_VERSION };

		if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
			return ret;

		tss_usb_list[tssd]->version = calloc( 33, sizeof( char ) );
		tss_usb_list[tssd]->version[32] = '\0';

		/* If Headers Are Enabled Read Header And Response */
		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
		{
			struct tss_usb_return_header hdr;
			if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
				goto comm_err;
			endian_swap( &hdr.timestamp );

			if( hdr.payload_len != 32 * sizeof( char ) )
			{
				ret = TSS_USB_ERROR_PAYLOAD_MISMATCH;
				goto comm_err;
			}	

			if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)tss_usb_list[tssd]->version, 32 * sizeof( char ) ) ) < 0 )
				goto comm_err;

			if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)tss_usb_list[tssd]->version, 32 * sizeof( char ) ) != hdr.checksum )
			{
				ret = TSS_USB_ERROR_CHECKSUM;
				goto comm_err;
			}
		}
		/* Else Read Only Response */
		else
		{
			if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)tss_usb_list[tssd]->version, 32 * sizeof( char ) ) ) < 0 )
				return ret;
		}
	}

	memcpy( vals, tss_usb_list[tssd]->version, 33 * sizeof( char ) );

	return ret;

comm_err:
	free( tss_usb_list[tssd]->version );
	tss_usb_list[tssd]->version = NULL;
	return ret;
}

int tss_get_version_extended( const int tssd, char vals[13] )
{
	int ret = TSS_USB_SUCCESS;
	if( !tss_usb_list[tssd]->version_extended )
	{
		#ifndef NO_FLUSH_BUFFER
		/* Clear Response Buffer */
		tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
		#endif

		/* Construct Packet Payload */
		unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_READ_VERSION_EXTENDED, CMD_READ_VERSION_EXTENDED };

		if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
			return ret;

		tss_usb_list[tssd]->version_extended = calloc( 13, sizeof( char ) );
		tss_usb_list[tssd]->version_extended[12] = '\0';

		/* If Headers Are Enabled Read Header And Response */
		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
		{
			struct tss_usb_return_header hdr;
			if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
				goto comm_err;
			endian_swap( &hdr.timestamp );

			if( hdr.payload_len != 12 * sizeof( char ) )
			{	
				ret = TSS_USB_ERROR_PAYLOAD_MISMATCH;
				goto comm_err;
			}

			if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)tss_usb_list[tssd]->version, 12 * sizeof( char ) ) ) < 0 )
				goto comm_err;

			if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)tss_usb_list[tssd]->version, 12 * sizeof( char ) ) != hdr.checksum ) 
			{
				ret = TSS_USB_ERROR_CHECKSUM;
				goto comm_err;
			}
		}
		/* Else Read Only Response */
		else
		{
			if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)tss_usb_list[tssd]->version, 12 * sizeof( char ) ) ) < 0 )
				return ret;
		}
	}

	memcpy( vals, tss_usb_list[tssd]->version_extended, 13 * sizeof( char ) );

	return TSS_USB_SUCCESS;

comm_err:
	free( tss_usb_list[tssd]->version_extended );
	tss_usb_list[tssd]->version_extended = NULL;
	return ret;
}

int tss_reset_filter( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_RESET_FILTER, CMD_RESET_FILTER };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0)
		return ret;

	/* If Headers Are Enabled Read Header */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;

		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 0 )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	}

	return TSS_USB_SUCCESS;
}

int tss_tare( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_TARE_CURRENT_ORIENTATION, CMD_TARE_CURRENT_ORIENTATION };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0)
		return ret;

	/* If Headers Are Enabled Read Header */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;

		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 0 )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	}

	return TSS_USB_SUCCESS;
}

int tss_commit( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_COMMIT_SETTINGS, CMD_COMMIT_SETTINGS };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0)
		return ret;

	/* If Headers Are Enabled Read Header */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;

		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 0 )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	}

	return TSS_USB_SUCCESS;
}

int tss_reset( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_SOFTWARE_RESET, CMD_SOFTWARE_RESET };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0)
		return ret;

	/* If Headers Are Enabled Read Header */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;

		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 0 )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	}

	return TSS_USB_SUCCESS;
}

int tss_restore_factory_settings( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_RESTORE_FACTORY_SETTINGS, CMD_RESTORE_FACTORY_SETTINGS };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0)
		return ret;

	/* If Headers Are Enabled Read Header */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;

		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 0 )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	}

	return TSS_USB_SUCCESS;
}

int tss_set_multi_reference_vectors( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_SET_MULTI_REFERENCE_VECTORS_WITH_CURRENT_ORIENTATION, CMD_SET_MULTI_REFERENCE_VECTORS_WITH_CURRENT_ORIENTATION };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0)
		return ret;

	/* If Headers Are Enabled Read Header */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;

		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 0 )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	}

	return TSS_USB_SUCCESS;
}

int tss_set_reference_mode( const int tssd, const unsigned char val )
{
	/* Construct Packet Payload */
	unsigned char buf[2 + sizeof( unsigned char ) + 1] = { tss_usb_list[tssd]->header, CMD_SET_REFERENCE_VECTOR_MODE };
	int ret;
	memcpy( &buf[2], &val, sizeof( unsigned char ) );
	buf[2 + sizeof( unsigned char )] = create_checksum( &buf[1], sizeof( buf ) - 2 );

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0)
		return ret;

	/* If Headers Are Enabled Read Header */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;

		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 0 )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;
	}

	return TSS_USB_SUCCESS;
}

int tss_get_temperature_c( const int tssd, float *val )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_GET_TEMPERATURE_C, CMD_GET_TEMPERATURE_C };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* If Headers Are Enabled Read Header And Response */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;
		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != sizeof( float ) )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;

		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)val, sizeof( float ) ) ) < 0 )
			return ret;

		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)val, sizeof( float ) ) != hdr.checksum )
			return TSS_USB_ERROR_CHECKSUM;
	}
	/* Else Read Only Response */
	else
	{
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)val, sizeof( float ) ) ) < 0 )
			return ret;
	}


	endian_swap( (unsigned int *)val );

	return TSS_USB_SUCCESS;
}

int tss_read_compass( const int tssd, float vals[3] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { tss_usb_list[tssd]->header, CMD_READ_CORRECTED_COMPASS, CMD_READ_CORRECTED_COMPASS };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* If Headers Are Enabled Read Header And Response */
	if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE )
	{
		struct tss_usb_return_header hdr;
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)&hdr, sizeof( struct tss_usb_return_header ) ) ) < 0 )
			return ret;
		endian_swap( &hdr.timestamp );

		if( hdr.payload_len != 3 * sizeof( float ) )
			return TSS_USB_ERROR_PAYLOAD_MISMATCH;

		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;

		if( tss_usb_list[tssd]->header == CMD_HEADER_WITH_RESPONSE && create_checksum( (unsigned char *)vals, 3 * sizeof( float ) ) != hdr.checksum )
			return TSS_USB_ERROR_CHECKSUM;
	}
	/* Else Read Only Response */
	else
	{
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
			return ret;
	}


	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );

	return TSS_USB_SUCCESS;
}


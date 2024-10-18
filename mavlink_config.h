/********************************************************************************************************************************
 *
 *	@copyright Copyright (C) 2015  Unison Networks Ltd
 *
 *  FILE:           mavlink_config.h
 *
 *  LIBRARY:		mav_stack/mavlink
 *
 *  AUTHOR: 		Edwin Hayes
 *
 *  DATE CREATED:	13-2-2013
 *
 *	Header file which defines the MAVlink configuration used by the device.
 * 
 ********************************************************************************************************************************/

// Only include this header file once.
#pragma once

// INCLUDE REQUIRED HEADER FILES FOR INTERFACE.

// Include the STDINT fixed width types.
#include <stdint.h>

// DEFINE PUBLIC PREPROCESSOR MACROS.

// FORWARD DEFINE PRIVATE PROTOTYPES.

// DEFINE PUBLIC CLASSES, TYPES AND ENUMERATIONS.

#ifndef MAVLINK_STX
	#define MAVLINK_STX 253 //mavlink v2
#endif

#ifndef MAVLINK_ENDIAN
	#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
	#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
	#define MAVLINK_CRC_EXTRA 1
#endif

// #ifndef MAVLINK_COMM_NUM_BUFFERS
// 	#define MAVLINK_COMM_NUM_BUFFERS 1
// #endif

// DECLARE PUBLIC GLOBAL VARIABLES.

// DEFINE PUBLIC STATIC FUNCTION PROTOTYPES.

// ALL DONE.

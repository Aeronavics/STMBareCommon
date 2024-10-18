/********************************************************************************************************************************
 *
 *	@copyright Copyright (C) 2015 Aeronavics Ltd
 *
 *  FILE:           mavlink.h
 *
 *  LIBRARY:        avr_bare_common
 *
 *  AUTHOR:         Edwin Hayes
 *
 *  DATE CREATED:   21-8-2016
 *
 *	This is the header file which matches mavlink.c.
 *
 *  Provides common definitions required for using MAVlink.
 * 
 ********************************************************************************************************************************/

// Only include this header file once.
#pragma once

// INCLUDE REQUIRED HEADER FILES FOR INTERFACE.

// Include the STDINT fixed width types.
#include <stdint.h>

// Include the C99 boolean type.
#include <stdbool.h>
#include "mavlink/mavlink_types.h"
#define MAVLINK_EXTERNAL_RX_STATUS
#define MAVLINK_EXTERNAL_RX_BUFFER

#ifndef EXTERN
extern mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
extern mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#else
mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#endif

// Include the all important MAVlink library.
#include "mavlink_config.h"
#include "mavlink/aeronavics/mavlink.h"

// DEFINE PUBLIC PREPROCESSOR MACROS.

// DEFINE PUBLIC TYPES AND ENUMERATIONS.

// A type encapsulating the MAVlink system ID (a number between 1-255, 0 is not a valid sID).
typedef uint8_t SID;

// A type encapsulating the MAVlink component ID (a number between 1-255, 0 is not a valid cID).
typedef uint8_t CID;

// DECLARE PUBLIC GLOBAL VARIABLES.


// FORWARD DEFINE PRIVATE PROTOTYPES.

// DEFINE PUBLIC CLASSES.

// DEFINE PUBLIC STATIC FUNCTION PROTOTYPES.

/**
 * Returns the SystemID of the UAS the component is running on, which is loaded from a parameter the first time this is called.
 *
 * @param	Nothing.
 * @return	The MAVlink system ID of the UAS.
 */
SID get_sID(void);

/**
 * Sets the SystemID of the component: this actually just changes the underlying parameter value, so need to save the parameters, and then reboot.
 *
 * @param sID The MAVlink system ID of the UAS.
 * @return Nothing.
 */
void set_sID(SID sID);

/**
 * Returns the ComponentID of the component, which is loaded from a parameter the first time this is called.
 *
 * @param	Nothing.
 * @Returns	The MAVlink component ID of this component.
 */
CID get_cID(void);

/**
 * Sets the ComponentID of the component: this actually just changes the underlying parameter value, so need to save the parameters, and then reboot.
 *
 * @param cID The MAVlink component ID of this component.
 * @return Nothing.
 */
void set_cID(CID cID);

// ALL DONE.

/********************************************************************************************************************************
 *
 *	@copyright Copyright (C) 2016 Aeronavics Ltd
 *
 *  FILE: 		    mavlink.c
 *
 *  LIBRARY:		avr_bare_common
 *
 *  AUTHOR: 		Edwin Hayes
 *
 *  DATE CREATED:	21-8-2016
 *
 *  Provides common definitions required for using MAVlink.
 *
 ********************************************************************************************************************************/

// INCLUDE THE MATCHING HEADER FILE.
#define EXTERN // Fairly common C/CPP magic to allow global defintions in header files.
#include "mavlink.hpp"
#undef EXTERN
// INCLUDE REQUIRED HEADER FILES FOR IMPLEMENTATION.

#include "params.hpp"

// DEFINE PRIVATE MACROS.

// TODO - Why is 42 the default SID?

#define DEFAULT_SID 42

// TODO - Need to make sure this doesn't collide with any real component.

#define DEFAULT_CID 254

// DEFINE PRIVATE TYPES AND STRUCTS.

// DECLARE PRIVATE GLOBAL VARIABLES.

// DEFINE PRIVATE FUNCTION PROTOTYPES.

// IMPLEMENT PUBLIC FUNCTIONS.

SID get_sID(void)
{
	// Try to read the sID parameter, else use the default.
	// SID sID = param_fetch(Param_name("SID"), Param_value::from<uint8_t>(DEFAULT_SID)).as<uint8_t>();

	// All done.
	return param_fetch(Param_name("SID"), Param_value::from<uint8_t>(DEFAULT_SID)).as<uint8_t>();
}

void set_sID(SID sID)
{
	// Set the underlying parameter.
	param_set(Param_name("SID"), Param_value::from<uint8_t>(sID));

	// All done.
	return;
}

CID get_cID(void)
{
	// Try to read the cID parameter, else use the default.
	// static CID cID = param_fetch(Param_name("CID"), Param_value::from<uint8_t>(DEFAULT_CID)).as<uint8_t>();

	// All done.
	return param_fetch(Param_name("CID"), Param_value::from<uint8_t>(DEFAULT_CID)).as<uint8_t>();//ID;
}

void set_cID(CID cID)
{
	// Set the underlying parameter.
	param_set(Param_name("CID"), Param_value::from<uint8_t>(cID));

	// All done.
	return;
}

// IMPLEMENT PRIVATE FUNCTIONS.

// IMPLEMENT INTERRUPT SERVICE ROUTINES.

// ALL DONE.

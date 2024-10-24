/********************************************************************************************************************************
 *
 *  FILE: 	params.cpp
 *
 *  SUB-SYSTEM:	 	Avionics
 *
 *  COMPONENT:		Payload Controller.
 *
 *  TARGET:        	All
 *
 *  PLATFORM:       BareMetal
 *
 *  AUTHOR:         Anton Slooten
 *
 *  DATE CREATED:   27-02-2017
 *
 *	Device driver which provides a library for MAVlink parameter management using static compile time parameter definitions.
 *
 *  Copyright 2017 Aeronavics Ltd
 *
 ********************************************************************************************************************************/

// TODO - Change the comment block to the one meant for library files.

// INCLUDE THE MATCHING HEADER FILE.

#include "params.hpp"

// INCLUDE REQUIRED HEADER FILES FOR IMPLEMENTATION.

#define EXTERN // Fairly common C/CPP magic to allow global defintions in header files.
#include "mavlink_params.hpp" // A file containing the actual parameter definitions must be present in each component using the library.
#undef EXTERN

// DEFINE PRIVATE MACROS

#define CID_FC 1 // The cID of the flight controller, which is the canonical source of when the system is armed/disarmed.

// #define EEPROM_FLAG_ADDRESS 0x0000
// //This just checks to make sure there is no override for the start place of the eeprom.
// #ifndef EEPROM_PARAM_ADDRESS_START
// 	#define EEPROM_FLAG_BYTE0	0x34
// #else
// 	#define EEPROM_FLAG_BYTE0 EEPROM_PARAM_ADDRESS_START
// #endif

// DEFINE PRIVATE CLASSES, TYPES AND ENUMERATIONS.

// DECLARE PRIVATE GLOBAL VARIABLES.

// DEFINE PRIVATE FUNCTION PROTOTYPES.

void safe_strcpy(char* dst, const char* src);

// IMPLEMENT PUBLIC STATIC FUNCTIONS.

Mavlink_params& Mavlink_params::get_driver(void)
{
    // Create a singleton for the driver.
    static Mavlink_params singleton;

    // Return the driver.
    return singleton;
}

const Param_value param_fetch(const size_t index, const Param_value def)
{
    // Check if we're asking for an index which doesn't exist.
    if (index >= NUM_PARAMS)
    {
        // We just return the default.  This is a little insane because you can't actually tell if this has happened, but since these functions are for internal use, too bad.
        return def;
    }
    // Else return the value from the parameter at the specified index.
    return parameters[index].value;
}

const Param_name param_name_fetch(const size_t index)
{
    if (index >= NUM_PARAMS)
    {
        // We just return an empty string.
        return Param_name("");
    }

    return parameters[index].name;

}

const Param_value param_fetch(const Param_name name, const Param_value def)
{
    // Iterate over each of the parameters.
    for (size_t i = 0; i < NUM_PARAMS; i++)
    {
        // Check if the name matches.
        bool match = true;
        for (size_t c = 0; c < MAVLINK_PARAM_NAME_MAX_LENGTH; c++)
        {
            // If the characters in both strings at this position aren't the same, it's not a match.
            if (name[c] != parameters[i].name[c])
            {
                match = false;
                break;
            }

            // If we reach the end of the string, stop.
            if (name[c] == 0) break;
        }
        if (match)
        {
            // This is the right one.
            return parameters[i].value;
        }
    }

    // Else if we get here, we didn't find a match, so we just return the default.
    return def;
}

bool param_exists(const size_t index)
{
    // Pretty simple, just return whether the index is too large.
    return (index <= NUM_PARAMS);
}

bool param_exists(const Param_name name)
{
    // Iterate over each of the parameters.
    for (size_t i = 0; i < NUM_PARAMS; i++)
    {
        // Check if the name matches.
        bool match = true;
        for (size_t c = 0; c < MAVLINK_PARAM_NAME_MAX_LENGTH; c++)
        {
            // If the characters in both strings at this position aren't the same, it's not a match.
            if (name[c] != parameters[i].name[c])
            {
                match = false;
                break;
            }

            // If we reach the end of the string, stop.
            if (name[c] == 0) break;
        }
        if (match)
        {
            // This is the right one.
            return true;
        }
    }

    // Else if we get here, we didn't find a match.
    return false;
}

size_t param_index(const Param_name name)
{
    // Iterate over each of the parameters.
    size_t i = 0;
    for (; i < NUM_PARAMS; i++)
    {
        // Check if the name matches.
        bool match = true;
        for (size_t c = 0; c < MAVLINK_PARAM_NAME_MAX_LENGTH; c++)
        {
            // If the characters in both strings at this position aren't the same, it's not a match.
            if (name[c] != parameters[i].name[c])
            {
                match = false;
                break;
            }
            // If we reach the end of the string, stop.
            if (name[c] == 0) break;

        }
        if (match)
        {
            // This is the right one.
            break;
        }
    }

    // Return whatever index we stopped at.
    return i;
}

bool param_set(const size_t index, const Param_value value)
{
    // Check if we're asking for an index which doesn't exist.
    if (index >= NUM_PARAMS)
    {
        return false;
    }

    // Else set the value of the parameter.
    parameters[index].value = value;

    // All done.
    return true;
}

bool param_set(const Param_name name, const Param_value value)
{
    // Iterate over each of the parameters.
    for (size_t i = 0; i < NUM_PARAMS; i++)
    {
        // Check if the name matches.
        bool match = true;
        for (size_t c = 0; c < MAVLINK_PARAM_NAME_MAX_LENGTH; c++)
        {
            // If the characters in both strings at this position aren't the same, it's not a match.
            if (name[c] != parameters[i].name[c])
            {
                match = false;
                break;
            }

            // If we reach the end of the string, stop.
            if (name[c] == 0) break;
        }
        if (match)
        {
            // Set the value of the parameter.
            parameters[i].value = value;
            return true;
        }
    }

    // Else if we get here, we didn't find a match.
    return false;
}

bool is_system_armed()
{
    // Just return the armed flag.
    return ((Mavlink_params::get_driver().fc_base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY) == MAV_MODE_FLAG_SAFETY_ARMED);
}

// IMPLEMENT PUBLIC CLASS FUNCTIONS.

// Param_name

Param_name::Param_name(const char* name)
{
    // Set our name to the specified name.
    safe_strcpy(this->name, name);

    // All done.
    return;
}

Param_name::operator const char*() const
{
    return this->name;
}

// Param_value

Param_value::Param_value() :
type(Param_value::TYPE_NONE),
raw_value(0)
{
    // Nothing to do here.
    return;
}

Param_value::Param_value(Param_value::type_t type, Param_value::value_t raw_value) :
type(type),
raw_value(raw_value)
{
    // Nothing to do here.
    return;
}

template <typename T>
T Param_value::value_to(const Param_value::value_t raw_value)
{
    static_assert(sizeof (T) <= sizeof (Param_value::value_t), "Type provided to Param_value::value_as() must be no greater than 4 bytes.");

    // This operation is safe as long as the size of T is no more than 4 bytes.
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstrict-aliasing"
    return *reinterpret_cast<const T*> (&raw_value);
    #pragma GCC diagnostic pop
}
// Explicitly instantiate the supported types.
template char Param_value::value_to<char>(const Param_value::value_t value);
template uint8_t Param_value::value_to<uint8_t>(const Param_value::value_t value);
template int8_t Param_value::value_to<int8_t>(const Param_value::value_t value);
template uint16_t Param_value::value_to<uint16_t>(const Param_value::value_t value);
template int16_t Param_value::value_to<int16_t>(const Param_value::value_t value);
template uint32_t Param_value::value_to<uint32_t>(const Param_value::value_t value);
template int32_t Param_value::value_to<int32_t>(const Param_value::value_t value);
template float Param_value::value_to<float>(const Param_value::value_t value);

template <typename T>
Param_value::value_t Param_value::value_from(const T value)
{
    static_assert(sizeof (T) <= sizeof (Param_value::value_t), "Type provided to Param_value::value_from() must be no greater than 4 bytes.");

    value_t result = 0;

    const size_t size = sizeof (T);

    // Cast to void* to allow byte level access to the values.
    const void* src = static_cast<const void*> (&value);
    void* dest = static_cast<void*> (&result);

    // Because T can be smaller than value_t, must use memcpy to copy the exact number of bytes required.  A reinterpret cast is unsafe here.
    memcpy(dest, src, size);
    return result;
}
// Explicitly instantiate the supported types.
template Param_value::value_t Param_value::value_from<char>(const char value);
template Param_value::value_t Param_value::value_from<uint8_t>(const uint8_t value);
template Param_value::value_t Param_value::value_from<int8_t>(const int8_t value);
template Param_value::value_t Param_value::value_from<uint16_t>(const uint16_t value);
template Param_value::value_t Param_value::value_from<int16_t>(const int16_t value);
template Param_value::value_t Param_value::value_from<uint32_t>(const uint32_t value);
template Param_value::value_t Param_value::value_from<int32_t>(const int32_t value);
template Param_value::value_t Param_value::value_from<float>(const float value);

template <typename T>
Param_value::type_t Param_value::type_of()
{
    // The function has to be implemented specifically for each of the supported types.
    return Param_value::TYPE_NONE;
}
// Create specialised variant for each supported type.

template <> Param_value::type_t Param_value::type_of<char>()
{
    return Param_value::TYPE_CHAR;
}

template <> Param_value::type_t Param_value::type_of<uint8_t>()
{
    return Param_value::TYPE_UINT8_T;
}

template <> Param_value::type_t Param_value::type_of<int8_t>()
{
    return Param_value::TYPE_INT8_T;
}

template <> Param_value::type_t Param_value::type_of<uint16_t>()
{
    return Param_value::TYPE_UINT16_T;
}

template <> Param_value::type_t Param_value::type_of<int16_t>()
{
    return Param_value::TYPE_INT16_T;
}

template <> Param_value::type_t Param_value::type_of<uint32_t>()
{
    return Param_value::TYPE_UINT32_T;
}

template <> Param_value::type_t Param_value::type_of<int32_t>()
{
    return Param_value::TYPE_INT32_T;
}

template <> Param_value::type_t Param_value::type_of<float>()
{
    return Param_value::TYPE_FLOAT;
}

template <typename T>
T Param_value::as() const
{
    type_t type = type_of<T>();
    if (type != this->type)
    {
        // Something truly terrible is about to happen.  As Jared said: think about the poor life choices that have led us here.
        while (true)
        {
            // Do nothing.
            HAL_Delay(50);
            HAL_GPIO_TogglePin(ERROR1_GPIO_Port, ERROR1_Pin);
        }
    }

    return value_to<T>(this->raw_value);
}
// Explicitly instantiate the supported types.
template char Param_value::as<char>() const;
template uint8_t Param_value::as<uint8_t>() const;
template int8_t Param_value::as<int8_t>() const;
template uint16_t Param_value::as<uint16_t>() const;
template int16_t Param_value::as<int16_t>() const;
template uint32_t Param_value::as<uint32_t>() const;
template int32_t Param_value::as<int32_t>() const;
template float Param_value::as<float>() const;

// Param

Param::Param() :
name(""),
value()
{
    // Nothing to do here.
    return;
}

Param::Param(const Param_name name, const Param_value& value) :
name(name),
value(value)
{
    // Nothing to do here.
    return;
}

Param::Param(const Param& org_param) :
name(org_param.name),
value(org_param.value)
{
    // Nothing to do here.
    return;
}

// Mavlink_params

Mavlink_params::Mavlink_params(void)
{
    // Initially, we won't need to send anything.
    send_param = NUM_PARAMS;
    send_params_from = NUM_PARAMS;
    do_load = false;
    do_store = false;
    send_ack = false;

    // Initialise the driver state machine.
    prev_state = DRIVER_STATE_UNKNOWN;
    this_state = DRIVER_STATE_UNKNOWN;

    // All done.
    return;
}

Mavlink_params::~Mavlink_params(void)
{
    // Nothing to do here.

    // All done.
    return;
}

void Mavlink_params::sync_update_unthrottled()
{
    // NOTE - The housekeeping state machine is called at 1Hz.

    // If we're not in STATE_NORMAL, then we don't do anything yet.
    if (this_state != DRIVER_STATE_NORMAL)
    {
        return;
    }

    // All done.
    return;
}

void Mavlink_params::sync_update_100Hz()
{
    // NOTE - The housekeeping state machine is called at 1Hz.

    // If we're not in STATE_NORMAL, then we don't do anything yet.
    if (this_state != DRIVER_STATE_NORMAL)
    {
        return;
    }
    if (do_send_firmware_version)
    {
        mavlink_message_t msg;
        mavlink_msg_anv_msg_firmware_information_pack(get_sID(), get_cID(), &msg, FIRMWARE_VERSION);
        driverhost_broadcast_mavlink(msg, this);
        do_send_firmware_version = false;
    }
    //Check if we are meant to do a reboot.
    if (do_reboot_to_bootloader)
    {
        //transmit an ACK
        if (do_reboot_to_bootloader_ack)
        {
            mavlink_command_ack_t payload;
            payload.command = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
            payload.result = MAV_RESULT_ACCEPTED;
            mavlink_message_t msg;
            mavlink_msg_command_ack_encode(get_sID(), get_cID(), &msg, &payload);
            driverhost_broadcast_mavlink(msg, this);
            //don't do this again
            do_reboot_to_bootloader_ack = false;
        }

        if (reboot_to_bootloader_timeout_count > 20)
        {
            reboot_to_bootloader(do_reboot_to_bootloader_multi_boot);
        }

        reboot_to_bootloader_timeout_count++;
    }

    // We perform deferred transmission at 100Hz, which should keep the bus reasonably free.

    // Check if we need to send a command ack.
    if (send_ack)
    {
        // Send a response: no need for complications, because we only ever response of one particular command.

        mavlink_command_ack_t payload;
        payload.command = MAV_CMD_PREFLIGHT_STORAGE;
        payload.result = MAV_RESULT_ACCEPTED;

        mavlink_message_t msg;
        mavlink_msg_command_ack_encode(get_sID(), get_cID(), &msg, &payload);

        driverhost_broadcast_mavlink(msg, this);

        // Don't do this again.
        send_ack = false;
    }

    // Check if we've been asked to send a single parameter.
    if (send_param < NUM_PARAMS)
    {
        // Send the specific parameter that's been requested.
        Param& param = parameters[send_param];

        mavlink_param_value_t payload;
        safe_strcpy(payload.param_id, param.name);
        payload.param_type = param.value.type;
        payload.param_count = NUM_PARAMS;
        payload.param_index = send_param;
        payload.param_value = param.value.raw_value;

        mavlink_message_t msg;
        mavlink_msg_param_value_encode(get_sID(), get_cID(), &msg, &payload);

        driverhost_broadcast_mavlink(msg, this);
        // Don't do this again.
        send_param = NUM_PARAMS;
    }
    else if (send_params_from < NUM_PARAMS)
    {
        // Send the parameter which we're up to.
        Param& param = parameters[send_params_from];



        mavlink_param_value_t payload;
        safe_strcpy(payload.param_id, param.name);
        payload.param_type = param.value.type;
        payload.param_count = NUM_PARAMS;
        payload.param_index = send_params_from;
        payload.param_value = param.value.raw_value;

        mavlink_message_t msg;
        mavlink_msg_param_value_encode(get_sID(), get_cID(), &msg, &payload);

        driverhost_broadcast_mavlink(msg, this);

        // Advance to the next parameter.
        send_params_from++;
    }

    // Check if we're supposed to be loading parameters from EEPROM.
    if (do_load)
    {
        // NOTE - This will block for some time.  Interrupts are disabled.  There isn't much you can do about this though.

        // If we try to load parameters and fail, then immediate try to store the parameters: this fixes the parameter table if it's been smashed.
        if (try_load_from_eeprom()) do_store = true;

        // Don't do this again.
        do_load = false;
    }

    // Check if we're supposed to be storing parameters to EEPROM.
    if (do_store)
    {
        do_flash_store(PARAM_COMPAT_VERSION);
        do_store = false;
    }

    // All done.
    return;
}

void Mavlink_params::sync_update_10Hz()
{
    // NOTE - The housekeeping state machine is called at 1Hz.

    // If we're not in STATE_NORMAL, then we don't do anything yet.
    if (this_state != DRIVER_STATE_NORMAL)
    {
        return;
    }

    // All done.
    return;
}

void Mavlink_params::sync_update_1Hz()
{
    // Run the driver housekeeping state machine.

    // If we don't specify a new state, assume we will just remain in the current state.
    Driver_state next_state = this_state;

    // Select behaviour depending on the current state.
    switch (this_state)
    {
        case DRIVER_STATE_UNKNOWN:
        {
            // If we don't know which state we are in, then we probably want to initialise.
            next_state = DRIVER_STATE_INIT;
            break;
        }

        case DRIVER_STATE_INIT:
        {
            // The first time through, we probably want to try to load parameters from EEPROM.
            do_load = true;

            // We're all set up.
            next_state = DRIVER_STATE_NORMAL;

            break;
        }

        case DRIVER_STATE_NORMAL:
        {
            // Nothing to do here.
            break;
        }

        case DRIVER_STATE_ERROR:
        {
            // Nothing to do here.
            break;
        }

        default:
        {
            // We shouldn't ever end up here.
            this_state = DRIVER_STATE_UNKNOWN;
            next_state = DRIVER_STATE_UNKNOWN;
            break;
        }
    }

    // Advance to the next state.
    prev_state = this_state;
    this_state = next_state;

    // All done.
    return;
}

void Mavlink_params::handle_rx_mavlink(const mavlink_message_t& message)
{
    // Select behavior depending on the type of message.
    switch (message.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // This is a hearbeat message.
			mavlink_heartbeat_t payload;
			mavlink_msg_heartbeat_decode(&message, &payload);

			// Check if the message is from something we care about.
			if ((message.sysid == get_sID()) && (message.compid == CID_FC))
			{
				// This is a heartbeat from the flight controller.
				on_mavlink_heartbeat_from_fc(payload);
			}
			break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            // This is a request to send the entire parameter map.
            mavlink_param_request_list_t payload;
            mavlink_msg_param_request_list_decode(&message, &payload);

            // Check that the message is intended for us.
            if ((payload.target_system == get_sID()) && ((payload.target_component == get_cID()) || (payload.target_component == 0)))
            {
                on_mavlink_param_request_list(payload);
            }
            break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
            // This is a request to retrieve a parameter by index or name.
            mavlink_param_request_read_t payload;
            mavlink_msg_param_request_read_decode(&message, &payload);

            // Check that the message is intended for us.
            if ((payload.target_system == get_sID()) && ((payload.target_component == get_cID()) || (payload.target_component == 0)))
            {
                //we can get here on every message sent
                on_mavlink_param_request_read(payload);
            }
            break;
        }

        case MAVLINK_MSG_ID_PARAM_SET:
        {
            // This is a request to set a parameter to a new value.
            mavlink_param_set_t payload;
            mavlink_msg_param_set_decode(&message, &payload);

            // Check that the message is intended for us.
            if ((payload.target_system == get_sID()) && ((payload.target_component == get_cID()) || (payload.target_component == 0)))
            {
                on_mavlink_param_set(payload);
            }
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            // This is a command to the system.
            mavlink_command_long_t payload;
            mavlink_msg_command_long_decode(&message, &payload);

            // Check that the message is intended for us.
            if ((payload.target_system == get_sID()) && ((payload.target_component == get_cID()) || (payload.target_component == 0)))
            {
                on_mavlink_command_long(payload);
            }
        }

        default:
        {
            // We don't care about this message.
            break;
        }
    }

    // All done.
    return;
}

void Mavlink_params::request_data_store(void)
{
  //store all parameters
  do_store = true;
  return;
}
void Mavlink_params::request_data_restore(void)
{
  //destroy the flash storage. so we reload this on next bootup.
  do_flash_store(0x00);
  return;
}

void Mavlink_params::do_flash_store(uint8_t param_version)
{
    HAL_GPIO_WritePin(ERROR1_GPIO_Port, ERROR1_Pin, GPIO_PIN_SET);
    const volatile uint32_t *param_values = (const volatile uint32_t *)(FLASH_PAGE_1);
    EEPROM_Status_t status_block;
    status_block.uint32_block = *param_values;

    /*
     * First two bytes are flag and version
     */
    status_block.uint8_block[0] = EEPROM_FLAG;
    status_block.uint8_block[1] = param_version;
    //increment the counter
    status_block.uint16_block[1]++;


    /**
     * Erase the memory
     */
#ifdef STM32F1XX
#define ERASE_EEPROM_PAGE_DEFINED
      erase_eeprom_page_by_address(FLASH_PAGE_1);
#endif
#if defined(STM32G4XX) || defined(STM32L4XX)
#define ERASE_EEPROM_PAGE_DEFINED
      erase_eeprom_page_by_number(1);
#endif
#ifndef ERASE_EEPROM_PAGE_DEFINED
#error No erase_eeprom_page defined for this chip
#endif
    /**
     * Write the status
     */
    store_in_flash(FLASH_PAGE_1, status_block.uint32_block);

    /**
     * Increment the next page, because we do not want to interfere with our boot options
     */
    uint32_t address = FLASH_PAGE_1 + sizeof (uint32_t);
    /*
     * Write parameters at the start of the next word
     */
    address += sizeof (uint32_t);
    for (uint8_t i = 0; i < NUM_PARAMS; i++)
    {
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wstrict-aliasing"
        uint32_t data = *reinterpret_cast<uint32_t*> (&(parameters[i].value.raw_value));
      #pragma GCC diagnostic pop
        store_in_flash(address, data);
        address += sizeof (uint32_t);
    }

    HAL_GPIO_WritePin(ERROR1_GPIO_Port, ERROR1_Pin, GPIO_PIN_RESET);
}


#ifdef STM32F1XX
/**
 * Erases a specified page at given address
 * @param page number to erase
 */
void Mavlink_params::erase_eeprom_page_by_address(uint32_t address)
{
    /**
     * Unlock flashing ability
     */
    HAL_FLASH_Unlock();
    uint32_t PageError = 0;
    FLASH_EraseInitTypeDef pErase;
    pErase.NbPages = 1; //single page
    pErase.PageAddress = address;
    pErase.Banks = FLASH_BANK_1;
    pErase.TypeErase = FLASH_TYPEERASE_PAGES;
    /**
     * Perform erase
     */
    HAL_FLASHEx_Erase(&pErase, &PageError);
    /**
     * Lock Flash ability
     */
    HAL_FLASH_Lock();
}
#endif

#if defined(STM32G4XX) || defined(STM32L4XX)
/**
 * Erases a specified page at given page number
 * @param page number to erase
 */
void Mavlink_params::erase_eeprom_page_by_number(uint32_t page)
{
    /**
     * Unlock flashing ability
     */
    HAL_FLASH_Unlock();
    uint32_t PageError = 0;
    FLASH_EraseInitTypeDef pErase;
    pErase.NbPages = 1; //single page
    pErase.Page = page;
    pErase.Banks = FLASH_BANK_1;
    pErase.TypeErase = FLASH_TYPEERASE_PAGES;
    /**
     * Perform erase
     */
    HAL_FLASHEx_Erase(&pErase, &PageError);
    /**
     * Lock Flash ability
     */
    HAL_FLASH_Lock();
}
#endif

/**
 * Stores 4 bytes (1 mavlink parameter' raw value) of data in flash memory as specified
 * @param address
 * @param data
 */
void Mavlink_params::store_in_flash(uint32_t address, uint32_t data)
{
    /**
     * Start by unlocking the flash partition
     */
    HAL_FLASH_Unlock();

    /**
     * Write the 32Bit data word
     */
#ifdef STM32F1XX
#define STM_FOUND_FLASH
    	HAL_FLASH_Program(FLSH_TYPEPROGRAM_WORD, address, data);
#endif
#if defined(STM32G4XX) || defined(STM32L4XX)
#define STM_FOUND_FLASH
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, address, data);
#endif
#ifndef STM_FOUND_FLASH
#error No HAL_FLASH_Program defined for this chip
#endif

    /**
     * End by locking the flash partition
     */
    HAL_FLASH_Lock();
}

bool Mavlink_params::try_load_from_eeprom(void)
{
    // NOTE - This will block for some time.  Interrupts are disabled.  There isn't much you can do about this though.
    bool no_parameters = true;
    /**
     * Read the first 32 bytes of Flash storage.
     */
    const volatile uint32_t *param_values = (const volatile uint32_t *)(FLASH_PAGE_1);
    EEPROM_Status_t status_block;
    status_block.uint32_block = *param_values;
    /*
     * First two bytes are flag and version
     */
    uint8_t flag = status_block.uint8_block[0];
    uint8_t version = status_block.uint8_block[1];
    /**
     * The next two bytes are a counter to check how many writes are performed
     */
    //uint16_t storage_counter = status_block.uint16_block[1];
    if (flag == EEPROM_FLAG && version == PARAM_COMPAT_VERSION)
    {
        //move onto the first param (the boot flags)
        param_values++;
        //move onto the second parameter
        param_values++;
        for (uint8_t i = 0; i < NUM_PARAMS; i++)
        {
            uint32_t* data = reinterpret_cast<uint32_t*> (&(parameters[i].value.raw_value));
            *data = *param_values;
            param_values++; //= sizeof (uint32_t);
        }
        no_parameters = false;
    }
    // All done.
    return no_parameters;
}

void Mavlink_params::on_mavlink_param_request_list(mavlink_param_request_list_t& payload)
{
    // Schedule sending all the parameters one after another, starting with the first parameter.
    send_params_from = 0;

    // All done.
    return;
}

void Mavlink_params::on_mavlink_param_request_read(mavlink_param_request_read_t& payload)
{
    // Check if we're requesting by name or ID.
    if (payload.param_index < 0)
    {
        // The index is -1, so we're looking up by name.

        // Try to fetch the index of the requested parameter.
        size_t i = param_index(payload.param_id);
        if (i == NUM_PARAMS)
        {
            // We don't have a parameter with that name.
            return;
        }

        // Else we found a corresponding index, so schedule sending that parameter.
        send_param = i;
    }
    else if (payload.param_index < static_cast<int16_t> (NUM_PARAMS))
    {
        // The index is legit, so we're looking up by index.
        send_param = payload.param_index;
    }

    // Else, we requested an index which didn't exist, so nothing to do.
    return;
}

void Mavlink_params::on_mavlink_param_set(mavlink_param_set_t& payload)
{
    // NOTE - Unlike requests, setting parameter values always happens by ID (not index).

    // Try to fetch the index of the requested parameter.
    size_t i = param_index(payload.param_id);
    if (i == NUM_PARAMS)
    {
        // We don't have a parameter with that name.
        return;
    }
    //check to see the parameter type we are receiving is actually the type we have been asked to care about:
    //This breaks some things further on if we have been asked to recieve a parameter of a certain type, but have just overwritten that parameter type with whatever this is.
    //Given the two types will match up if they are supported, anything that is wrong, or unsuported will cause this to fail (I am looking at you CHAR, and undefined).
    //Conveniently CHAR is not supported here, so if param_fetch fails, this whole thing will fail
    if((Param_value::type_t)payload.param_type == param_fetch(i, Param_value::from<char>(0)).type)
    {
        // All happy happy, set the value of the parameter.
        parameters[i].value = Param_value(static_cast<Param_value::type_t> (payload.param_type), payload.param_value);
    }


    // We always finish by sending the value (changed or not).
    send_param = i;

    // All done.
    return;
}

void Mavlink_params::on_mavlink_command_long(mavlink_command_long_t& payload)
{
    // Depending on what type of command this actually is, do something.

    switch (payload.command)
    {
        case MAV_CMD_USER_1:
        {
            switch ((uint16_t) payload.param4)
            {
                case 1:
                {
                    do_send_firmware_version = true;
                    break;
                }
            }
            break;
        }
        case MAV_CMD_PREFLIGHT_STORAGE:
        {
            // TODO - param1 is a float: does this even work?

            // The command is either to load (0), or store (1).
            switch ((uint16_t) payload.param1)
            {
                case 0:
                {
                    // Schedule load parameters from EEPROM.
                    do_load = true;
                    send_ack = true;
                    break;
                }

                case 1:
                {
                    // Schedule store parameters to EEPROM.
                    do_store = true;
                    send_ack = true;
                    break;
                }

                default:
                {
                    // Otherwise, do nothing.
                    break;
                }
            }
            break;
        }

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        {
            // TODO - param1 is a float: does this even work?

            // We ignore param #1, because it's for the 'flight controller', and we use #2, for the 'computer'.

            // The command is either do nothing (0), reboot (1), shutdown (2), or reboot to bootloader (3).
            switch ((uint16_t) payload.param2)
            {
                case 0:
                {
                    // Nothing to do.
                    break;
                }

                case 1:
                {
                    // Reboot the microcontroller.
                    reboot();

                    // We'll never make it to here.
                    break;
                }

                case 2:
                {
                    // We can't shut down, so just reboot the microcontroller.
                    reboot();

                    // We'll never make it to here.
                    break;
                }

                case 3:
                {
                    // Reboot the microcontroller to the bootloader.
                    do_reboot_to_bootloader = true;
                    // We'll never make it to here.
                    break;
                }
                case 4:
                {
                    // Reboot the microcontroller to the bootloader.
                    do_reboot_to_bootloader = true;
                    do_reboot_to_bootloader_multi_boot = true;
                    // We'll never make it to here.
                    break;
                }
                case 5:
                {
                    /**
                     * force a reset of the parameters.
                     */
                    do_flash_store(0x00);
                    /**
                     * Reboot so we load the new parameters
                     */
                    reboot();

                    break;
                }

                default:
                {
                    // Otherwise, do nothing.
                    break;
                }
            }
            break;
        }

        default:
        {
            // Otherwise, it's not a command we care about, so do nothing.
            break;
        }
    }

    // All done.
    return;
}

void Mavlink_params::on_mavlink_heartbeat_from_fc(mavlink_heartbeat_t& payload)
{
    // Extract the status information from the heartbeat message.
    fc_base_mode = payload.base_mode;
    fc_custom_mode = payload.custom_mode;
    fc_system_status = payload.system_status;

    // All done.
    return;
}

void Mavlink_params::reboot()
{
    NVIC_SystemReset();
    // We'll never make it to here.
    return;
}

void Mavlink_params::reboot_to_bootloader(bool multiboot)
{
    //update the vairables at stake;
    //It is intentional that the address is hardwired here - the bootloader just looks at this address

    //Tell the bootloader to boot into bootloader mode.
    //volatile uint8_t *do_bootloader = (volatile uint8_t *)(RAM_BOOTLOADER_ACTION_LOCATION);
    bootloader_info_location = RAM_DO_BOOTLOADER_BYTE;

    //restart the MC
    reboot();
    return;
}

// IMPLEMENT PRIVATE STATIC FUNCTIONS.

void safe_strcpy(char* dst, const char* src)
{
    // Figure out how long the provided string is.
    size_t i;
    for (i = 0; i < MAVLINK_PARAM_NAME_MAX_LENGTH; i++)
    {
        // If we reach a null, then we've finished.
        if (src[i] == 0) break;
    }

    // Copy the name, now that we know how long it is.
    memcpy(dst, src, i);

    // Make sure we've got a termination if required.
    if (i < MAVLINK_PARAM_NAME_MAX_LENGTH) dst[i] = 0;

    // All done.
    return;
}

// IMPLEMENT PRIVATE CLASS FUNCTIONS.

// IMPLEMENT INTERRUPT SERVICE ROUTINES.

// ALL DONE.

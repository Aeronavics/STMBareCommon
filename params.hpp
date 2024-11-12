/********************************************************************************************************************************
 *
 *  FILE:           params.hpp
 *
 *  SUB-SYSTEM:     Avionics
 *
 *  COMPONENT:      Payload Controller
 *
 *  TARGET:         AT90CAN128
 *
 *  PLATFORM:       BareMetal
 *
 *  AUTHOR:         Edwin Hayes
 *
 *  DATE CREATED:   16-08-2016
 *
 *	This is the header file which matches params.cpp.  Device driver which provides a library for MAVlink parameter
 *  management using static compile time parameter definitions.
 *
 *  Copyright 2016 Aeronavics Ltd
 *
 ********************************************************************************************************************************/

// TODO - Change the comment block to the one meant for library files.

// Only include this header file once.
#pragma once

// INCLUDE REQUIRED HEADER FILES FOR INTERFACE.

// Include the required IO header file.
//#include <<<TC_INSERTS_IO_FILE_NAME_HERE>>>

// Include the STDINT fixed width types.
#include <stdint.h>

// Include the generic driver module template.
#include "driver_module.hpp"

// Include the all important MAVlink library.
#include "mavlink.hpp"
//this is included for the map to system flash macro it provides.
#include "Legacy/stm32_hal_legacy.h"


// Include the hal libraries required for operation
//flash library is important

#include "config.h"
#include STM_HAL
#include "iwdg.h"
#include STM_HAL_IWDG
// #include "i2c.h"
#include STM_HAL_FLASH

// #ifndef I2C_MODULE
// #define I2C_MODULE hi2c1
// #endif
#ifndef EEPROM_ADDRESS
#define EEPROM_ADDRESS (0b10100000 )//>>3)
#endif

#define NUMBER_OF_FLASH_PAGES 5
#define NUMBER_OF_WRITE_CYCLES 1000
#define FLASH_SIZE 2000

#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08004000) /* Base @ of Page 8, 2 Kbytes */
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08004800) /* Base @ of Page 9, 2 Kbytes */
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08005000) /* Base @ of Page 10, 2 Kbytes */
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08005800) /* Base @ of Page 11, 2 Kbytes */
#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08006000) /* Base @ of Page 12, 2 Kbytes */
#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08006800) /* Base @ of Page 13, 2 Kbytes */
#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08007000) /* Base @ of Page 14, 2 Kbytes */
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08007800) /* Base @ of Page 15, 2 Kbytes */
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08008000) /* Base @ of Page 16, 2 Kbytes */
#define ADDR_FLASH_PAGE_17    ((uint32_t)0x08008800) /* Base @ of Page 17, 2 Kbytes */
#define ADDR_FLASH_PAGE_18    ((uint32_t)0x08009000) /* Base @ of Page 16, 2 Kbytes */
#define ADDR_FLASH_PAGE_19    ((uint32_t)0x08009800) /* Base @ of Page 17, 2 Kbytes */
#define ADDR_FLASH_PAGE_20    ((uint32_t)0x0800A000) /* Base @ of Page 16, 2 Kbytes */
#define ADDR_FLASH_PAGE_21    ((uint32_t)0x0800A800) /* Base @ of Page 17, 2 Kbytes */
#define ADDR_FLASH_PAGE_22    ((uint32_t)0x0800B000) /* Base @ of Page 16, 2 Kbytes */
#define ADDR_FLASH_PAGE_23    ((uint32_t)0x0800B800) /* Base @ of Page 17, 2 Kbytes */
#define ADDR_FLASH_PAGE_24    ((uint32_t)0x0800C000) /* Base @ of Page 16, 2 Kbytes */
#define ADDR_FLASH_PAGE_25    ((uint32_t)0x0800C800) /* Base @ of Page 17, 2 Kbytes */
#define ADDR_FLASH_PAGE_26    ((uint32_t)0x0800D000) /* Base @ of Page 16, 2 Kbytes */
#define ADDR_FLASH_PAGE_27    ((uint32_t)0x0800D800) /* Base @ of Page 17, 2 Kbytes */
#define ADDR_FLASH_PAGE_104   ((uint32_t)0x08034000) /* Base @ of Page 17, 2 Kbytes */

#define FLASH_PAGE_1 ADDR_FLASH_PAGE_104
#define FLASH_PAGE_2 ADDR_FLASH_PAGE_21
#define FLASH_PAGE_3 ADDR_FLASH_PAGE_22
#define FLASH_PAGE_4 ADDR_FLASH_PAGE_23
#define FLASH_PAGE_5 ADDR_FLASH_PAGE_24

extern uint8_t bootloader_info_location;
#define RAM_BOOTLOADER_ACTION_LOCATION 0x20000000
#define RAM_FASTBOOT_BYTE   0xAB //if this is set, it indicates a fastboot (which incidentaly is default)
#define RAM_DO_BOOTLOADER_BYTE   0x22 //if this is set, it indicates we should deliberatlty go into bootloader
#define EEPROM_FLAG 0xAA
#define MULTI_BOOTLOADER 0x12
#define ONETIME_BOOTLOADER 0x71
#define BOOTLOADER_ADDRESS 0x00
#define I2C_TIMEOUT 1000
// DEFINE PUBLIC MACROS.

#define MAVLINK_PARAM_NAME_MAX_LENGTH 16

// FORWARD DEFINE PRIVATE PROTOTYPES.

// DEFINE PUBLIC CLASSES, TYPES AND ENUMERATIONS.

union EEPROM_Status_t {
    uint32_t uint32_block;
    uint16_t uint16_block[2];
    uint8_t uint8_block[4];
};

class Param_name {
public:

    // Fields.

    char name[MAVLINK_PARAM_NAME_MAX_LENGTH];

    // Methods.

    Param_name(const char* name);

    operator const char*() const;
};

/**
 * A generic MAVlink numeric parameter value.
 */
class Param_value {
public:

    // Fields.

    // For some reason MAVlink uses float as its base type (size is 4 bytes).
    typedef float value_t;

    // Supported parameter value types.
    //These types should match up with the mavlink PARAM_TYPE:
    //https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
    //This allows casting between the two.
    //On the stm32's please do not use the 64bit types, as we have only catered to storing them in 32 bit fields.

    typedef enum {
        TYPE_CHAR = 0,      
        TYPE_UINT8_T = 1,   
        TYPE_INT8_T = 2,    
        TYPE_UINT16_T = 3,  
        TYPE_INT16_T = 4,   
        TYPE_UINT32_T = 5,  
        TYPE_INT32_T = 6,   
        //TYPE_UINT64_T = 7,
        //TYPE_INT64_T = 8,
        TYPE_FLOAT = 9,     //technically float32
        //TYPE_FLOAT64_t = 10,
        TYPE_NONE = 255,
    } type_t;

    type_t type;
    value_t raw_value;

    // Methods.

    /**
     * Default constructor.
     */
    Param_value();

    /**
     * Create a new parameter value of the given type and value.
     *
     * NOTE - Be careful: raw_value may be implicity cast to value_t, which is probably not what you want!  Consider using Param_value::from<T>() instead.
     *
     * @param type		The type of the parameter (eg. Param_value::TYPE_FLOAT).
     * @param raw_value The raw value.  If the type is not TYPE_FLOAT, this will not properly represent the value you want.
     */
    Param_value(type_t type, value_t raw_value);

    /**
     * Construct a new Param_value of the specified type.
     *
     * NOTE - Behavior, if the specified type is not one of the types supported according to the type_t enum above, is undefined.
     *
     * @param value The numeric value.
     * @return		A new Param_value object of the specified type and value.
     */
    template <typename T>
    static Param_value from(T value) {
        return Param_value(type_of<T>(), value_from<T>(value));
    }

    /**
     * Safe cast from a raw value_t to the templated type.
     *
     * NOTE - Behavior, if the specified type is not one of the types supported according to the type_t enum above, is undefined.
     *
     * @param raw_value	The raw value to convert.
     * @return			The converted value of the correct type.
     */
    template <typename T>
    static T value_to(const value_t raw_value);

    /**
     * Safe cast from the templated type to a raw value_t.
     *
     * NOTE - Behavior, if the specified type is not one of the types supported according to the type_t enum above, is undefined.
     *
     * @param value		The walue to convert.
     * @return			The raw value.
     */
    template <typename T>
    static value_t value_from(const T value);

    /**
     * Returns the value of Param_value::type_t for the specified type.  For example, Param_value::type_of<uint8_t> will return the value Param_value::TYPE_UINT8_T.
     *
     * NOTE - Unsupported types return Param_value::TYPE_NONE.
     *
     * @return The corresponding value of the Param_value::type_t enum.
     */
    template <typename T>
    static type_t type_of();

    /**
     * Convert this Param_value to a specific type.
     *
     * NOTE - Behavior, if the specified type is not one of the types supported according to the type_t enum above, is undefined.
     *
     */
    template <typename T>
    T as() const;
};

/**
 * A named MAVlink numeric parameter.
 *
 */
class Param {
public:

    // Fields.

    Param_name name;
    Param_value value;

    // Methods.

    /**
     * Default constructor.
     *
     * Constructs an empty parameter (blank name, null value).
     */
    Param();

    /**
     * Constructs a new parameter with the given name and value.
     *
     * @param name	The name of the parameter.
     * @param value The value of the parameter.
     */
    Param(const Param_name name, const Param_value& value);

    /**
     * Construct a new parameter by copying an existing parameter.
     *
     * @param org_param	The original parameter we want to make a copy of.
     */
    Param(const Param& org_param);
};

class Mavlink_params : public Driver_module {
public:

    // Fields.

    // Methods.

    // Inherited from Driver_module.
    void sync_update_unthrottled();
    void sync_update_100Hz();
    void sync_update_10Hz();
    void sync_update_1Hz();
    void handle_rx_mavlink(const mavlink_message_t& message);

    /**
     * Return an instance to the singleton instance of the driver.
     *
     * @param Nothing.
     * @return The singleton instance of the driver.
     */
    static Mavlink_params& get_driver(void);

    /**
     * Free the driver instance when it goes out of scope.
     *
     * @param	Nothing.
     * @return	Nothing.
     */
    ~Mavlink_params(void);

    /**
     * Try to load parameters from EEPROM.  This method is provided because parameters need to be loaded before the driver state machines
     * begin running, since there may be parameter values required for initial configuration.
     *
     * @param	Nothing.
     * @param	False if parmeters were loaded, true otherwise.
     */
    bool try_load_from_eeprom(void);
    /**
     * Tell the params driver that another module wants the parameters to be stored.
     * The typical usage is that CAN has updated some parameters and wants the MAVLINK parameters stored too.
     * @param nothing
     * @return nothing
    */
    void request_data_store(void);
    /**
    * Tells the params that we want to erase back to default data. This is achieved by overriding the default parameter id and restarting.
    * @param nothing
    * @returns nothing
    */
    void request_data_restore(void);

private:

    // Fields.

    // Indices of parameters to send, either individually, or in turn.
    size_t send_param;
    size_t send_params_from;

    // Flags indicating to perform EEPROM operations.
    bool do_store;
    bool do_load;
    bool send_ack;
    bool do_reboot;
    bool do_reboot_to_bootloader;
    bool do_reboot_to_bootloader_multi_boot;
    bool do_reboot_to_bootloader_ack;
    bool do_send_firmware_version;
    uint16_t reboot_to_bootloader_timeout_count;

    Driver_state prev_state;
    // Driver_state this_state;

    uint8_t fc_base_mode;
    uint32_t fc_custom_mode;
    uint32_t fc_system_status;

    // Methods.

    Mavlink_params(void);

    Mavlink_params(Mavlink_params const&); // Poisoned.

    void operator=(Mavlink_params const&); // Poisoned.

    void on_mavlink_param_request_list(mavlink_param_request_list_t& payload);

    void on_mavlink_param_request_read(mavlink_param_request_read_t& payload);

    void on_mavlink_param_set(mavlink_param_set_t& payload);

    void on_mavlink_command_long(mavlink_command_long_t& payload);

    void on_mavlink_heartbeat_from_fc(mavlink_heartbeat_t& payload);

    void do_flash_store(uint8_t param_version);

    void reboot();

    void reboot_to_bootloader(bool multiboot);

    void store_in_flash(uint32_t address, uint32_t data);

    void erase_eeprom_page_by_address(uint32_t address);

    void erase_eeprom_page_by_number(uint32_t page);

    friend bool is_system_armed();
};

// DECLARE PUBLIC GLOBAL VARIABLES.

// DEFINE PUBLIC STATIC FUNCTION PROTOTYPES.

const Param_value param_fetch(const size_t index, const Param_value def);

const Param_value param_fetch(const Param_name name, const Param_value def);

const Param_name param_name_fetch(const size_t index);

bool param_exists(const size_t index);

bool param_exists(const Param_name name);

size_t param_index(const Param_name name);

bool param_set(const size_t index, const Param_value value);

bool param_set(const Param_name name, const Param_value value);

bool is_system_armed();

// ALL DONE.

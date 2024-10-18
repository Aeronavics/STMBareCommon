/********************************************************************************************************************************
 *
 *  FILE:           driver.hpp
 *
 *  SUB-SYSTEM:		Avionics Baseboard
 *
 *  COMPONENT:		baseboard_peripheral
 *
 *  TARGET:         ATmega328p
 *
 *  PLATFORM:       BareMetal
 *
 *  AUTHOR:         Edwin Hayes
 *
 *  DATE CREATED:   13-10-2015
 *
 * Header only class which provides an abstraction of a driver module for baremetal AVR components.  Also provides definitions
 * of functions which are provided by the driver host, and which may be called by driver modules.
 *
 ********************************************************************************************************************************/

// TODO - Change the comment block to the one meant for library files.

// Only include this header file once.
#pragma once

// INCLUDE REQUIRED HEADER FILES FOR INTERFACE.

// Include the required IO header file.
//#include <<<TC_INSERTS_IO_FILE_NAME_HERE>>>

// Include the STDINT fixed width types.
#include <<<TC_INSERTS_STDINT_FILE_NAME_HERE>>>

// Include the all important MAVlink library.
#ifndef NO_MAVLINK_ENABLED
#include "stm_bare_common/mavlink.hpp"
#endif

#ifdef LIBCANARD_ENABLED
#define CAN_TRANSFER_PRIORITY_HIGHEST            0
#define CAN_TRANSFER_PRIORITY_HIGH               8
#define CAN_TRANSFER_PRIORITY_MEDIUM             16
#define CAN_TRANSFER_PRIORITY_LOW                24
#define CAN_TRANSFER_PRIORITY_LOWEST             31

#include "canard.h"

#endif

// FORWARD DEFINE PRIVATE PROTOTYPES.

// DEFINE PUBLIC CLASSES, TYPES AND ENUMERATIONS.

enum Driver_state {
    DRIVER_STATE_UNKNOWN, DRIVER_STATE_INIT, DRIVER_STATE_NORMAL, DRIVER_STATE_ERROR
};

enum Driver_event_severity {
    DRIVER_EVENT_WARNING, DRIVER_EVENT_ALARM, DRIVER_EVENT_CRITICAL
};

enum Uart_module_t {
    UART_MODULE_UART_1 = 1,
    UART_MODULE_UART_2 = 2,
    UART_MODULE_UART_3 = 3,
    UART_MODULE_UART_4 = 4,
    UART_MODULE_UART_5 = 5,
};
typedef uint8_t Driver_event_code;

class Driver_module {
public:

    // Fields.

    // Methods.

    // NOTE - Driver modules are always singletons.  They should generally also have a method like: static Driver_module::get_driver();

    // NOTE - There is no init() method defined.  Generally, drivers should implement state machines that take care of this sort of thing.

    /**
     * Perform synchronous housekeeping for the driver; this method gets called as fast as possible..
     *
     * @param	Nothing.
     * @return	Nothing.
     */
    virtual void sync_update_unthrottled();

    /**
     * Perform synchronous housekeeping for the driver; this method gets called at 100Hz.
     *
     * @param	Nothing.
     * @return	Nothing.
     */
    virtual void sync_update_100Hz();

    /**
     * Perform synchronous housekeeping for the driver; this method gets called at 100Hz.
     * The calling driverhost should increment this number so the driver module can stagger processing if required
     * This is handy when transmitting data at high rates, or in large quantities.
     *
     * If in implemented, the compiler should remove the reference
     * @param	uint8_t iterator.
     * @return	Nothing.
     */
    virtual void sync_update_100Hz(uint8_t iterator) {
        return;
    }

    /**
     * Perform synchronous housekeeping for the driver; this method gets called at 10Hz.
     *
     * @param	Nothing.
     * @return	Nothing.
     */
    virtual void sync_update_10Hz();

    /**
     * Perform synchronous housekeeping for the driver; this method gets called at 1Hz.
     *
     * @param	Nothing.
     * @return	Nothing.
     */
    virtual void sync_update_1Hz();

    /**
     * Handle a MAVlink message which has been received.
     *
     * @param message The MAVlink message which has been received.
     * @param Nothing.
     */
#ifndef NO_MAVLINK_ENABLED
    virtual void handle_rx_mavlink(const mavlink_message_t& message)
    {
      return;
    }
#endif
  /**
   * Handle a CAN message which has been received. If you do not want anything to do with can
   This can safely be ignored, as it has no special significance, or calling types.
   *
     @param transfer the pointer to a transfer created when a message is received. Other modules require this. This will be nullptr for other generated messages.
   * @param data_type_signature signature of the data type. This is pulled from the dsdl generated files
     @param data_type_id data type of the message. This is pulled from the dsdl generated files
     @param priority Priority of the message. 0 is max and 32 is minimul priority
     @param inout_transfer_id Pointer to a permanent instance of the transfer ID. This must be maintained from the calling module
     @param payload pointer to the actual CAN Payload.
     @param payload_len length of payload in bytes
     @return nothing
   */
   #ifdef LIBCANARD_ENABLED
    virtual void handle_rx_can(const CanardRxTransfer * transfer, uint64_t data_type_signature, uint16_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len)
    {
      return;
    }
  #endif

    /**
     * Returns a pointer to the current state of the CAN driver. This is used in the libuavcan module
     *
     * @param Nothing
     * @return instance to the CAN module defined in main.
     */
    virtual Driver_module * get_can_driver(void) {
        return nullptr;
    }

    /**
     * Return the current state of the driver.
     *
     * @return	The current state of the driver.
     */
    Driver_state get_state() const {
        return this_state;
    }

protected:

    // Fields.

    Driver_state this_state;

    // Methods.

private:

    // Fields.

    // Methods.

    void operator=(Driver_module const&); // Poisoned.
};

// DECLARE PUBLIC GLOBAL VARIABLES.

// DEFINE PUBLIC STATIC FUNCTION PROTOTYPES.
/**
 * Returns a pointer to the first CAN driver available
 * else returns NULLPTR
 */
Driver_module * driverhost_get_can_module(void);

/**
 * Returns the current monotonic time duration in microseconds
 */
uint64_t driverhost_get_monotonic_time_us(void);

/**
 * Broadcast a MAVlink message to be handled by other driver modules.
 *
 * @param message The MAVlink message to be broadcast.
 * @param except Address of a driver module to which the message should not be broadcast.  Helpful for preventing loops.
 * @return Nothing.
 */
#ifndef NO_MAVLINK_ENABLED
void driverhost_broadcast_mavlink(const mavlink_message_t& message, Driver_module * const except);
#endif

/**
 * Broadcast a CAN message to be handled by other driver modules.
 *
  @params are the same as the matching handle_rx_can
  @return nothing
 */
#ifdef LIBCANARD_ENABLED
void driverhost_broadcast_can(const CanardRxTransfer * transfer, uint64_t data_type_signature, uint16_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len, Driver_module * const except);
#endif
/**
 *
 * @param message
 * @param size
 * @param target_module
 */
void driverhost_broadcast_rx_raw(const uint8_t character, Uart_module_t origin_module);
/**
 * Outputs a uart message to a target uart modules
 * @param message
 * @param size
 */
void driverhost_uart_tx_raw(const uint8_t * message, uint16_t size, Uart_module_t target_module);

/**
 * Report an event.  Usually this means some kind of error condition (and usually this results in flashing an LED).
 *
 * @param severity The severity of the event which has occurred.
 * @param event_code The event code to report.
 * @return Nothing.
 */
void driverhost_report_event(Driver_event_severity severity, Driver_event_code event_code);

// ALL DONE.

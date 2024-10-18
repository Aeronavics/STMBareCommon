/********************************************************************************************************************************
 *
 *  FILE:           uart_mavlink.hpp
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
 *	This is the header file which matches uart_mavlink.cpp.  Device driver which controls the a UART for communicating with the
 *  rest of the aircraft avionics via MAVlink.
 *
 *  Copyright 2016 Aeronavics Ltd
 *
 ********************************************************************************************************************************/

// Only include this header file once.
#pragma once

// INCLUDE REQUIRED HEADER FILES FOR INTERFACE.

// Include the required IO header file.
// #include <<<TC_INSERTS_IO_FILE_NAME_HERE>>>

// Include the STDINT fixed width types.
#include <stdint.h>

// Include the generic driver module template.
#include "driver_module.hpp"

// Include the hal libraries required for operation
//Include the HAL

#include "config.h"
#include STM_HAL
#include "usart.h"
#include "gpio.h"

// #include "mavlink_types.hpp"

// Include the all important MAVlink library.
#include "mavlink.hpp"

// Include Smart Port
#include "FrSky_Smart_Port.h"

// DEFINE PUBLIC MACROS.

#define TX_QUEUE_LENGTH 20 //rather large buffer
#define UART_MAVLINK_RX_CHARS_RECEIVED_LENGTH 10
#define RX_QUEUE_LENGTH 400 //Used as the length of the rx buffer.
#define PROCESSING_LIMIT 100 //Limit the processing to 100 characters at a time. This is used to prevent overflow of the input data. I.e. if Input data rate ~= processing data rate we can potentially spend all our time processing data

#define MAX_MILLISECS_SINCE_RX 1500
#define RAW_SBUS_DATA_LENGTH 25
#define SBUS_MAGIC_START_BYTE 0x0F

// FORWARD DEFINE PRIVATE PROTOTYPES.

// DEFINE PUBLIC CLASSES, TYPES AND ENUMERATIONS.

struct Uart_queue_t {
    uint16_t length;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
};

struct Buffer_status_t {
    uint16_t current_buffer_fullness;
    uint16_t mavlink_errors;
    uint16_t buffer_overrun;
};

/*
 * Sets the UART type. This handles how the data reception is handled.
 * Mavlink gets parsed into mavlink containers.
 * SBUS gets parsed into sbus containers
 * NORMAL Just gets parsed into a buffer
 * Smart Port ???
 */
enum UART_DATA_Type_t {
    UART_DATA_TYPE_MAVLINK,
    UART_DATA_TYPE_SBUS,
    UART_DATA_TYPE_DATA,
    UART_DATA_TYPE_SMART_PORT
};

enum SBUS_STATUS_t {
    SBUS_STATUS_UNSYNCED,
    SBUS_STATUS_SYNCED
};

enum SMART_PORT_STATUS_t {
    SMART_PORT_STATUS_UNSYNCED,
    SMART_PORT_STATUS_SYNCED
};

enum UART_TRAMSMIT_TYPE_t {
    UART_TRAMSMIT_TYPE_INTERRUPT,
    UART_TRAMSMIT_TYPE_DMA
};

class Uart_module : public Driver_module {
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
     * Queue a message to transmit. This is typically arbitrary data
     * @param message
     * @param size
     */
    void queue_tx_raw(const uint8_t * message, uint16_t size);
    /**
     * Return an instance to the singleton instance of the driver.
     *
     * @param Nothing.
     * @return The singleton instance of the driver.
     */
    // static Uart_module& get_driver(void);
    Uart_module(void);
    // Uart_module(Uart_module const&);		// Poisoned.
    /**
     * Free the driver instance when it goes out of scope.
     *
     * @param	Nothing.
     * @return	Nothing.
     */
    ~Uart_module(void);
    /**
     *set the uart to be handled by this instance
     * @param the address of the uart object. This is globaly defined in usart.h
     * @param the uart module number. This enum is defined gobally here and identifies the preceding uart module before it is initialised
     * @param the mavlink channel that should be used for decoding.
     */
    void set_uart_module(UART_HandleTypeDef * uart, Uart_module_t uart_module_type, bool idle_processing, mavlink_channel_t mav_channel);


    /**
     * Indicate whether the UART is ready to send a message asynchronously (or whether sending will block).
     *
     * @param	Nothing.sd
     * @return	True if the UART is not busy transmitting.
     */
    bool ready_to_send() const;
    /*
     *	retuns the size of the above buffer
     */

    //returns true if the module is the one in question
    bool is_uart(Uart_module_t uart_module);

    /**
     * Sets the number of bytes to receive before interrupting
     */
    void set_receive_bytes(uint16_t bytes_to_receive);
    void set_uart_type(UART_DATA_Type_t uart_type_to_set);

    void callback_rx_complete(void);
    void callback_rx_idle(bool restart_interrupt);
    void callback_tx_complete(void);
    void callback_tx_half_complete(void);
    void reset_uart(void);
    void init_uart(void);
    void enable_idle_callback(void);
    void disable_idle_callback(void);
    //create headers and tails

    Uart_queue_t current_queue;
    Buffer_status_t tx_buffer_status;
    Buffer_status_t rx_buffer_status;
    /**
     * Sets or resets the ability to restart the uart when a counter has timed out.
     * If you are transmitting only we do not care for this feature
     * @param is_timeout_relevant
     */
    void set_is_timeout_relevant(bool is_timeout_relevant);
    /**
     * Sets the transmit type to use. Typically you would want to use DMA
     * Execept on peripherals that do not use it.
     * @param transmit_type
     */
    void set_transmit_type(UART_TRAMSMIT_TYPE_t transmit_type);
    //sets a filter list. Messages matching this msg ID will not be output on this uart.
    void set_filter_list(uint32_t * filter, uint8_t size);
    void set_whitelist_filter_list(uint32_t * filter, uint8_t size);
    void register_smart_port_poll_packet(uint8_t packet_id);

    // void set_instance(Uart_module * instance_address);

private:

    // Fields.
    //INSTANCING VARIABLES
    mavlink_channel_t MAVLINK_CHANNEL;
    UART_HandleTypeDef * UART_MODULE;
    Uart_module_t uart_type;

    uint8_t get_data(void);
    bool is_rx_data_available(void);
    /**
     * Seconds between messages before timeout things occur
     */
    uint32_t millisecs_since_rx;

    volatile bool timeout_enabled;
    volatile Uart_queue_t tx_queue[TX_QUEUE_LENGTH];

    uint8_t rx_queue[RX_QUEUE_LENGTH];

    //a storage medium for a decoded message.
    mavlink_message_t rx_msg;

    uint16_t receive_chars_before_interrupt;
    UART_DATA_Type_t uart_data_type;
    UART_TRAMSMIT_TYPE_t uart_transmit_type;
    /*
     * Variables related to SBUS data
     */
    SBUS_STATUS_t sbus_status;
    uint8_t raw_sbus_frame[RAW_SBUS_DATA_LENGTH];
    uint8_t sbus_data_pointer;

    /**
    * Variable related to S-PORT data
    */
    SMART_PORT_STATUS_t smart_port_status;
    uint8_t smart_port_poll_frame[SMART_PORT_POLL_LENGTH];
    uint8_t smart_port_data_pointer;

    //status of the buffers.
    volatile uint16_t tx_queue_head;
    volatile uint16_t tx_queue_tail;
    volatile uint16_t rx_queue_tail;

    uint32_t * filter_list;
    uint8_t filter_list_size;
    uint32_t * whitelist_filter_list;
    uint8_t whitelist_filter_list_size;
    volatile bool uart_idle;
    uint8_t smart_port_poll_packet;
    //driver states start in the init state
    Driver_state prev_state = DRIVER_STATE_INIT;
    // Driver_state this_state = DRIVER_STATE_INIT;

    // Methods.
    /*
     *	Aborts the transmit and recieve.
     */
    inline void stop_tx_rx(void);
    /*
     *	Starts the transmit and reception
     */
    inline void start_tx_rx(void);
    inline void start_rx(void);

    inline void increment_tx_tail(void);
    inline void increment_tx_head(void);
    // void operator =(Uart_module const&);	// Poisoned.
};

bool check_SMART_PORT_Packet(const uint8_t * packet);  // from: https://github.com/opentx/opentx/blob/2.3/radio/src/telemetry/frsky_sport.cpp  checkSportPacket


// DECLARE PUBLIC GLOBAL VARIABLES.

// DEFINE PUBLIC STATIC FUNCTION PROTOTYPES.

// ALL DONE.

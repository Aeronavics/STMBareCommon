/********************************************************************************************************************************
*
*  FILE:      uart_mavlink.cpp
*
*  SUB-SYSTEM:		Avionics
*
*  COMPONENT:		Payload Controller.
*
*  TARGET:         AT90CAN128
*
*  PLATFORM:       BareMetal
*
*  AUTHOR:         Anton Slooten
*
*  DATE CREATED:   16-01-2017
*
*	Device driver which controls the a UART for communicating with the rest of the aircraft avionics via MAVlink.
*
*  Copyright 2016 Aeronavics Ltd
*
********************************************************************************************************************************/

// INCLUDE THE MATCHING HEADER FILE.

#include "uart_module.hpp"

// INCLUDE REQUIRED HEADER FILES FOR IMPLEMENTATION.

// DECLARE PRIVATE GLOBAL VARIABLES.

// IMPLEMENT PUBLIC STATIC FUNCTIONS.

bool Uart_module::is_uart(Uart_module_t uart_module)
{
        return uart_module == uart_type;
}

Uart_module::~Uart_module(void)
{
        // Nothing to do here.
        // All done.
        return;
}

// IMPLEMENT PUBLIC CLASS FUNCTIONS.

Uart_module::Uart_module()
{
        // Initialise the driver state machine.
        prev_state = DRIVER_STATE_UNKNOWN;
        this_state = DRIVER_STATE_UNKNOWN;

        set_receive_bytes(UART_MAVLINK_RX_CHARS_RECEIVED_LENGTH);
        set_uart_type(UART_DATA_TYPE_MAVLINK);

}

void Uart_module::set_uart_module(UART_HandleTypeDef * uart, Uart_module_t uart_module_type, bool idle_processing, mavlink_channel_t mav_channel)
{
        UART_MODULE = uart;
        uart_type = uart_module_type;
        millisecs_since_rx = 0;
        MAVLINK_CHANNEL = mav_channel;
        set_is_timeout_relevant(true);
        set_transmit_type(UART_TRAMSMIT_TYPE_INTERRUPT);
}

void Uart_module::set_transmit_type(UART_TRAMSMIT_TYPE_t transmit_type)
{
        uart_transmit_type = transmit_type;
}

void Uart_module::set_is_timeout_relevant(bool is_timeout_relevant)
{
        timeout_enabled = is_timeout_relevant;
}

void Uart_module::set_uart_type(UART_DATA_Type_t uart_type_to_set)
{
        uart_data_type = uart_type_to_set;
}

void Uart_module::set_whitelist_filter_list(uint32_t * filter, uint8_t size)
{
        whitelist_filter_list = filter;
        whitelist_filter_list_size = size;
}

/**
 * Sets the number of bytes to receive before interrupting
 * @param bytes_to_receive
 */
void Uart_module::set_receive_bytes(uint16_t bytes_to_receive)
{
        receive_chars_before_interrupt = bytes_to_receive;
}

void Uart_module::sync_update_unthrottled()
{
        // NOTE - The housekeeping state machine is called at 1Hz.

        // If we're not in STATE_NORMAL, then we don't do anything yet.
        if (this_state != DRIVER_STATE_NORMAL)
        {
                return;
        }
        /**
         * Check to see if we are ready to transmit, and we are currently not transmitting
         * 0x01 is the last bit of HAL_UART_STATE_BUSY_TX, which indicates tx is ongoing.
         */
        HAL_UART_StateTypeDef hal_state = HAL_UART_GetState(UART_MODULE);
        if (( hal_state & HAL_UART_STATE_READY) && (hal_state & 0x01) == 0x00 )
        {
                /**
                 * Check to see if we have anything to send.
                 */
                if (tx_queue_head != tx_queue_tail)
                {

                        //some bounds checking here. Apparently we might be able to get around it.
                        // memcpy((void *) &current_queue, (void *) &tx_queue[tx_queue_tail], sizeof (current_queue));
                        current_queue = const_cast<Uart_queue_t &>(tx_queue[tx_queue_tail]);
                        increment_tx_tail();
                        if(current_queue.length <= MAVLINK_MAX_PACKET_LEN)
                        {
                                //check to see what type of transmission we need to do.
                                if (uart_transmit_type == UART_TRAMSMIT_TYPE_DMA)
                                {
                                        HAL_UART_Transmit_DMA(UART_MODULE, (uint8_t *) current_queue.buffer, current_queue.length);
                                }
                                else 
                                {
                                        HAL_UART_Transmit_IT(UART_MODULE, (uint8_t *) current_queue.buffer, current_queue.length);
                                }
                        }
                }
        }


        uint8_t processing_counter = 0;
        //This will break in the init sequence, however this is not enabled during that sequence.
        while (is_rx_data_available() && processing_counter < PROCESSING_LIMIT)
        {
                processing_counter++;
                millisecs_since_rx = 0;
                uint8_t data = get_data();
                switch (uart_data_type)
                {
                        case UART_DATA_TYPE_MAVLINK:
                        {
                                mavlink_message_t msg;
                                mavlink_status_t * status = mavlink_get_channel_status(MAVLINK_CHANNEL);

                                if (mavlink_parse_char(MAVLINK_CHANNEL, data, &msg, status) == MAVLINK_FRAMING_OK)
                                {
                                        driverhost_broadcast_mavlink(const_cast<const mavlink_message_t&> (msg), this);
                                }
                                break;
                        }
                        case UART_DATA_TYPE_SBUS:
                        {
                                /**
                                 * Handle the sbus case data.
                                 */
                                if (uart_idle)
                                {
                                        sbus_status = SBUS_STATUS_UNSYNCED;
                                }
                                switch (sbus_status)
                                {
                                        case SBUS_STATUS_SYNCED:
                                        {
                                                sbus_data_pointer++;
                                                raw_sbus_frame[sbus_data_pointer] = data;
                                                if (sbus_data_pointer >= (RAW_SBUS_DATA_LENGTH - 1))
                                                {
                                                        /**
                                                         * We have got the total of data. Lets do something with that
                                                         */
                                                        mavlink_message_t msg;
                                                        mavlink_msg_anv_msg_sbus_data_pack(get_sID(), get_cID(), &msg, raw_sbus_frame);
                                                        driverhost_broadcast_mavlink(msg, this);
                                                        sbus_status = SBUS_STATUS_UNSYNCED;
                                                        for (uint8_t i = 0; i < RAW_SBUS_DATA_LENGTH; i++)
                                                        {
                                                                raw_sbus_frame[i] = 0;
                                                        }
                                                }
                                                break;
                                        }
                                        default:
                                        {
                                                //check if this is the byte we are looking for.
                                                if (data == SBUS_MAGIC_START_BYTE && uart_idle)
                                                {
                                                        sbus_status = SBUS_STATUS_SYNCED;
                                                        sbus_data_pointer = 0;
                                                        raw_sbus_frame[sbus_data_pointer] = data;
                                                        /**
                                                         * Start the idle line detection so we can monitor our frame progress
                                                         */
                                                        uart_idle = false;
                                                }
                                                break;
                                        }
                                }
                                break;
                        }
                        case UART_DATA_TYPE_SMART_PORT:
                        {
                                /**
                                * Handle the SMART PORT case data
                                * This is an STM generated interrupt that detects when a uart line goes idle. Given that we are talking in predefined block messages,
                                * it is expected that between messages there is some idle time. If we are idle, then we basically assume we have not started receiving anything yet.
                                * */
                                switch (smart_port_status)
                                {
                                        case SMART_PORT_STATUS_SYNCED:
                                        {
                                                /**
                                                 * 
                                                 * Adding a check to see if this is the packet we care about.
                                                 * This simply reduces the loading on function calls, as this will be called very regularly.
                                                 * */
                                                if(data == smart_port_poll_packet)
                                                {
                                                        //smart_port_data_pointer++;
                                                        smart_port_poll_frame[1] = data;
                                                        mavlink_message_t msg;
                                                        mavlink_msg_anv_msg_smart_port_poll_packet_pack(get_sID(), get_cID(), &msg, smart_port_poll_frame);
                                                        driverhost_broadcast_mavlink(msg, this);
                                                        //rather than creating a for loop, just manually reset the data.
                                                        smart_port_poll_frame[0] = 0;
                                                        smart_port_poll_frame[1] = 0;
                                                        //disable this uart:
                                                        HAL_UART_AbortReceive(UART_MODULE);

                                                }
                                                //go back to waiting for the start bit
                                                smart_port_status = SMART_PORT_STATUS_UNSYNCED;
                                                break;
                                        }
                                        default:
                                        {
                                                if (data == FSSP_START_STOP && uart_idle)
                                                {
                                                        smart_port_status = SMART_PORT_STATUS_SYNCED;
                                                        //smart_port_data_pointer = 0;
                                                        smart_port_poll_frame[0] = data;
                                                        /**
                                                        * Start the idle line detection so we can monitor our frame progress
                                                        */
                                                        uart_idle = false;
                                                }
                                        break;
                                        }
                                }
                                break;
                        }
                        case UART_DATA_TYPE_DATA:
                        {
                                //we are currently receiving raw data. Let us
                                driverhost_broadcast_rx_raw(data, uart_type);
                                break;
                        }
                        default:
                        {
                                break;
                        }
                }
        }
        // All done.
        return;
}

void Uart_module::sync_update_100Hz()
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

void Uart_module::sync_update_10Hz()
{
        // NOTE - The housekeeping state machine is called at 1Hz.

        // If we're not in STATE_NORMAL, then we don't do anything yet.
        if (this_state != DRIVER_STATE_NORMAL)
        {
                return;
        }

        // Increment the number of seconds since we received a message, and if it gets too high, assume something has gone wrong.
        millisecs_since_rx += 100;
        if (millisecs_since_rx > MAX_MILLISECS_SINCE_RX && timeout_enabled)
        {
                millisecs_since_rx = 0;
                this_state = DRIVER_STATE_ERROR;
        }

        return;
}

void Uart_module::sync_update_1Hz()
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
                /**
                 * Start the UART
                 */
                init_uart();
                if (uart_data_type == UART_DATA_TYPE_SBUS || uart_data_type == UART_DATA_TYPE_SMART_PORT)
                {
                        enable_idle_callback();
                } 
                
                if(uart_data_type == UART_DATA_TYPE_SMART_PORT)
                {
                        //disable the tx line which happens here automatically
                        callback_tx_complete();
                }
       
                /**
                 * Clear the timer since last rx message
                 */
                millisecs_since_rx = 0;
                /**
                 * Progress to the next state
                 */
                /*
                * Safety check to see if the DMA has actually been enabled - i.e. we have properly provided a receive peripheral, and a transmit peripheral
                TODO: Is this actually what we want to do? Shouldn't we make compilation fail here?
                */
                if(uart_transmit_type == UART_TRAMSMIT_TYPE_DMA && (UART_MODULE->hdmarx == nullptr || UART_MODULE->hdmatx == nullptr))
                {
                        set_transmit_type(UART_TRAMSMIT_TYPE_INTERRUPT);
                }

                next_state = DRIVER_STATE_NORMAL;
                break;
        }
        case DRIVER_STATE_NORMAL:
        {
                break;
        }
        case DRIVER_STATE_ERROR:
        {
                // Report a problem.
                driverhost_report_event(DRIVER_EVENT_ALARM, 2);
                //restart the receive process. Also abort the transmit process
                reset_uart();
                next_state = DRIVER_STATE_NORMAL;
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

void Uart_module::callback_rx_complete(void)
{
        start_rx();
}

void Uart_module::start_tx_rx(void)
{
        /**
         * Reset the data pointers. This might break a currently being received message,
         * however we don't really care, as this is called in an error.
         */
        rx_queue_tail = RX_QUEUE_LENGTH;
        start_rx();

}

void Uart_module::stop_tx_rx(void)
{
        //aborts reception
        HAL_UART_AbortReceive(UART_MODULE);
        //aborts the transmit - this gets automatically started again on next transmit
        HAL_UART_AbortTransmit(UART_MODULE);
}

void Uart_module::reset_uart(void)
{
        /**
         * Clear the error.
         * apparently this is done by reading the status register and the data register.
         * */
        uint32_t clear_status = READ_REG(UART_MODULE->Instance->SR);
        uint32_t clear_data = READ_REG(UART_MODULE->Instance->DR);
        if(clear_status == 0 && clear_data == 0)
        {
                //something here to stop the compiler optimising this out. We actually do not care if this is reached at all, it is simply so the SR and DR can be read.
                asm("nop");
        }

        stop_tx_rx();
        start_tx_rx();
}

void Uart_module::init_uart(void)
{
        //reset the requesite parts
        //rx_queue_head = 0;
        rx_queue_tail = 0;
        tx_queue_head = 0;
        tx_queue_tail = 0;
        millisecs_since_rx = 0;
        uart_idle = true;
        //disable the uart - this is useful if we have a malfunctioning uart and, well, need to reset it. Guarantees a known state.
        HAL_UART_MspDeInit(UART_MODULE);

        
        switch (uart_type)
        {
    #undef UART_MODULE_FOUND
        case UART_MODULE_UART_1:
        {
            #ifdef UART1_ENABLED
            #define UART_MODULE_FOUND
                MX_USART1_UART_Init();
                UART_MODULE = &huart1;
                break;
            #endif
        }
        case UART_MODULE_UART_2:
        {
            #ifdef UART2_ENABLED
            #define UART_MODULE_FOUND
                MX_USART2_UART_Init();
                UART_MODULE = &huart2;
                break;
            #endif
        }
        case UART_MODULE_UART_3:
        {
            #ifdef UART3_ENABLED
            #define UART_MODULE_FOUND
                MX_USART3_UART_Init();
                UART_MODULE = &huart3;
                break;
            #endif
        }
        case UART_MODULE_UART_4:
        {
            #ifdef UART4_ENABLED
            #define UART_MODULE_FOUND
                MX_UART4_Init();
                UART_MODULE = &huart4;
                break;
            #endif
        }
        case UART_MODULE_UART_5:
        {
            #ifdef UART5_ENABLED
            #define UART_MODULE_FOUND
                MX_UART5_Init();
                UART_MODULE = &huart5;
                break;
            #endif
        }
        default: break;
        }
    #ifndef UART_MODULE_FOUND
    #error "Please enable UART<N>_ENABLE in your config file's CPP section"
    #endif

        start_tx_rx();
}

void Uart_module::enable_idle_callback(void)
{
        __HAL_UART_ENABLE_IT(UART_MODULE, UART_IT_IDLE);
}

void Uart_module::disable_idle_callback(void)
{
        __HAL_UART_DISABLE_IT(UART_MODULE, UART_IT_IDLE);
}

void Uart_module::set_filter_list(uint32_t * filter, uint8_t size)
{
        //set the filter list
        filter_list = filter;
        filter_list_size = size;
}

void Uart_module::callback_rx_idle(bool restart_interrupt)
{
        /**
         * Set the IDLE flag - if we are currently processing things and the line is set to idle,
         * this might mean different things (i.e. the MAVLINK message has been split up between packets,
         * OR in the case of SBUS, we are not aligned correctly.
         */
        uart_idle = true;
}

void Uart_module::start_rx(void)
{
        /**
         * Start the reception of data.
         * We simply provide a massive buffer to handle sporadic data.
         * This basically becomes our ring buffer, RX_QUEUE_LENGTH essentially resets the head pointer.
         * We do not care about the tail pointer, as that is potentially still processing data.
        */
        if (uart_transmit_type == UART_TRAMSMIT_TYPE_DMA)
        {
                HAL_UART_Receive_DMA(UART_MODULE, (uint8_t *) rx_queue, RX_QUEUE_LENGTH);
        }
        else
        {
                HAL_UART_Receive_IT(UART_MODULE, (uint8_t *) rx_queue, RX_QUEUE_LENGTH);
        }
}

/**
 * Add data to the transmit queue. This operates through the same mechanism as the MAVLINK tx queue
 */
void Uart_module::queue_tx_raw(const uint8_t * message, uint16_t size)
{
        /**
         * Only send this if the message is smaller than our buffer size.
         * Possibly an opportunity to break the message into buffer sized chunks and
         * transmit all of it?
         */
        if (size <= MAVLINK_MAX_PACKET_LEN)
        {
                /**
                 * If we have queued so many messages that we are at the tail again,
                 * lets push the tail around, so we do not overrun our buffer
                 */

                //memcpy((void *) tx_queue[tx_queue_head].buffer, (void *) message, size);
                *tx_queue[tx_queue_head].buffer = *message;
                tx_queue[tx_queue_head].length = size;
                increment_tx_head();
        }
}

void Uart_module::handle_rx_mavlink(const mavlink_message_t & message)
{
        // In the case of the UART MAVlink module, this function just SENDS the received message out the UART.

        // Unless the driver is actually working properly, don't try to send anything, because the UART might not be running, in which case we can get stuck.
        if (this_state != DRIVER_STATE_NORMAL) return;

        //Add message to the end of the buffer

        //only add the message if it is not in the filter list
        for (uint8_t i = 0; i < filter_list_size; i++)
        {
                if (filter_list[i] == message.msgid)
                {
                        return;
                }
        }
        /**
         * Only add message if it exists in the white-list (if the white-list exists)
         */
        if (whitelist_filter_list_size > 0)
        {
                bool in_whitelist = false;
                for (uint8_t i = 0; i < whitelist_filter_list_size; i++)
                {
                        if (whitelist_filter_list[i] == message.msgid)
                        {
                                in_whitelist = true;
                                break;
                        }
                }
                if (!in_whitelist)
                {
                        return;
                }
        }
        switch (uart_data_type)
        {
                case UART_DATA_TYPE_MAVLINK:
                {
                        tx_queue[tx_queue_head].length = static_cast<size_t> (mavlink_msg_to_send_buffer((uint8_t *) tx_queue[tx_queue_head].buffer, &message));
                        increment_tx_head();
                        break;
                }
                default:
                {
                        /**
                         * unless we know how to send what we want, lets not send it.
                         * This method is typically called when we broadcast mavlink messages. Unless we are talking mavlink, lets not do anything
                         */
                        return;
                }
        }

        return;
}

uint8_t Uart_module::get_data(void)
{
        uint8_t data = rx_queue[RX_QUEUE_LENGTH - rx_queue_tail];
        rx_queue_tail--;
        return data;
}
bool Uart_module::is_rx_data_available()
{
        uint16_t current_rx_counter = 0;
        // If there is a received MAVlink message waiting to be handled, we'd better do that.
        if (uart_transmit_type == UART_TRAMSMIT_TYPE_DMA)
        {
                current_rx_counter = __HAL_DMA_GET_COUNTER(UART_MODULE->hdmarx);
        }
        else 
        {
                current_rx_counter = UART_MODULE->RxXferCount;
        }
        //perform housekeeping in the event we have restarted the reception:
        if(rx_queue_tail == 0 && current_rx_counter > 0)
        {
                //There is nothing here for us. All that has happened is that we have restarted the transmission.
                //We may have already received data, but we do not know how much, hence we only care that it is larger than the tail (zero)
                rx_queue_tail = RX_QUEUE_LENGTH;
        }

        //check to see if there is any data.
        if(current_rx_counter == rx_queue_tail)
        {
                return false;
        }
        //There is? Go forth an process.
        return true;
}

void Uart_module::increment_tx_tail(void)
{
        tx_queue_tail++;
        tx_queue_tail %= TX_QUEUE_LENGTH;
}

void Uart_module::increment_tx_head(void)
{
        tx_queue_head++;
        tx_queue_head %= TX_QUEUE_LENGTH;
        //check to see we have not caught up to the tail. If we have, just increment it and get rid of the oldest data.
        if(tx_queue_head == tx_queue_tail)
        {
                increment_tx_tail();
        }
}

void callback_tx_half_complete(void)
{
        /**
         * Unused. This might be useful with DMA transfers.
         */
}

void Uart_module::callback_tx_complete(void)
{
        //Our transmission is complete, let us begin receiving again.
        if(uart_data_type == UART_DATA_TYPE_SMART_PORT)
        {
                //reset the rx_tail header.
                rx_queue_tail = RX_QUEUE_LENGTH;
                start_rx();
        }
        
}

bool Uart_module::ready_to_send() const
{
        return ((this_state == DRIVER_STATE_NORMAL));
}

void Uart_module::register_smart_port_poll_packet(uint8_t packet_id)
{
        smart_port_poll_packet = packet_id;
        return;
}


// This method may well yet not be the correct way to validate the poll packet's checksum...
bool check_SMART_PORT_Packet(const uint8_t * packet) // from: https://github.com/opentx/opentx/blob/2.3/radio/src/telemetry/frsky_sport.cpp  checkSportPacket
{
  short crc = 0;
  for (int i=1; i<SMART_PORT_POLL_LENGTH; ++i) {
    crc += packet[i]; // 0-1FE
    crc += crc >> 8;  // 0-1FF
    crc &= 0x00ff;    // 0-FF
  }
  
  return (crc == 0x00ff);
}

// IMPLEMENT PRIVATE STATIC FUNCTIONS.

// IMPLEMENT PRIVATE CLASS FUNCTIONS.

// IMPLEMENT INTERRUPT SERVICE ROUTINES.

// ALL DONE.

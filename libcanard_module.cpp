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

#include "libcanard_module.hpp"
#ifdef LIBCANARD_MESSAGE_PARAMETERS
  #define CAN_EXTERN // Fairly common C/CPP magic to allow global defintions in header files.
    #include "can_params.hpp"
  #undef CAN_EXTERN
#endif

/**
 A pile of global functions for creating new parameters. This is mostly used in the can_params.hpp
*/
#ifdef LIBCANARD_MESSAGE_PARAMETERS
uavcan_protocol_param_GetSetResponse new_boolean_can_param(const char * name, bool value, bool default_value)
{
  return new_can_param(name, (bool) value, (bool) default_value);
}
uavcan_protocol_param_GetSetResponse new_can_param(const char * name, bool value, bool default_value)
{
  uavcan_protocol_param_GetSetResponse return_val;
  return_val.value.boolean_value = (uint8_t) value;
  return_val.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
  return_val.default_value.boolean_value = (uint8_t) default_value;
  return_val.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
  return_val.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
  return_val.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
  std::string str_name(name);
  std::copy(str_name.begin(), str_name.end(), std::begin(return_val.name.data));
  return_val.name.len  = str_name.length();
  return return_val;
}
uavcan_protocol_param_GetSetResponse new_integer_can_param(const char * name, int64_t value, int64_t default_value, int64_t min_value, int64_t max_value)
{
  return new_can_param(name, (int64_t) value, (int64_t) default_value, (int64_t) min_value, (int64_t) max_value);
}
uavcan_protocol_param_GetSetResponse new_can_param(const char * name, int64_t value, int64_t default_value, int64_t min_value, int64_t max_value)
{
  uavcan_protocol_param_GetSetResponse return_val;
  return_val.value.integer_value = value;
  return_val.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
  return_val.default_value.integer_value = default_value;
  return_val.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
  if(min_value == 0 && max_value == 0)
  {
    return_val.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
    return_val.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
    
  }
  else
  {
    return_val.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
    return_val.min_value.integer_value = min_value;
    return_val.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
    return_val.max_value.integer_value = max_value;
  }
  std::string str_name(name);
  std::copy(str_name.begin(), str_name.end(), std::begin(return_val.name.data));
  return_val.name.len  = str_name.length();
  return return_val;
}
uavcan_protocol_param_GetSetResponse new_real_can_param(const char * name, float value, float default_value, float min_value, float max_value)
{
  return new_can_param(name, (float) value, (float) default_value, (float) min_value, (float) max_value);
}
uavcan_protocol_param_GetSetResponse new_can_param(const char * name, float value, float default_value, float min_value, float max_value)
{
  uavcan_protocol_param_GetSetResponse return_val;
  return_val.value.real_value = value;
  return_val.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
  return_val.default_value.real_value = default_value;
  return_val.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
  if(min_value == 0 && max_value == 0)
  {
    return_val.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
    return_val.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
  }
  else
  {
    return_val.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
    return_val.min_value.real_value = min_value;
    return_val.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
    return_val.max_value.real_value = max_value;
  }

  std::string str_name(name);
  std::copy(str_name.begin(), str_name.end(), std::begin(return_val.name.data));
  return_val.name.len  = str_name.length();
  return return_val;
}
#endif

void Libcanard_module::sync_update_unthrottled()
{
    if (this_state != DRIVER_STATE_NORMAL)
        return;
    poll();
    return;
}

void Libcanard_module::sync_update_100Hz()
{
    if (this_state != DRIVER_STATE_NORMAL)
        return;
    using namespace impl_;

    #ifdef LIBCANARD_MESSAGE_FIRMWARE_UPGRADE
      /*
       * Waiting for the firmware update request
       */
      int result = 1;
      switch (download_state)
      {
          case DownloadState::DownloadStateIdle:
          {
              //This state is the default state and does nothing of importance
              break;
          }
          case DownloadState::DownloadStateDownloading:
          {
              //house keeping and a check to see if we need to request a new chunk

              if (transfer_started)
              {
                  /**
                   * There is a live transfer being asked. Let us check to see if we need a chunk requested
                   */
                  uint64_t timestamp = getMonotonicTimestampUSec();

                  if (timestamp > response_deadline)
                  {
                      char bytes_downloaded[20];
                      snprintf(bytes_downloaded, 20, "%u", (uint32_t) offset);
                      std::string string_bytes_downloaded(bytes_downloaded);
                      sendLog(LogLevel::Info, string_bytes_downloaded + "B down...");
                      response_deadline = timestamp + ServiceRequestTimeoutMillisecond * 1000;
                  }
                  if (timestamp > wait_deadline & !response_recieved)
                  {
                      sendLog(LogLevel::Warning, "Retrying Chunk");
                      retry_count++;
                      if (retry_count > 10)
                      {
                          //if we have timed out - let the user know
                          sendLog(impl_::LogLevel::Error, "Downloaded timed out");
                          //clean up and make sure we are in a state that will not incorrectly boot
                          download_state = DownloadState::DownloadStateEnd;
                          can_module_state = State::NoAppToBoot;
                          result = 1;
                          break;
                      }
                      request_next_chunk = true;
                  }
              }

              if (request_next_chunk)
              {
                  /**
                   * Once we have a firmware update request, then we can begin asking for file chunks.
                   */
                  //we only do this if we are not retrying chunks. Keeps some sanity
                  if (response_recieved)
                  {
                      if (read_result_ > 0 && read_result_ != std::numeric_limits<int>::max())
                      {
                          //Offset is current sitting at our last index
                          for (uint16_t i = 0; i <= read_result_ - 1; i++)
                          {
                              tmp_array[i] = read_buffer_[i];
                          }
                          /**
                           * At every new page let us clean and start again. Careful! This only works if we start flashing from a multiple of 2048
                           *
                           */
                          if (offset % 2048 == 0)
                          {
                              FLASH_If_Erase_Page(FLASH_START_ADDRESS + (offset));
                          }
                          FLASH_If_Write(FLASH_START_ADDRESS + offset, reinterpret_cast<uint32_t*> (tmp_array), read_result_ / 4);
                          offset += read_result_;
                      }
                      else if (read_result_ == 0)
                      {
                          //done downloading
                          sendLog(impl_::LogLevel::Info, "Finished Transfer");
                          download_state = DownloadState::DownloadStateEnd;
                          //Check to see if we have some code to boot, if not we will send the user an error
                          if (is_boot_image_available())
                          {
                              can_module_state = State::BootDelay;
                          }
                          break;
                      }
                  }

                  std::uint8_t buffer[dsdl::FileRead::MaxSizeBytesRequest]{};
                  canardEncodeScalar(buffer, 0, 40, &offset);
                  std::copy(firmware_file_path_.begin(), firmware_file_path_.end(), &buffer[5]);

                  const int res = canardRequestOrRespond(&canard_,
                          remote_server_node_id_,
                          dsdl::FileRead::DataTypeSignature,
                          dsdl::FileRead::DataTypeID,
                          &file_read_transfer_id_,
                          CANARD_TRANSFER_PRIORITY_HIGH,
                          CanardRequest,
                          buffer,
                          firmware_file_path_.size() + 5);
                  wait_deadline = getMonotonicTimestampUSec() + ServiceRequestTimeoutMillisecond * 800;
                  request_next_chunk = false;
                  response_recieved = false;
              }
              break;
          }
          case DownloadState::DownloadStateEnd:
          {
              //when we have finished downloading we handle the finished product here

              if (result >= 0)
              {
                  vendor_specific_status_ = 0;
                  if (can_module_state == State::NoAppToBoot)
                  {
                      sendLog(impl_::LogLevel::Error, "Downloaded image is invalid");
                  }
                  else
                  {
                      sendLog(impl_::LogLevel::Info, "OK");
                  }
              }
              else
              {
                  vendor_specific_status_ = abs(result);
              }

              /*
               * Reset everything to zero and loop again, because there's nothing else to do.
               * The outer logic will request reboot if necessary.
               */
              remote_server_node_id_ = 0;
              firmware_file_path_.clear();
              offset = 0;
              //move to an idle state
              download_state = DownloadState::DownloadStateIdle;
              break;
          }
          default:
          {
              download_state = DownloadState::DownloadStateIdle;
              break;
          }
      }
    #endif

    return;
}

void Libcanard_module::sync_update_10Hz()
{
    return;
}

void Libcanard_module::sync_update_1Hz()
{
    Driver_state next_state = this_state;

    switch (this_state)
    {
        case DRIVER_STATE_UNKNOWN:
        {
            next_state = DRIVER_STATE_INIT;
            break;
        }
        case DRIVER_STATE_INIT:
        {
            //check to see if we have enabled parameters
            if(!can_enabled)
            {
                return;
            }
            #if defined(STM32F1XX) || defined(STM32L4XX)
            if (init_state == INIT_STATE_UNINITIALIZED)
            {
                    can_tx_count = 0;
                    //can_rx_count = 0;
                    //load parameters if we want them
                    #ifdef LIBCANARD_MESSAGE_PARAMETERS
                    //if we have failed to load the parameters, let us store the default ones.
                    if(!load_can_parameters())
                    {
                        //This will store the relavent flags so we load correctly the next time.
                        save_can_parameters(false);
                    }
                    #endif
                    //start the STM32 CAN drivers
#ifdef STM32F1XX
#define STM_CAN_DEFINED
                    MX_CAN_Init();
#endif
#ifdef STM32G4XX
#define STM_CAN_DEFINED
                    MX_FDCAN1_Init();
#endif
#ifdef STM32L4XX
#define STM_CAN_DEFINED
                    MX_CAN1_Init();
#endif
#ifndef STM_CAN_DEFINED
#error No Can Initialisation function defined
#endif

                    //todo:fix bit rate allocation
                    this->can_bus_bit_rate_ = 1000000; //can_bus_bit_rate;

                    //Initiate the canard driver
                    canardInit(&canard_,
                            memory_pool_.data(),
                            memory_pool_.size(),
                            &Libcanard_module::onTransferReceptionTrampoline,
                            &Libcanard_module::shouldAcceptTransferTrampoline,
                            (void *) this);


                    //begin bit rate autodetection
                    init_state = INIT_STATE_BITRATE_AUTODETECT;
                    //This break is deliberately commented out. We always want the rest to run. On potential subsequent runthroughs, this will be skipped
                    //break;
            }

            if(init_state == INIT_STATE_BITRATE_AUTODETECT)
            {
                    //we can keep coming back to here if there is no CAN messages coming through.
                    //check if we succeed in getting a CAN baud, if so, continue the boot
                    if(init_can_device())
                    {
                        init_state = INIT_STATE_POST_SETUP;
                    }
             }
             if (init_state == INIT_STATE_POST_SETUP)
             {
                    //we have a baud rate set - let us finish setting everything up.
                    //let us no longer stay in the init phase
                    next_state = DRIVER_STATE_NORMAL;

                    #ifdef LIBCANARD_MESSAGE_FIRMWARE_UPGRADE
                        if (is_boot_image_available())
                        {
                            //check if we have a boot image for our starting state.
                            can_module_state = State::BootDelay;
                        }
                    #else
                    can_module_state = State::Initialization;
                    #endif
            }
            #elif defined(STM32G4XX)
                can_tx_count = 0;
                //can_rx_count = 0;
                //load parameters if we want them
                #ifdef LIBCANARD_MESSAGE_PARAMETERS
                    //if we have failed to load the parameters, let us store the default ones.
                    if(!load_can_parameters())
                    {
                        //This will store the relavent flags so we load correctly the next time.
                        save_can_parameters(false);
                    }
                #endif
                //start the STM32 CAN drivers

                MX_FDCAN1_Init();
#ifdef CANARD_MULTI_IFACE
                MX_FDCAN2_Init();
#endif

                /* Configure Rx filter */
                sFilterConfig.IdType = FDCAN_EXTENDED_ID;
                sFilterConfig.FilterIndex = 0;
                sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
                sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
                sFilterConfig.FilterID1 = 0x0;
                sFilterConfig.FilterID2 = 0x1FFFFFFF;

                this->can_bus_bit_rate_ = 1000000; //can_bus_bit_rate;

				//Initiate the canard driver
				canardInit(&canard_,
						memory_pool_.data(),
						memory_pool_.size(),
						&Libcanard_module::onTransferReceptionTrampoline,
						&Libcanard_module::shouldAcceptTransferTrampoline,
						(void *) this);

                init_can_device();

                if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
                {
                    Error_Handler();
                }

                /* Start the FDCAN module */
                if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
                {
                    Error_Handler();
                }

                if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
                {
                    Error_Handler();
                }

#ifdef CANARD_MULTI_IFACE
                if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
								{
										Error_Handler();
								}

                /* Start the FDCAN module */
								if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
								{
									Error_Handler();
								}

                if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
								{
									Error_Handler();
								}
#endif


                // let us finish setting everything up.
                // let us no longer stay in the init phase
                next_state = DRIVER_STATE_NORMAL;

                #ifdef LIBCANARD_MESSAGE_FIRMWARE_UPGRADE
                    if (is_boot_image_available())
                    {
                        //check if we have a boot image for our starting state.
                        can_module_state = State::BootDelay;
                    }
                #else
                    can_module_state = State::Initialization;
                #endif

            #endif

        }
        case DRIVER_STATE_NORMAL:
        {
          /*
          * If we have an app to boot - let us wait for a timeout before booting
          */
          #ifdef LIBCANARD_MESSAGE_FIRMWARE_UPGRADE
            if (can_module_state == State::BootDelay)
            {
              if (boot_delay_count >= BOOT_DELAY)
              {
                boot();
                //we should never get here unless there is actually no app to boot and the starts align and we happen to not stuff up anything
                can_module_state = State::NoAppToBoot;
                sendLog(impl_::LogLevel::Error, "Failed to boot!");
              }
              boot_delay_count++;
            }
            #else


            can_module_state = State::NormalOperation;

            /*
              This is not a bootloader, and thus we do not actually care about booting, we will continue as per normal.
            */
          #endif

          if(force_reboot)
          {
              NVIC_SystemReset();
          }
          if(signal_reboot)
          {
            //we have been signaled to perform a reboot.
            //let us send a message that we are in the proccess of rebooting.
            sendLog(impl_::LogLevel::Info, "Node Rebooting!");
            force_reboot = true;
          }
          handle1HzTasks();
          //check
            break;
        }
        case DRIVER_STATE_ERROR:
        {
            next_state = DRIVER_STATE_INIT;
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
    return;
}

/**
 * sets the name of the UAVCAN device.
 *  */
void Libcanard_module::set_name(const char * name)
{
    node_name_ = name;
    return;
}

void Libcanard_module::performCANBitRateDetection()
{
    /// These are defined by the specification; 100 Kbps is added due to its popularity.
    static constexpr std::array<std::uint32_t, 5> StandardBitRates
            {
        1000000, ///< Recommended by UAVCAN
        500000, ///< Standard
        250000, ///< Standard
        125000, ///< Standard
        100000 ///< Popular bit rate that is not defined by the specification
    };

    // Loop forever until the bit rate is detected
    if (can_bus_bit_rate_ == 0)
    {
        //TODO:fwd
        //        watchdog_.reset();
        if (kill_loading)
        {
            //The user has asked for ymodem as the protocol to upload code. Therefore we should not touch libcanard
            //enter error state. This is locking
            this_state = DRIVER_STATE_ERROR;
            return;
        }
        const std::uint32_t br = StandardBitRates[current_bit_rate_index];
        current_bit_rate_index = (current_bit_rate_index + 1) % StandardBitRates.size();
#if defined(STM32F1XX) || defined(STM32L4XX)
        int init = initCAN(br, Mode::Silent);
        if (init >= 0)
        {
            const int res = receive(500).first;

            if (res > 0)
            {
                can_bus_bit_rate_ = br;
            }
        }
#endif
    }
}

bool Libcanard_module::init_can_device(void)
{
  if (node_name_ == "")
  {
    this->set_name("unconfigured.aeronavics.com");
  }

    /*
     * CAN bit rate
     */
    if (can_bus_bit_rate_ == 0)
    {
        performCANBitRateDetection();
    }
    //check we still have not got a valid baud 
    if(can_bus_bit_rate_ == 0)
    {
        return false;
    }

    /*
     * Node ID
     */
    if (canardGetLocalNodeID(&canard_) == 0)
    {
        //we actually do not want to do this by default.
        //If there is a node ID stored in the application code, we will use that
        //we will not use the CAN id - as we might not always have a CAN ID
        const volatile uint8_t *param_values = (const volatile uint8_t *)(CAN_EEPROM_LOCATION + CAN_NODE_LOCATION_OFFSET);
        //we will cast this to uint8_t and do some safety checks.
        uint8_t node_id = *param_values;
        if (node_id > 0 && node_id < 128)
        {
            //use the parameter found
            confirmed_local_node_id_ = node_id;
            canardSetLocalNodeID(&canard_, node_id);
        }
        else
        {
            //we will get here if we have not flashed the board before,
            //or something terrible has happened to the EEPROM
            performDynamicNodeIDAllocation();
            confirmed_local_node_id_ = canardGetLocalNodeID(&canard_);
        }
    }


    // This is the only info message we output during initialization.
    // Fewer messages reduce the chances of breaking UART CLI data flow.

    using namespace impl_;

    /*
     * Init CAN in proper mode now
     */
    //    watchdog_.reset();
#if defined(STM32F1XX) || defined(STM32L4XX)
    while (true)
    {
        if (kill_loading)
            return false;
        // Accept only correctly addressed service requests and responses
        // We don't need message transfers anymore
        AcceptanceFilterConfig filt;
        filt.id =   0b00000000000000000000010000000 | (confirmed_local_node_id_ << 8) | CANARD_CAN_FRAME_EFF;
        filt.mask = 0b00000000000000111111110000000 |
                CANARD_CAN_FRAME_EFF | CANARD_CAN_FRAME_RTR | CANARD_CAN_FRAME_ERR;


        if (initCAN(can_bus_bit_rate_, Mode::Normal, filt) >= 0)
        {
            break;
        }
    }
#endif

    init_done_ = true;
    return true;
}

Libcanard_module& Libcanard_module::get_driver(void)
{
    static Libcanard_module singleton;
    return singleton;
}

Libcanard_module::~Libcanard_module(void)
{
    return;
}

Libcanard_module::Libcanard_module(void)
{
    // Initialise the driver state machine.
    prev_state = DRIVER_STATE_UNKNOWN;
    this_state = DRIVER_STATE_UNKNOWN;
    hw_info_.unique_id = readUniqueID();
    hw_info_.major = 1;
    hw_info_.minor = 1;

    // All done.
    return;
}

float Libcanard_module::getRandomFloat(void)
{
    static bool initialized = false;
    if (!initialized)
    {
        initialized = true;
        srand((uint32_t) time(NULL));
    }
    return (float) rand() / (float) RAND_MAX;
}

uint64_t Libcanard_module::getMonotonicTimestampUSec(void) const
{
    //well, this just returns the number of usecs since boot
    return driverhost_get_monotonic_time_us();
}

uint64_t Libcanard_module::get_system_time_ms(void)
{
    return driverhost_get_monotonic_time_us() / 1000;
}

#if defined(STM32F1XX) || defined(STM32L4XX)

int Libcanard_module::init(const std::uint32_t bitrate, const Mode mode, const AcceptanceFilterConfig& acceptance_filter)
{
    //TODO:Fix these comments
    had_activity_ = false;
    last_led_update_timestamp_st_ = get_system_time_ms();
    // Computing CAN timings
    CanardSTM32CANTimings timings{};
    int res = canardSTM32ComputeCANTimings(STM32_PCLK1, bitrate, &timings);
    if (res < 0)
    {
        return res;
    }

    // Initializing the interface
    CanardSTM32IfaceMode mode2{};
    switch (mode)
    {
        case Mode::Normal:
        {
            mode2 = CanardSTM32IfaceModeNormal;
            break;
        }
        case Mode::Silent:
        {
            mode2 = CanardSTM32IfaceModeSilent;
            break;
        }
        case Mode::AutomaticTxAbortOnError:
        {
            mode2 = CanardSTM32IfaceModeAutomaticTxAbortOnError;
            break;
        }
    }
    res = canardSTM32Init(&timings, mode2);
    if (res < 0)
    {
        return res;
    }

    // Configuring acceptance filters
    CanardSTM32AcceptanceFilterConfiguration acceptance_filter_config_array[5];
    acceptance_filter_config_array[0].id = acceptance_filter.id;
    acceptance_filter_config_array[0].mask = acceptance_filter.mask;

#ifdef LIBCANARD_MESSAGE_ARRAYCOMMAND
    //let us accept a broadcast message, specifically, 1010 (array command),
    //as this is output by the flight controller
    acceptance_filter_config_array[1].id =   0b00000000000000000000000000000 | UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID << 8;
    acceptance_filter_config_array[1].mask = 0b00000111111111111111110000000;
#endif

#ifdef LIBCANARD_MESSAGE_LIGHTSCOMMAND
    //add lights command
    acceptance_filter_config_array[2].id =   0b00000000000000000000000000000 | UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID << 8;
    acceptance_filter_config_array[2].mask = 0b00000111111111111111110000000;
#endif

#ifdef LIBCANARD_MESSAGE_ARMINGSTATUS
    //add safety status command
    acceptance_filter_config_array[3].id =   0b00000000000000000000000000000 | UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID << 8;
    acceptance_filter_config_array[3].mask = 0b00000111111111111111110000000;
#endif

#ifdef LIBCANARD_MESSAGE_NOTIFYSTATE
    //add notify state command
    acceptance_filter_config_array[3].id =   0b00000000000000000000000000000 | ARDUPILOT_INDICATION_NOTIFYSTATE_ID << 8;
    acceptance_filter_config_array[3].mask = 0b00000111111111111111110000000;
#endif


    res = canardSTM32ConfigureAcceptanceFilters(&acceptance_filter_config_array[0], 5);

    if (res < 0)
    {
        return res;
    }

    return 0;
}

int Libcanard_module::send(const CanardCANFrame& frame, const int timeout_millisec)
{
    const auto started_at = get_system_time_ms();
    do
    {
        int res = canardSTM32Transmit(&frame); // Try to transmit
        if (res != 0)
        {
            had_activity_ |= res > 0;
            return res; // Either success or error, return
        }
        //        HAL_Delay(1);
        //wait(); // No space in the buffer, skip the time quantum and try again
    } while (get_system_time_ms() < (timeout_millisec + started_at));

    return 0; // Timed out
}

std::pair<int, CanardCANFrame> Libcanard_module::receive(const int timeout_millisec)
{
    const auto started_at = get_system_time_ms();
    CanardCANFrame f{};
    do
    {
        int res = canardSTM32Receive(&f);
        if (res != 0)
        {
            had_activity_ |= res > 0;
            return {res, f}; // Either success or error, return
        }
        //HAL_Delay(1); // Buffer is empty, skip the time quantum and try again
    } while (get_system_time_ms() < (timeout_millisec + started_at));

    return {0, f}; // Timed out
}

void Libcanard_module::onTransferReception(CanardRxTransfer * const transfer)
{
    using namespace impl_;


    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     * This piece of dark magic has been carefully transplanted from the libcanard demo application.
     */
    if ((canardGetLocalNodeID(&canard_) == CANARD_BROADCAST_NODE_ID) &&
            (transfer->transfer_type == CanardTransferTypeBroadcast) &&
            (transfer->data_type_id == dsdl::NodeIDAllocation::DataTypeID))
    {
        // Rule C - updating the randomized time interval
        send_next_node_id_allocation_request_at_ =
                getMonotonicTimestampUSec() + getRandomDurationMicrosecond(600000, 1000000);

        if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
        {
            node_id_allocation_unique_id_offset_ = 0;
            return;
        }

        // Copying the unique ID from the message
        static constexpr unsigned UniqueIDBitOffset = 8;
        std::uint8_t received_unique_id[hw_info_.unique_id.size()];
        std::uint8_t received_unique_id_len = 0;
        for (;
                received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U));
                received_unique_id_len++)
        {
            assert(received_unique_id_len < hw_info_.unique_id.size());
            const std::uint8_t bit_offset = std::uint8_t(UniqueIDBitOffset + received_unique_id_len * 8U);
            (void) canardDecodeScalar(transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
        }

        // Matching the received UID against the local one
        if (memcmp(received_unique_id, hw_info_.unique_id.data(), received_unique_id_len) != 0)
        {
            node_id_allocation_unique_id_offset_ = 0;
            return; // No match, return
        }

        if (received_unique_id_len < hw_info_.unique_id.size())
        {
            // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
            node_id_allocation_unique_id_offset_ = received_unique_id_len;
            send_next_node_id_allocation_request_at_ =
                    getMonotonicTimestampUSec() + getRandomDurationMicrosecond(0, 400000);
        }
        else
        {
            // Allocation complete - copying the allocated node ID from the message
            std::uint8_t allocated_node_id = 0;
            (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
            assert(allocated_node_id <= 127);

            canardSetLocalNodeID(&canard_, allocated_node_id);
        }
    }

    /*
     * GetNodeInfo request.
     * Someday this mess should be replaced with auto-generated message serialization code, like in libuavcan.
     */
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
            (transfer->data_type_id == dsdl::GetNodeInfo::DataTypeID))
    {
        uavcan_protocol_GetNodeInfoResponse node_info;
        uint8_t node_info_buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

        uavcan_protocol_NodeStatus node_status;
        node_status.uptime_sec = getMonotonicTimestampUSec() / 1000000UL;


        std::uint8_t node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
        std::uint8_t node_mode = std::uint8_t(impl_::dsdl::NodeMode::Maintenance);

        switch (can_module_state)
        {
            case State::NoAppToBoot:
            {
                node_mode = std::uint8_t(impl_::dsdl::NodeMode::SoftwareUpdate);
                node_health = std::uint8_t(impl_::dsdl::NodeHealth::Error);
                break;
            }
            case State::AppUpgradeInProgress:
            {
                node_mode = std::uint8_t(impl_::dsdl::NodeMode::SoftwareUpdate);
                break;
            }
            case State::BootCancelled:
            {
                node_health = std::uint8_t(impl_::dsdl::NodeHealth::Warning);
                break;
            }
            case State::NormalOperation:
            {
              node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
              node_mode = std::uint8_t(impl_::dsdl::NodeMode::NormalOperation);
              break;
            }
            case State::Initialization:
            {
              node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
              node_mode = std::uint8_t(impl_::dsdl::NodeMode::Initialization);
              break;
            }
            case State::Error:
            {
              node_health = std::uint8_t(impl_::dsdl::NodeHealth::Error);
              node_mode = std::uint8_t(impl_::dsdl::NodeMode::Initialization);
              break;
            }
            case State::BootDelay:
            case State::ReadyToBoot:
            {
                break;
            }
        }


        node_status.health = node_health;
        node_status.mode = node_mode;
        node_status.sub_mode = 0;
        node_status.vendor_specific_status_code = 0;
        //let us assign this to the data object
        node_info.status = node_status;
        /*
          Let us construct a node information object
        */
        uavcan_protocol_SoftwareVersion software_version;
        software_version.major = 1;
        software_version.minor = 0;
        software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT;
        software_version.vcs_commit = FIRMWARE_VERSION; //The actual git hash. Generated by VF
        software_version.image_crc = 0; //currently do not care about this

        node_info.software_version = software_version;

        uavcan_protocol_HardwareVersion hardware_version;
        hardware_version.major = 1;
        hardware_version.minor = 0;
        hardware_version.certificate_of_authenticity.len = 0;
        //get and copy the unique id into the data array
        UniqueID unique_id = readUniqueID();
        for(uint8_t i = 0; i < unique_id.size(); i++)
        {
          hardware_version.unique_id[i] = unique_id[i];
        }

        node_info.hardware_version = hardware_version;

        //copy node name to our object
        std::copy(node_name_.begin(), node_name_.end(), std::begin(node_info.name.data));
        node_info.name.len = node_name_.length();

        uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&node_info, node_info_buffer);

        (void) canardRequestOrRespond(&canard_,
                transfer->source_node_id,
                UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                UAVCAN_PROTOCOL_GETNODEINFO_ID,
                &transfer->transfer_id,
                transfer->priority,
                CanardResponse,
                &node_info_buffer,
                len);
        can_tx_count++;
    }

    /*
     * RestartNode request.
     */
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
            (transfer->data_type_id == dsdl::RestartNode::DataTypeID))
    {
        std::uint8_t response = 0; // 1 - ok, 0 - rejected

        std::uint64_t magic_number = 0;
        (void) canardDecodeScalar(transfer, 0, 40, false, &magic_number);

        if (magic_number == 0xACCE551B1E)
        {
            response = 1 << 7;
            //TODO: Actually hand this off to somewhere else. We do not get to the next part where we actually need to reboot.
            bootloader_info_location = RAM_DO_BOOTLOADER_BYTE;
            signal_reboot = true;//NVIC_SystemReset();
        }

        // No need to release the transfer payload, it's single frame anyway
        (void) canardRequestOrRespond(&canard_,
                transfer->source_node_id,
                dsdl::RestartNode::DataTypeSignature,
                dsdl::RestartNode::DataTypeID,
                &transfer->transfer_id,
                transfer->priority,
                CanardResponse,
                &response,
                1);
        can_tx_count++;
    }
    /*
     * Transport Stats Request
     */
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
            (transfer->data_type_id == UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID))
    {
      //This actually just sends an empty object. The fact that we are here is enough to solicit a response.
      uavcan_protocol_GetTransportStatsResponse response;
//TODO: FIX THIS! So we actually keep track of tranfers
  CanardSTM32Stats status = canardSTM32GetStats();
      response.transfers_tx = can_tx_count;
      response.transfers_rx = get_can_rx_count();
      response.transfer_errors = status.error_count;
      response.can_iface_stats.len = 0;

      uint8_t transport_status_buffer[30];

      uint32_t len = uavcan_protocol_GetTransportStatsResponse_encode(&response,transport_status_buffer);

      canardReleaseRxTransferPayload(&canard_, transfer);

        (void) canardRequestOrRespond(&canard_,
                transfer->source_node_id,
                UAVCAN_PROTOCOL_GETTRANSPORTSTATS_SIGNATURE,
                UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID,
                &transfer->transfer_id,
                transfer->priority,
                CanardResponse,
                &transport_status_buffer,
                len);
        can_tx_count++;
    }

#ifdef LIBCANARD_MESSAGE_FIRMWARE_UPGRADE
    /*
     * BeginFirmwareUpdate request.
     */
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
            (transfer->data_type_id == dsdl::BeginFirmwareUpdate::DataTypeID))
    {
        std::uint8_t error = 0;

        if ((can_module_state == State::AppUpgradeInProgress) || (remote_server_node_id_ != 0))
        {
            error = 2; // Already in progress
        }
        else if ((can_module_state == State::ReadyToBoot))
        {
            error = 1; // Invalid mode
        }
        else
        {
            // Determine the node ID of the firmware server
            (void) canardDecodeScalar(transfer, 0, 8, false, &remote_server_node_id_);
            if ((remote_server_node_id_ == 0) ||
                    (remote_server_node_id_ >= CANARD_MAX_NODE_ID))
            {
                remote_server_node_id_ = transfer->source_node_id;
            }

            // Copy the path
            firmware_file_path_.clear();
            for (unsigned i = 0;
                    i < (transfer->payload_len - 1U);
                    i++)
            {
                char val = '\0';
                (void) canardDecodeScalar(transfer, i * 8 + 8, 8, false, &val);
                firmware_file_path_.push_back(val);
            }
            //tell the driver to begin requesting file chunks
            download_state = DownloadState::DownloadStateDownloading;
            //tell the driver it is OK to ask for a chunk
            //            request_next_chunk = true;
            read_result_ = std::numeric_limits<int>::max();

            can_module_state = State::AppUpgradeInProgress;
            //erase the first page so we don't think there is anything to boot
            error = 0;
            retry_count = 0;
            transfer_started = true;
            sendNodeStatus();
        }

        canardReleaseRxTransferPayload(&canard_, transfer);
        canardRequestOrRespond(&canard_,
                transfer->source_node_id,
                dsdl::BeginFirmwareUpdate::DataTypeSignature,
                dsdl::BeginFirmwareUpdate::DataTypeID,
                &transfer->transfer_id,
                transfer->priority,
                CanardResponse,
                &error,
                1);
        can_tx_count++;
    }

    /*
     * File read response.
     */
    if ((transfer->transfer_type == CanardTransferTypeResponse) &&
            (transfer->data_type_id == dsdl::FileRead::DataTypeID) &&
            (((transfer->transfer_id + 1) & 31) == file_read_transfer_id_))
    {
        std::uint16_t error = 0;
        (void) canardDecodeScalar(transfer, 0, 16, false, &error);
        if (error != 0)
        {
            read_result_ = -error;
        }
        else
        {
            read_result_ = std::min(256, transfer->payload_len - 2);
            for (int i = 0; i < read_result_; i++)
            {
                (void) canardDecodeScalar(transfer, 16 + i * 8, 8, false, &read_buffer_[i]);
            }
            //tell the driver to ask for another chunk
            retry_count = 0;
            response_recieved = true;
            request_next_chunk = true;
        }
    }
#endif
#ifdef LIBCANARD_MESSAGE_PARAMETERS
  /**
    Handles all the messages regarding parameters. Components might not want this, especially the bootloader_
  */
  if ((transfer->transfer_type == CanardTransferTypeRequest) &&
      (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID))
      {
        //to decode a string variable, we need somewhere to put it.

        uint8_t buffer[50]; //50 characters!!? This can only go wrong.
        uint8_t * buff_ptr;
        //let us move the ptr to the buffers
        buff_ptr = buffer;

        //create a parameter request
        uavcan_protocol_param_GetSetRequest id_request;
        uavcan_protocol_param_GetSetRequest_decode(transfer, &id_request);
        int16_t can_param_index = -1;
        //we have no value here, therefore we are wanting some information
        if(id_request.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY)
        {
          //if we have no name len, then we are being asked to list the parameter
          //we are being asked to update a parameter
          //Here we check to see if parameter exists by name.


          //we will only ever use the index if no name is supplied.
          if(id_request.name.len == 0)
          {
            if(can_param_exists(id_request.index))
              {
                can_param_index = (int16_t)id_request.index;
              }
          }
          else
          {
            can_param_index = can_param_exists_by_name(id_request.name.data, id_request.name.len);
          }
        }
        else
        {
            can_param_index = can_param_exists_by_name(id_request.name.data, id_request.name.len);
            if(can_param_index >= 0)
            {
              can_parameters[can_param_index].value = id_request.value;

              if(id_request.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE)
                can_parameters[can_param_index].value.integer_value = id_request.value.integer_value;
              if(id_request.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE)
                can_parameters[can_param_index].value.real_value = id_request.value.real_value;
              if(id_request.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE)
                can_parameters[can_param_index].value.boolean_value = id_request.value.boolean_value;
            }
            sendLog(impl_::LogLevel::Info, "Updating ID");
        }

          //This is an empty packet. If we can't find a parameter, we will respond with a null object. 
          uavcan_protocol_param_GetSetResponse resp;
          resp.name.len = 0;
          //wohoo, we have a parameter
          if (can_param_index >= 0)
          {
            resp = can_parameters[can_param_index];
          }
          //This buffer is significantly smaller than the potential size of the array.
          //we do not care as we define the string size, thus why the string is the array size factor.
          uint8_t get_set_buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
          //encode the value
          uint32_t get_set_buffer_length = uavcan_protocol_param_GetSetResponse_encode(&resp, get_set_buffer);


          //we have been asked for parameters, let us send some back
          canardRequestOrRespond(&canard_,
                  transfer->source_node_id,
                  UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                  UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                  &transfer->transfer_id,
                  transfer->priority,
                  CanardResponse,
                  get_set_buffer,
                  get_set_buffer_length);
          can_tx_count++;
        
      }

      if ((transfer->transfer_type == CanardTransferTypeRequest) &&
          (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID))
          {
            uavcan_protocol_param_ExecuteOpcodeRequest op_code_request;
            uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &op_code_request);

            //we have been asked to do something
            bool succeeded = false;
            if(op_code_request.opcode  == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE)
            {
              //store the data.
              succeeded = save_can_parameters(false);
              sendLog(impl_::LogLevel::Info, "Saving Parameters");
            }
            if(op_code_request.opcode  == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE)
            {
              //store the data, but break the versioning. Then go down for a reboot.
              succeeded = save_can_parameters(true);
              signal_reboot = true;
              sendLog(impl_::LogLevel::Info, "Erasing Parameters");
            }

            uavcan_protocol_param_ExecuteOpcodeResponse op_code_response;
            op_code_response.ok = succeeded;
            op_code_response.argument = 0;
            uint8_t op_code_buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
            uint32_t length = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&op_code_response, op_code_buffer);


            //we have executed the opcode, let us send the response back
            canardRequestOrRespond(&canard_,
                    transfer->source_node_id,
                    UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                    UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                    &transfer->transfer_id,
                    transfer->priority,
                    CanardResponse,
                    op_code_buffer,
                    length);
            can_tx_count++;
          }
#endif



    /**
      As we have received an RX. Let us increment the counter.
    */
    increment_can_rx_count();
    /**
      Let us ask the driver host to spread the transfer around. This means that each module can subscribe to what ever transfer that they require.
      data signature is not used when internally transferring
    */

    driverhost_broadcast_can(transfer, 0, transfer->data_type_id, &transfer->transfer_id, transfer->priority, transfer->payload_head, transfer->payload_len, this);
    /**
      Let us clear the transfer for Justin Case. This might already be released, in this case we have re-released it. Noice.
    */
    canardReleaseRxTransferPayload(&canard_, transfer);
}

bool Libcanard_module::shouldAcceptTransfer(std::uint64_t* out_data_type_signature,
        std::uint16_t data_type_id,
        CanardTransferType transfer_type,
        std::uint8_t source_node_id)
{
    using namespace impl_::dsdl;

    (void) source_node_id;

    if (canardGetLocalNodeID(&canard_) == CANARD_BROADCAST_NODE_ID)
    {
        // Dynamic node ID allocation broadcast
        if ((transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == NodeIDAllocation::DataTypeID))
        {
            *out_data_type_signature = NodeIDAllocation::DataTypeSignature;
            return true;
        }
    }
    else
    {
        // GetNodeInfo REQUEST
        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == GetNodeInfo::DataTypeID))
        {
            *out_data_type_signature = GetNodeInfo::DataTypeSignature;
            return true;
        }

        // BeginFirmwareUpdate REQUEST
        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == BeginFirmwareUpdate::DataTypeID))
        {
            *out_data_type_signature = BeginFirmwareUpdate::DataTypeSignature;
            return true;
        }

        // FileRead RESPONSE (we don't serve requests of this type)
        if ((transfer_type == CanardTransferTypeResponse) &&
                (data_type_id == FileRead::DataTypeID))
        {
            *out_data_type_signature = FileRead::DataTypeSignature;
            return true;
        }

        // RestartNode REQUEST
        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == RestartNode::DataTypeID))
        {
            *out_data_type_signature = RestartNode::DataTypeSignature;
            return true;
        }
#ifdef LIBCANARD_MESSAGE_PARAMETERS
        //parameter get/set
        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }

        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
#endif

        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETTRANSPORTSTATS_SIGNATURE;
            return true;
        }
#ifdef LIBCANARD_MESSAGE_ARRAYCOMMAND
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID))
        {
            *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
            return true;
        }
#endif
#ifdef LIBCANARD_MESSAGE_LIGHTSCOMMAND
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID))
        {
            *out_data_type_signature = UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_SIGNATURE;
            return true;
        }
#endif
#ifdef LIBCANARD_MESSAGE_NOTIFYSTATE
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == ARDUPILOT_INDICATION_NOTIFYSTATE_ID))
        {
            *out_data_type_signature = ARDUPILOT_INDICATION_NOTIFYSTATE_SIGNATURE;
            return true;
        }
#endif


#ifdef LIBCANARD_MESSAGE_ARMINGSTATUS
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID))
        {
            *out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
            return true;
        }
#endif

    }

    return false;
}

void Libcanard_module::onTransferReceptionTrampoline(CanardInstance* ins,
        CanardRxTransfer * transfer)
{
    assert((ins != nullptr) && (ins->user_reference != nullptr));
    Libcanard_module * const self = reinterpret_cast<Libcanard_module*> (ins->user_reference);
    self->onTransferReception(transfer);
}

bool Libcanard_module::shouldAcceptTransferTrampoline(const CanardInstance* ins,
        std::uint64_t* out_data_type_signature,
        std::uint16_t data_type_id,
        CanardTransferType transfer_type,
        std::uint8_t source_node_id)
{
    assert((ins != nullptr) && (ins->user_reference != nullptr));
    Libcanard_module * const self = reinterpret_cast<Libcanard_module*> (ins->user_reference);
    return self->shouldAcceptTransfer(out_data_type_signature,
            data_type_id,
            transfer_type,
            source_node_id);
}

int Libcanard_module::initCAN(const std::uint32_t bitrate, const Mode mode, const AcceptanceFilterConfig & acceptance_filter)
{
    const int res = init(bitrate, mode, acceptance_filter);
    return res;
}

#endif

std::uint64_t Libcanard_module::getRandomDurationMicrosecond(std::uint64_t lower_bound_usec, std::uint64_t upper_bound_usec) const
{
    const std::uint64_t rnd = std::uint64_t(rand()) * 128UL;
    return lower_bound_usec + rnd % (upper_bound_usec - lower_bound_usec);
}

void Libcanard_module::makeNodeStatusMessage(std::uint8_t * buffer) const
{
    memset(buffer, 0, impl_::dsdl::NodeStatus::MaxSizeBytes);

    const std::uint64_t uptime_sec = getMonotonicTimestampUSec() / 1000000UL;

    /*
     * Bootloader State        Node Mode       Node Health
     * ----------------------------------------------------
     * NoAppToBoot             SoftwareUpdate  Error
     * BootDelay               Maintenance     Ok
     * BootCancelled           Maintenance     Warning
     * AppUpgradeInProgress    SoftwareUpdate  Ok
     * ReadyToBoot             Maintenance     Ok
     * NormalOperation         Normal operation Ok
     * Initialization          Initialization  Ok
     */
    std::uint8_t node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
    std::uint8_t node_mode = std::uint8_t(impl_::dsdl::NodeMode::Maintenance);

    switch (can_module_state)
    {
        case State::NoAppToBoot:
        {
            node_mode = std::uint8_t(impl_::dsdl::NodeMode::SoftwareUpdate);
            node_health = std::uint8_t(impl_::dsdl::NodeHealth::Error);
            break;
        }
        case State::AppUpgradeInProgress:
        {
            node_mode = std::uint8_t(impl_::dsdl::NodeMode::SoftwareUpdate);
            break;
        }
        case State::BootCancelled:
        {
            node_health = std::uint8_t(impl_::dsdl::NodeHealth::Warning);
            break;
        }
        case State::NormalOperation:
        {
          node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
          node_mode = std::uint8_t(impl_::dsdl::NodeMode::NormalOperation);
          break;
        }
        case State::Initialization:
        {
          node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
          node_mode = std::uint8_t(impl_::dsdl::NodeMode::Initialization);
          break;
        }
        case State::Error:
        {
          node_health = std::uint8_t(impl_::dsdl::NodeHealth::Error);
          node_mode = std::uint8_t(impl_::dsdl::NodeMode::Initialization);
          break;
        }
        case State::BootDelay:
        case State::ReadyToBoot:
        {
            break;
        }
    }

    canardEncodeScalar(buffer, 0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32, 2, &node_health);
    canardEncodeScalar(buffer, 34, 3, &node_mode);
    canardEncodeScalar(buffer, 40, 16, &vendor_specific_status_);
}

void Libcanard_module::performDynamicNodeIDAllocation()
{
#if defined(STM32F1XX) || defined(STM32L4XX)
    // CAN bus initialization
    while (true)
    {
        if (kill_loading)
            return;
        // Accept only anonymous messages with DTID = 1 (Allocation)
        // Observe that we need both responses from allocators and requests from other nodes!
        AcceptanceFilterConfig filt;
        filt.id = 0b00000000000000000000100000000 | CANARD_CAN_FRAME_EFF;
        filt.mask = 0b00000000000000000001110000000 | CANARD_CAN_FRAME_EFF | CANARD_CAN_FRAME_RTR |
                CANARD_CAN_FRAME_ERR;

        if (initCAN(can_bus_bit_rate_, Mode::AutomaticTxAbortOnError, filt) >= 0)
        {
            break;
        }

        //        delayAfterDriverError();
    }
#endif

    using namespace impl_;

    while ((canardGetLocalNodeID(&canard_) == 0))
    {
        if (kill_loading)
            return;
        //        watchdog_.reset();

        send_next_node_id_allocation_request_at_ =
                getMonotonicTimestampUSec() + getRandomDurationMicrosecond(600000, 1000000);

        while ((getMonotonicTimestampUSec() < send_next_node_id_allocation_request_at_) &&
                (canardGetLocalNodeID(&canard_) == 0))
        {
            if (kill_loading)
                return;
            poll();
        }

        if (canardGetLocalNodeID(&canard_) != 0)
        {
            break;
        }

        // Structure of the request is documented in the DSDL definition
        // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
        std::uint8_t allocation_request[7]{};

        if (node_id_allocation_unique_id_offset_ == 0)
        {
            allocation_request[0] |= 1; // First part of unique ID
        }

        static constexpr std::uint8_t MaxLenOfUniqueIDInRequest = 6;
        std::uint8_t uid_size = std::uint8_t(hw_info_.unique_id.size() - node_id_allocation_unique_id_offset_);
        if (uid_size > MaxLenOfUniqueIDInRequest)
        {
            uid_size = MaxLenOfUniqueIDInRequest;
        }

        // Paranoia time
        assert(node_id_allocation_unique_id_offset_ < hw_info_.unique_id.size());
        assert(uid_size <= MaxLenOfUniqueIDInRequest);
        assert(uid_size > 0);
        assert((uid_size + node_id_allocation_unique_id_offset_) <= hw_info_.unique_id.size());

        memmove(&allocation_request[1], &hw_info_.unique_id[node_id_allocation_unique_id_offset_], uid_size);

        // Broadcasting the request
        canardBroadcast(&canard_,
                dsdl::NodeIDAllocation::DataTypeSignature,
                dsdl::NodeIDAllocation::DataTypeID,
                &node_id_allocation_transfer_id_,
                CANARD_TRANSFER_PRIORITY_LOW,
                &allocation_request[0],
                std::uint16_t(uid_size + 1U)
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
        );
        //increment the CAN tx counter
        can_tx_count++;
        // Preparing for timeout; if response is received, this value will be updated from the callback.
        node_id_allocation_unique_id_offset_ = 0;
    }

    //    watchdog_.reset();
}

void Libcanard_module::poll()
{
    constexpr int MaxFramesPerSpin = 10;
    // Receive
    for (int i = 0; i < MaxFramesPerSpin; i++)
    {
        #if defined(STM32F1XX) || defined(STM32L4XX)
        const auto res = receive(1); // Blocking call
        if (res.first < 1)
        {
            break; // Error or no frames
        }

        canardHandleRxFrame(&canard_, &res.second, getMonotonicTimestampUSec());
        #elif defined(STM32G4XX)
        if (rx_queue.empty())
        {
            break; // No frames
        }
        CanardCANFrame frame = rx_queue.front();
        rx_queue.pop();
        HAL_GPIO_TogglePin(STATUS2_GPIO_Port, STATUS2_Pin);
        canardHandleRxFrame(&canard_, &frame, getMonotonicTimestampUSec());
        #endif
        
    }

    // Transmit
    for (int i = 0; i < MaxFramesPerSpin; i++)
    {
        const CanardCANFrame* txf = canardPeekTxQueue(&canard_);
        if (txf == nullptr)
        {
            break; // Nothing to transmit
        }

        #if defined(STM32F1XX) || defined(STM32L4XX)
        const int res = send(*txf, 0); // Non-blocking call
        if (res == 0)
        {
            break; // Queue is full
        }
        #elif defined(STM32G4XX)
        FDCAN_TxHeaderTypeDef TxHeader;
        TxHeader.Identifier = txf->id;
        TxHeader.IdType = FDCAN_EXTENDED_ID;
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;
        TxHeader.DataLength = txf->data_len;
        TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
        TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
        TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker = 0;

#ifdef CANARD_MULTI_IFACE
				if ((txf->iface_mask >> 0) & 0x1 == 1)
				{
#endif
					if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txf->data) != HAL_OK)
					{
							/* Transmission request Error */
							Error_Handler();
					}
#ifdef CANARD_MULTI_IFACE
				}
				if ((txf->iface_mask >> 1) & 0x1 == 1)
				{
					if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, txf->data) != HAL_OK)
					{
						/* Transmission request Error */
						Error_Handler();
					}
				}
#endif
        #endif

        canardPopTxQueue(&canard_); // Transmitted successfully or error, either way remove the frame
        HAL_Delay(1);
    }

//    // 1Hz process
//    if (getMonotonicTimestampUSec() >= next_1hz_task_invocation_)
//    {
//        next_1hz_task_invocation_ += 1000000UL;
//        handle1HzTasks();
//    }
}

void Libcanard_module::handle1HzTasks()
{
    canardCleanupStaleTransfers(&canard_, getMonotonicTimestampUSec());

    // NodeStatus broadcasting
    if (init_done_ && (canardGetLocalNodeID(&canard_) > 0))
    {
        sendNodeStatus();
    }
}

void Libcanard_module::sendNodeStatus()
{
    using namespace impl_;
    std::uint8_t buffer[dsdl::NodeStatus::MaxSizeBytes]{};
    makeNodeStatusMessage(buffer);
    canardBroadcast(&canard_,
            dsdl::NodeStatus::DataTypeSignature,
            dsdl::NodeStatus::DataTypeID,
            &node_status_transfer_id_,
            CANARD_TRANSFER_PRIORITY_LOW,
            buffer,
            dsdl::NodeStatus::MaxSizeBytes
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
    );
    //increment the CAN counter
    can_tx_count ++;

}

int usleep(uint32_t useconds)
{
    uint64_t delay = driverhost_get_monotonic_time_us() + useconds;
    while (driverhost_get_monotonic_time_us() < delay)
    {
      __NOP();
    }
    return 0;
}

UniqueID Libcanard_module::readUniqueID()
{
    UniqueID out_bytes;
    memcpy(out_bytes.data(), reinterpret_cast<const void*> (UID_BASE), (size_t)12);
    return out_bytes;
}

void Libcanard_module::sendLog(const impl_::LogLevel level, const std::string & txt)
{
    static const std::string SourceName = node_name_;
    std::uint8_t buffer[1 + 31 + 90]{};
    buffer[0] = (std::uint8_t(level) << 5) | SourceName.length();
    std::copy(SourceName.begin(), SourceName.end(), &buffer[1]);
    std::copy(txt.begin(), txt.end(), &buffer[1 + SourceName.length()]);

    using impl_::dsdl::LogMessage;
    canardBroadcast(&canard_,
            LogMessage::DataTypeSignature,
            LogMessage::DataTypeID,
            &log_message_transfer_id_,
            CANARD_TRANSFER_PRIORITY_LOWEST,
            buffer,
            1 + SourceName.length() + txt.length()
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
    );

    //increment counter
    can_tx_count++;
}
//Handles driver broadcasting and enqueues a frame to send.
void Libcanard_module::handle_rx_can(const CanardRxTransfer * transfer, uint64_t data_type_signature, uint16_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len)
{
    //check to see if the CAN 
    if(!can_enabled)
    {
        return;
    }
    canardBroadcast(&canard_,
          data_type_signature,
          data_type_id,
          inout_transfer_id,
          priority,
          payload,
          payload_len
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
    );
          //increment the CAN TX counter
    can_tx_count++;
}

#ifdef LIBCANARD_MESSAGE_PARAMETERS

    bool Libcanard_module::can_param_exists(uint16_t index)
    {
      if(index < NUM_CAN_PARAMS)
        return true;
      return false;
    }

    int16_t  Libcanard_module::can_param_exists_by_name(uint8_t * name, uint8_t len)
    {
      for(uint8_t i = 0; i < NUM_CAN_PARAMS; i++ )
      {
        //we do not want to compare the \0
        if(strncmp((char *) name, (char *) can_parameters[i].name.data,can_parameters[i].name.len-1) == 0)
        {
          return i;
        }
      }
      return -1;
    }
    uavcan_protocol_param_GetSetResponse * get_can_param_by_id(uint16_t id)
    {
      if(Libcanard_module::get_driver().can_param_exists(id))
      {
        return &can_parameters[id];
      }
      //return nothing if the parameter we are asking for does not exist.
      return nullptr;
    }
    uavcan_protocol_param_GetSetResponse * Libcanard_module::get_can_param_by_name(uint8_t * name)
    {
      return nullptr;
    }

    void Libcanard_module::request_save_parameters(void)
    {
      //store can parameters.
      save_can_parameters(false);
      return;
    }
    void Libcanard_module::request_erase_parameters(void)
    {
      //load new can parameters on next boot.
      save_can_parameters(true);
      return;
    }
    bool Libcanard_module::save_can_parameters(bool overwrite_with_defaults = false)
    {
      uint8_t can_param_version  = CAN_PARAM_COMPAT_VERSION; //provided by the can_params.hpp
      //Check to  determine if we need to override with defaults:
      if(overwrite_with_defaults)
      {
        //break the versioning so we know when booting to NOT use the default loaded values
        can_param_version = 0;
      }

      //get the physical location of where we are going to put the data.
      const volatile uint64_t *param_values = (const volatile uint64_t *)(CAN_EEPROM_LOCATION);
      Flash_status_t status_block;
      status_block.uint64_block = *param_values;

      /*
       * First two bytes are flag and version
       */
      status_block.uint8_block[0] = CAN_EEPROM_FLAG;
      status_block.uint8_block[1] = can_param_version;
      //increment the counter
      status_block.uint16_block[1]++;

      /**
       * Erase the memory
       */
#ifdef STM32F1XX
#define ERASE_EEPROM_PAGE_DEFINED
      erase_eeprom_page(CAN_EEPROM_LOCATION);
#endif
#if defined(STM32G4XX) || defined(STM32L4XX)
#define ERASE_EEPROM_PAGE_DEFINED
      erase_eeprom_page_by_number(CAN_EEPROM_PAGE_NUM);
#endif
#ifndef ERASE_EEPROM_PAGE_DEFINED
#error No erase_eeprom_page defined for this chip
#endif
      /**
       * Write the status
       */
      store_in_flash(CAN_EEPROM_LOCATION, status_block.uint64_block);

      /**
       * Increment the next page, because we do not want to interfere with our boot options
       */
      uint32_t address = CAN_EEPROM_LOCATION + sizeof (uint64_t);
      /*
       * Write parameters at the start of the next word
       */
      address += sizeof (uint64_t);
      for (uint8_t i = 0; i < NUM_CAN_PARAMS; i++)
      {
        //Disable the strict aliasing rule
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wstrict-aliasing"
          uint64_t data;
          if (can_parameters[i].value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE)
          {
              data = *reinterpret_cast<uint64_t*> (&(can_parameters[i].value.integer_value));
          }
          else if (can_parameters[i].value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE)
          {
              data = *reinterpret_cast<uint64_t*> (&(can_parameters[i].value.real_value));
          }
          else if (can_parameters[i].value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE)
          {
              data = *reinterpret_cast<uint64_t*> (&(can_parameters[i].value.boolean_value));
          }
          else
          {
            //we are trying for an unused type
            return false;
          }
          store_in_flash(address, data);
          address += sizeof (uint64_t);
        #pragma GCC diagnostic pop
      }
      return true;
    }
    bool Libcanard_module::load_can_parameters(void)
    {

      // NOTE - This will block for some time.  Interrupts are disabled.  There isn't much you can do about this though.
      bool parameters_loaded = false;
      /**
       * Read the first 32 bytes of Flash storage.
       */
      const volatile uint64_t *param_values = (const volatile uint64_t *)(CAN_EEPROM_LOCATION);
      Flash_status_t status_block;
      status_block.uint64_block = *param_values;
      /*
       * First two bytes are flag and version
       */
      uint8_t flag = status_block.uint8_block[0];
      uint8_t version = status_block.uint8_block[1];
      /**
       * The next two bytes are a counter to check how many writes are performed
       */
      //uint16_t storage_counter = status_block.uint16_block[1];
      if (flag == CAN_EEPROM_FLAG && version == CAN_PARAM_COMPAT_VERSION)
      {
          //move onto the first param (the boot flags)
          param_values++;
          //move onto the second parameter
          param_values++;
          for (uint8_t i = 0; i < NUM_CAN_PARAMS; i++)
          {
              //Check what kind of data is used.
              uint64_t* data;
              if (can_parameters[i].value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE)
              {
                  data = reinterpret_cast<uint64_t*> (&(can_parameters[i].value.integer_value));
              }
              else if (can_parameters[i].value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE)
              {
                  data = reinterpret_cast<uint64_t*> (&(can_parameters[i].value.real_value));
              }
              else if (can_parameters[i].value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE)
              {
                  data = reinterpret_cast<uint64_t*> (&(can_parameters[i].value.boolean_value));
              }
              else {
                //we are trying for an unused type!
                return false;
              }
              *data = *param_values;
              param_values++; //= sizeof (uint32_t);
          }
          parameters_loaded = true;
      }
      // All done.
      return parameters_loaded;
    }

    /**
     * Stores 8 bytes (1 can parameter' raw value) of data in flash memory as specified
     * @param address
     * @param data
     */
    void Libcanard_module::store_in_flash(uint32_t address, uint64_t data)
    {
        /**
         * Start by unlocking the flash partition
         */
        HAL_FLASH_Unlock();

        /**
         * Write the 64Bit data word
         */
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);

        /**
         * End by locking the flash partition
         */
        HAL_FLASH_Lock();
    }

#ifdef STM32F1XX
    /**
     * Erases a specified page by address
     * @param address
     */
    void Libcanard_module::erase_eeprom_page_by_address(uint32_t address)
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
     * Erases a specified page by page number
     * @param page number
     */
    void Libcanard_module::erase_eeprom_page_by_number(uint32_t page_number)
    {
        /**
         * Unlock flashing ability
         */
        HAL_FLASH_Unlock();
        uint32_t PageError = 0;
        FLASH_EraseInitTypeDef pErase;
        pErase.NbPages = 1; //single page
        pErase.Page = page_number;
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



#endif
/**
* Here we define and create each message we actually want to send.
*/

#ifdef LIBCANARD_MESSAGE_FIRMWARE_UPGRADE

  bool Libcanard_module::is_boot_image_available(void)
  {
      if (((*(__IO uint32_t*) (APPLICATION_ADDRESS + APPLICATION_OFFSET)) & 0x2FFE0000) == 0x20000000)
      {
          return true;
      }
      return false;
  }

  bool Libcanard_module::boot(void)
  {
      if (is_boot_image_available())
      {
          Set_Boot();
          NVIC_SystemReset();
      }
  }
#endif


#if defined(STM32G4XX)
void Libcanard_module::add_to_rx_queue(CanardCANFrame can_frame)
{
    rx_queue.push(can_frame);
}

void Libcanard_module::onTransferReception(CanardRxTransfer * const transfer)
{
    using namespace impl_;


    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     * This piece of dark magic has been carefully transplanted from the libcanard demo application.
     */
    if ((canardGetLocalNodeID(&canard_) == CANARD_BROADCAST_NODE_ID) &&
            (transfer->transfer_type == CanardTransferTypeBroadcast) &&
            (transfer->data_type_id == dsdl::NodeIDAllocation::DataTypeID))
    {
        // Rule C - updating the randomized time interval
        send_next_node_id_allocation_request_at_ =
                getMonotonicTimestampUSec() + getRandomDurationMicrosecond(600000, 1000000);

        if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
        {
            node_id_allocation_unique_id_offset_ = 0;
            return;
        }

        // Copying the unique ID from the message
        static constexpr unsigned UniqueIDBitOffset = 8;
        std::uint8_t received_unique_id[hw_info_.unique_id.size()];
        std::uint8_t received_unique_id_len = 0;
        for (;
                received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U));
                received_unique_id_len++)
        {
            assert(received_unique_id_len < hw_info_.unique_id.size());
            const std::uint8_t bit_offset = std::uint8_t(UniqueIDBitOffset + received_unique_id_len * 8U);
            (void) canardDecodeScalar(transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
        }

        // Matching the received UID against the local one
        if (memcmp(received_unique_id, hw_info_.unique_id.data(), received_unique_id_len) != 0)
        {
            node_id_allocation_unique_id_offset_ = 0;
            return; // No match, return
        }

        if (received_unique_id_len < hw_info_.unique_id.size())
        {
            // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
            node_id_allocation_unique_id_offset_ = received_unique_id_len;
            send_next_node_id_allocation_request_at_ =
                    getMonotonicTimestampUSec() + getRandomDurationMicrosecond(0, 400000);
        }
        else
        {
            // Allocation complete - copying the allocated node ID from the message
            std::uint8_t allocated_node_id = 0;
            (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
            assert(allocated_node_id <= 127);

            canardSetLocalNodeID(&canard_, allocated_node_id);
        }
    }

    /*
     * GetNodeInfo request.
     * Someday this mess should be replaced with auto-generated message serialization code, like in libuavcan.
     */
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
            (transfer->data_type_id == dsdl::GetNodeInfo::DataTypeID))
    {
        uavcan_protocol_GetNodeInfoResponse node_info;
        uint8_t node_info_buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

        uavcan_protocol_NodeStatus node_status;
        node_status.uptime_sec = getMonotonicTimestampUSec() / 1000000UL;


        std::uint8_t node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
        std::uint8_t node_mode = std::uint8_t(impl_::dsdl::NodeMode::Maintenance);

        switch (can_module_state)
        {
            case State::NoAppToBoot:
            {
                node_mode = std::uint8_t(impl_::dsdl::NodeMode::SoftwareUpdate);
                node_health = std::uint8_t(impl_::dsdl::NodeHealth::Error);
                break;
            }
            case State::AppUpgradeInProgress:
            {
                node_mode = std::uint8_t(impl_::dsdl::NodeMode::SoftwareUpdate);
                break;
            }
            case State::BootCancelled:
            {
                node_health = std::uint8_t(impl_::dsdl::NodeHealth::Warning);
                break;
            }
            case State::NormalOperation:
            {
              node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
              node_mode = std::uint8_t(impl_::dsdl::NodeMode::NormalOperation);
              break;
            }
            case State::Initialization:
            {
              node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
              node_mode = std::uint8_t(impl_::dsdl::NodeMode::Initialization);
              break;
            }
            case State::Error:
            {
              node_health = std::uint8_t(impl_::dsdl::NodeHealth::Error);
              node_mode = std::uint8_t(impl_::dsdl::NodeMode::Initialization);
              break;
            }
            case State::BootDelay:
            case State::ReadyToBoot:
            {
                break;
            }
        }


        node_status.health = node_health;
        node_status.mode = node_mode;
        node_status.sub_mode = 0;
        node_status.vendor_specific_status_code = 0;
        //let us assign this to the data object
        node_info.status = node_status;
        /*
          Let us construct a node information object
        */
        uavcan_protocol_SoftwareVersion software_version;
        software_version.major = 1;
        software_version.minor = 0;
        software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT;
        software_version.vcs_commit = FIRMWARE_VERSION; //The actual git hash. Generated by VF
        software_version.image_crc = 0; //currently do not care about this

        node_info.software_version = software_version;

        uavcan_protocol_HardwareVersion hardware_version;
        hardware_version.major = 1;
        hardware_version.minor = 0;
        hardware_version.certificate_of_authenticity.len = 0;
        //get and copy the unique id into the data array
        UniqueID unique_id = readUniqueID();
        for(uint8_t i = 0; i < unique_id.size(); i++)
        {
          hardware_version.unique_id[i] = unique_id[i];
        }

        node_info.hardware_version = hardware_version;

        //copy node name to our object
        std::copy(node_name_.begin(), node_name_.end(), std::begin(node_info.name.data));
        node_info.name.len = node_name_.length();

        uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&node_info, node_info_buffer);

        (void) canardRequestOrRespond(&canard_,
                transfer->source_node_id,
                UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                UAVCAN_PROTOCOL_GETNODEINFO_ID,
                &transfer->transfer_id,
                transfer->priority,
                CanardResponse,
                &node_info_buffer,
                len
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
		);
        can_tx_count++;
    }

    /*
     * RestartNode request.
     */
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
            (transfer->data_type_id == dsdl::RestartNode::DataTypeID))
    {
        std::uint8_t response = 0; // 1 - ok, 0 - rejected

        std::uint64_t magic_number = 0;
        (void) canardDecodeScalar(transfer, 0, 40, false, &magic_number);

        if (magic_number == 0xACCE551B1E)
        {
            response = 1 << 7;
            //TODO: Actually hand this off to somewhere else. We do not get to the next part where we actually need to reboot.
            bootloader_info_location = RAM_DO_BOOTLOADER_BYTE;
            signal_reboot = true;//NVIC_SystemReset();
        }

        // No need to release the transfer payload, it's single frame anyway
        (void) canardRequestOrRespond(&canard_,
                transfer->source_node_id,
                dsdl::RestartNode::DataTypeSignature,
                dsdl::RestartNode::DataTypeID,
                &transfer->transfer_id,
                transfer->priority,
                CanardResponse,
                &response,
                1
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
		);
        can_tx_count++;
    }
    /*
     * Transport Stats Request
     */
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
            (transfer->data_type_id == UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID))
    {
      //This actually just sends an empty object. The fact that we are here is enough to solicit a response.
      uavcan_protocol_GetTransportStatsResponse response;
      response.transfers_tx = can_tx_count;
      response.transfers_rx = get_can_rx_count();
      response.can_iface_stats.len = 0;

      uint8_t transport_status_buffer[30];

      uint32_t len = uavcan_protocol_GetTransportStatsResponse_encode(&response,transport_status_buffer);

      canardReleaseRxTransferPayload(&canard_, transfer);

        (void) canardRequestOrRespond(&canard_,
                transfer->source_node_id,
                UAVCAN_PROTOCOL_GETTRANSPORTSTATS_SIGNATURE,
                UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID,
                &transfer->transfer_id,
                transfer->priority,
                CanardResponse,
                &transport_status_buffer,
                len
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
		);
        can_tx_count++;
    }

#ifdef LIBCANARD_MESSAGE_FIRMWARE_UPGRADE
    /*
     * BeginFirmwareUpdate request.
     */
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
            (transfer->data_type_id == dsdl::BeginFirmwareUpdate::DataTypeID))
    {
        std::uint8_t error = 0;

        if ((can_module_state == State::AppUpgradeInProgress) || (remote_server_node_id_ != 0))
        {
            error = 2; // Already in progress
        }
        else if ((can_module_state == State::ReadyToBoot))
        {
            error = 1; // Invalid mode
        }
        else
        {
            // Determine the node ID of the firmware server
            (void) canardDecodeScalar(transfer, 0, 8, false, &remote_server_node_id_);
            if ((remote_server_node_id_ == 0) ||
                    (remote_server_node_id_ >= CANARD_MAX_NODE_ID))
            {
                remote_server_node_id_ = transfer->source_node_id;
            }

            // Copy the path
            firmware_file_path_.clear();
            for (unsigned i = 0;
                    i < (transfer->payload_len - 1U);
                    i++)
            {
                char val = '\0';
                (void) canardDecodeScalar(transfer, i * 8 + 8, 8, false, &val);
                firmware_file_path_.push_back(val);
            }
            //tell the driver to begin requesting file chunks
            download_state = DownloadState::DownloadStateDownloading;
            //tell the driver it is OK to ask for a chunk
            //            request_next_chunk = true;
            read_result_ = std::numeric_limits<int>::max();

            can_module_state = State::AppUpgradeInProgress;
            //erase the first page so we don't think there is anything to boot
            error = 0;
            retry_count = 0;
            transfer_started = true;
            sendNodeStatus();
        }

        canardReleaseRxTransferPayload(&canard_, transfer);
        canardRequestOrRespond(&canard_,
                transfer->source_node_id,
                dsdl::BeginFirmwareUpdate::DataTypeSignature,
                dsdl::BeginFirmwareUpdate::DataTypeID,
                &transfer->transfer_id,
                transfer->priority,
                CanardResponse,
                &error,
                1
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
		);
        can_tx_count++;
    }

    /*
     * File read response.
     */
    if ((transfer->transfer_type == CanardTransferTypeResponse) &&
            (transfer->data_type_id == dsdl::FileRead::DataTypeID) &&
            (((transfer->transfer_id + 1) & 31) == file_read_transfer_id_))
    {
        std::uint16_t error = 0;
        (void) canardDecodeScalar(transfer, 0, 16, false, &error);
        if (error != 0)
        {
            read_result_ = -error;
        }
        else
        {
            read_result_ = std::min(256, transfer->payload_len - 2);
            for (int i = 0; i < read_result_; i++)
            {
                (void) canardDecodeScalar(transfer, 16 + i * 8, 8, false, &read_buffer_[i]);
            }
            //tell the driver to ask for another chunk
            retry_count = 0;
            response_recieved = true;
            request_next_chunk = true;
        }
    }
#endif
#ifdef LIBCANARD_MESSAGE_PARAMETERS
  /**
    Handles all the messages regarding parameters. Components might not want this, especially the bootloader_
  */
  if ((transfer->transfer_type == CanardTransferTypeRequest) &&
      (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID))
      {
        //to decode a string variable, we need somewhere to put it.

        uint8_t buffer[50]; //50 characters!!? This can only go wrong.
        uint8_t * buff_ptr;
        //let us move the ptr to the buffers
        buff_ptr = buffer;

        //create a parameter request
        uavcan_protocol_param_GetSetRequest id_request;
        uavcan_protocol_param_GetSetRequest_decode(transfer, &id_request);
        int16_t can_param_index = -1;
        //we have no value here, therefore we are wanting some information
        if(id_request.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY)
        {
          //if we have no name len, then we are being asked to list the parameter
          //we are being asked to update a parameter
          //Here we check to see if parameter exists by name.


          //we will only ever use the index if no name is supplied.
          if(id_request.name.len == 0)
          {
            if(can_param_exists(id_request.index))
              {
                can_param_index = (int16_t)id_request.index;
              }
          }
          else
          {
            can_param_index = can_param_exists_by_name(id_request.name.data, id_request.name.len);
          }
        }
        else
        {
            can_param_index = can_param_exists_by_name(id_request.name.data, id_request.name.len);
            if(can_param_index >= 0)
            {
              can_parameters[can_param_index].value = id_request.value;

              if(id_request.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE)
                can_parameters[can_param_index].value.integer_value = id_request.value.integer_value;
              if(id_request.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE)
                can_parameters[can_param_index].value.real_value = id_request.value.real_value;
              if(id_request.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE)
                can_parameters[can_param_index].value.boolean_value = id_request.value.boolean_value;
            }
            sendLog(impl_::LogLevel::Info, "Updating ID");
        }

          //This is an empty packet. If we can't find a parameter, we will respond with a null object.
          uavcan_protocol_param_GetSetResponse resp;
          resp.name.len = 0;
          //wohoo, we have a parameter
          if (can_param_index >= 0)
          {
            resp = can_parameters[can_param_index];
          }

          uint8_t get_set_buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
          //encode the value
          uint32_t get_set_buffer_length = uavcan_protocol_param_GetSetResponse_encode(&resp, get_set_buffer);


          //we have been asked for parameters, let us send some back
          canardRequestOrRespond(&canard_,
                  transfer->source_node_id,
                  UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                  UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                  &transfer->transfer_id,
                  transfer->priority,
                  CanardResponse,
                  get_set_buffer,
                  get_set_buffer_length
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
		);
          can_tx_count++;

      }

      if ((transfer->transfer_type == CanardTransferTypeRequest) &&
          (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID))
          {
            uavcan_protocol_param_ExecuteOpcodeRequest op_code_request;
            uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &op_code_request);

            //we have been asked to do something
            bool succeeded = false;
            if(op_code_request.opcode  == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE)
            {
              //store the data.
              succeeded = save_can_parameters(false);
              sendLog(impl_::LogLevel::Info, "Saving Parameters");
            }
            if(op_code_request.opcode  == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE)
            {
              //store the data, but break the versioning. Then go down for a reboot.
              succeeded = save_can_parameters(true);
              signal_reboot = true;
              sendLog(impl_::LogLevel::Info, "Erasing Parameters");
            }

            uavcan_protocol_param_ExecuteOpcodeResponse op_code_response;
            op_code_response.ok = succeeded;
            op_code_response.argument = 0;
            uint8_t op_code_buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
            uint32_t length = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&op_code_response, op_code_buffer);


            //we have executed the opcode, let us send the response back
            canardRequestOrRespond(&canard_,
                    transfer->source_node_id,
                    UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                    UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                    &transfer->transfer_id,
                    transfer->priority,
                    CanardResponse,
                    op_code_buffer,
                    length
#ifdef CANARD_MULTI_IFACE
        , 0xFF
#endif
		);
            can_tx_count++;
          }
#endif



    /**
      As we have received an RX. Let us increment the counter.
    */
    increment_can_rx_count();
    /**
      Let us ask the driver host to spread the transfer around. This means that each module can subscribe to what ever transfer that they require.
      data signature is not used when internally transferring
    */

    driverhost_broadcast_can(transfer, 0, transfer->data_type_id, &transfer->transfer_id, transfer->priority, transfer->payload_head, transfer->payload_len, this);
    /**
      Let us clear the transfer for Justin Case. This might already be released, in this case we have re-released it. Noice.
    */
    canardReleaseRxTransferPayload(&canard_, transfer);
}

bool Libcanard_module::shouldAcceptTransfer(std::uint64_t* out_data_type_signature,
        std::uint16_t data_type_id,
        CanardTransferType transfer_type,
        std::uint8_t source_node_id)
{
    using namespace impl_::dsdl;

    (void) source_node_id;

    if (canardGetLocalNodeID(&canard_) == CANARD_BROADCAST_NODE_ID)
    {
        // Dynamic node ID allocation broadcast
        if ((transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == NodeIDAllocation::DataTypeID))
        {
            *out_data_type_signature = NodeIDAllocation::DataTypeSignature;
            return true;
        }
    }
    else
    {
        // GetNodeInfo REQUEST
        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == GetNodeInfo::DataTypeID))
        {
            *out_data_type_signature = GetNodeInfo::DataTypeSignature;
            return true;
        }

        // BeginFirmwareUpdate REQUEST
        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == BeginFirmwareUpdate::DataTypeID))
        {
            *out_data_type_signature = BeginFirmwareUpdate::DataTypeSignature;
            return true;
        }

        // FileRead RESPONSE (we don't serve requests of this type)
        if ((transfer_type == CanardTransferTypeResponse) &&
                (data_type_id == FileRead::DataTypeID))
        {
            *out_data_type_signature = FileRead::DataTypeSignature;
            return true;
        }

        // RestartNode REQUEST
        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == RestartNode::DataTypeID))
        {
            *out_data_type_signature = RestartNode::DataTypeSignature;
            return true;
        }
#ifdef LIBCANARD_MESSAGE_PARAMETERS
        //parameter get/set
        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }

        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
#endif

        if ((transfer_type == CanardTransferTypeRequest) &&
                (data_type_id == UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETTRANSPORTSTATS_SIGNATURE;
            return true;
        }
#ifdef LIBCANARD_MESSAGE_ARRAYCOMMAND
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID))
        {
            *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
            return true;
        }
#endif
#ifdef LIBCANARD_MESSAGE_LIGHTSCOMMAND
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID))
        {
            *out_data_type_signature = UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_SIGNATURE;
            return true;
        }
#endif
#ifdef LIBCANARD_MESSAGE_NOTIFYSTATE
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == ARDUPILOT_INDICATION_NOTIFYSTATE_ID))
        {
            *out_data_type_signature = ARDUPILOT_INDICATION_NOTIFYSTATE_SIGNATURE;
            return true;
        }
#endif


#ifdef LIBCANARD_MESSAGE_ARMINGSTATUS
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID))
        {
            *out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
            return true;
        }
#endif

#ifdef LIBCANARD_MESSAGE_LOADCELLINFO
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == COM_AERONAVICS_LOADCELLINFO_ID))
        {
            *out_data_type_signature = COM_AERONAVICS_LOADCELLINFO_SIGNATURE;
            return true;
        }
#endif

#ifdef LIBCANARD_MESSAGE_SPRAYCTRL
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == COM_AERONAVICS_SPRAYCTRL_ID))
        {
            *out_data_type_signature = COM_AERONAVICS_SPRAYCTRL_SIGNATURE;
            return true;
        }
#endif

#ifdef LIBCANARD_MESSAGE_EXTENDERINFO
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == COM_AERONAVICS_EXTENDERINFO_ID))
        {
            *out_data_type_signature = COM_AERONAVICS_EXTENDERINFO_SIGNATURE;
            return true;
        }
#endif

#ifdef LIBCANARD_MESSAGE_EXTENDERCTRL
        if ((transfer_type == CanardTransferTypeRequest || transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == COM_AERONAVICS_EXTENDERCTRL_ID))
        {
            *out_data_type_signature = COM_AERONAVICS_EXTENDERCTRL_SIGNATURE;
            return true;
        }
#endif
    }

    return false;
}

void Libcanard_module::onTransferReceptionTrampoline(CanardInstance* ins,
        CanardRxTransfer * transfer)
{
    assert((ins != nullptr) && (ins->user_reference != nullptr));
    Libcanard_module * const self = reinterpret_cast<Libcanard_module*> (ins->user_reference);
    self->onTransferReception(transfer);
}

bool Libcanard_module::shouldAcceptTransferTrampoline(const CanardInstance* ins,
        std::uint64_t* out_data_type_signature,
        std::uint16_t data_type_id,
        CanardTransferType transfer_type,
        std::uint8_t source_node_id)
{
    assert((ins != nullptr) && (ins->user_reference != nullptr));
    Libcanard_module * const self = reinterpret_cast<Libcanard_module*> (ins->user_reference);
    return self->shouldAcceptTransfer(out_data_type_signature,
            data_type_id,
            transfer_type,
            source_node_id);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	HAL_GPIO_TogglePin(STATUS3_GPIO_Port, STATUS3_Pin);
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        /* Retrieve Rx messages from RX FIFO0 */
        FDCAN_RxHeaderTypeDef RxHeader;
        CanardCANFrame can_frame;
        if (hfdcan->Instance == FDCAN1)
        {
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, can_frame.data) != HAL_OK)
			{
				Error_Handler();
			}
			can_frame.id = RxHeader.Identifier | CANARD_CAN_FRAME_EFF;
			can_frame.data_len = RxHeader.DataLength;
			can_frame.iface_id = 0;
			Libcanard_module::get_driver().add_to_rx_queue(can_frame);
    	}
		if (hfdcan->Instance == FDCAN2)
		{
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, can_frame.data) != HAL_OK)
			{
				Error_Handler();
			}
			can_frame.id = RxHeader.Identifier | CANARD_CAN_FRAME_EFF;
			can_frame.data_len = RxHeader.DataLength;
			can_frame.iface_id = 0;
			Libcanard_module::get_driver().add_to_rx_queue(can_frame);
		}
    }
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        /* Retrieve Rx messages from RX FIFO1 */
        FDCAN_RxHeaderTypeDef RxHeader;
        CanardCANFrame can_frame;
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, can_frame.data) != HAL_OK)
        {
            Error_Handler();
        }

        can_frame.id = RxHeader.Identifier;
        can_frame.data_len = RxHeader.DataLength;
        can_frame.iface_id = 0;

        Libcanard_module::get_driver().add_to_rx_queue(can_frame);
    }
}

#endif


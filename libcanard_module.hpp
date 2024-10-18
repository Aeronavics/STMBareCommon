/********************************************************************************************************************************
 *
 *	@copyright Copyright (C) 2019 Aeronavics Ltd
 *
 *  FILE:           libcanard_module.hpp
 *
 *  LIBRARY:        Libcanard
 *
 *  AUTHOR:         Anton Slooten
 *
 *  DATE CREATED:   07-10-2019
 *
 *	This is the header file which matches libcanard_module.cpp.
 *
 *  Provides a runnable module for using Libcanard for the UAVCAN protocol.
 *  This creates/receives messages and provies everything fully contained regarding CAN.
 *  Calling applications must first enable the modules that are required for this.
 *  The end user should also include the gen_libuavcan code generator.
 *
 ********************************************************************************************************************************/

// Only include this header file once.
#pragma once

// INCLUDE REQUIRED HEADER FILES FOR INTERFACE.

// Include the STDINT fixed width types.
#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <cstdint>
#include <time.h>
#include <string.h>
#include <string>
#include <assert.h>
#include <errno.h>
#include <array>
#include <utility>
#include <cstddef>
#include <unistd.h>
#include <limits>
//Include libcanard
#include <canard.h>
#include <canard_stm32.h>
//#define NO_MAVLINK_ENABLED 1
#include "driver_module.hpp"
//include our chip definition

#include "config.h"
#include STM_HAL
#include STM_CAN

#include "tim.h"

#define STM32_PCLK1		(36000000U)		// 36 MHz
#define STM32_TIMCLK1	(2*STM32_PCLK1) //72MHz

/*
  Here is a #include based definition for known data sets. The end application MUST add this to their
*/
#ifndef LIBCANARD_MESSAGE_NODE
  //There is never an application we do not want the node to be a thing, therefore this is added by default.
  //Users are encouraged to also add this to their definitions, as it is generally good practice
  #define LIBCANARD_MESSAGE_NODE
#endif

#ifdef LIBCANARD_MESSAGE_NODE
  #include <uavcan.protocol.NodeStatus.h>
  #include <uavcan.protocol.GetTransportStats.h>
  #include <uavcan.protocol.CANIfaceStats.h>
  #include <uavcan.protocol.GetNodeInfo.h>
#endif
#ifdef LIBCANARD_MESSAGE_FIRMWARE_UPGRADE
  #include "common.h"
  #include "flash_if.h"
  //Typically used by the bootloader, as a way of reading files from the CANBUS network.
  #include <uavcan.protocol.file.BeginFirmwareUpdate.h>
  #include <uavcan.protocol.file.Read.h>
#endif
#ifdef LIBCANARD_MESSAGE_PARAMETERS
  //Typically used by the bootloader, as a way of reading files from the CANBUS network.
  #include <uavcan.protocol.param.GetSet.h>
  #include <uavcan.protocol.param.NumericValue.h>
  #include <uavcan.protocol.param.Value.h>
  #include <uavcan.protocol.param.ExecuteOpcode.h>
  #include <cstring>
#endif

#ifdef LIBCANARD_MESSAGE_ARRAYCOMMAND
  #include <uavcan.equipment.actuator.ArrayCommand.h>
#endif
#ifdef LIBCANARD_MESSAGE_LIGHTSCOMMAND
  #include <uavcan.equipment.indication.LightsCommand.h>
#endif
#ifdef LIBCANARD_MESSAGE_NOTIFYSTATE
  #include <ardupilot.indication.NotifyState.h>
#endif
#ifdef LIBCANARD_MESSAGE_ARMINGSTATUS
  #include <uavcan.equipment.safety.ArmingStatus.h>
#endif


#define FIRMWARE_VERSION <<<TC_INSERTS_COMMIT_HASH_HERE>>>

#define APP_VERSION_MAJOR                                           1
#define APP_VERSION_MINOR                                           0



/*
  The bootloader info is provided as a static place in memory by the linker script
  extern is sufficient for the compiler to assign this as compile time.
*/
extern uint8_t bootloader_info_location;


/*
 * Some useful constants defined by the UAVCAN specification.
 * Data type signature values can be easily obtained with the script show_data_type_info.py
 */

 /**
 Todo: Fix this and get it from the dsdl
 */
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID                      1
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE               0x0b2a812620a11d40
#define UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC         400UL
#define UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC         600UL

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      ((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE                    0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID                           1


#define UNIQUE_ID_LENGTH_BYTES 16 //standard size for STM32
#ifndef MEMORY_POOL_SIZE
  #define MEMORY_POOL_SIZE 2000 //TODO: Make this an included number so it can be overridden
#endif
#define BOOT_DELAY 5 //seconds //TODO: Move this to bootloader.


/*
  Location of the CAN Parameters in flash memory.
*/
#define CAN_EEPROM_LOCATION ((uint32_t)0x0800B000) //ADDR_FLASH_PAGE_22
#define CAN_NODE_LOCATION_OFFSET 0x10 //This is so we do not have to perform auto can node allocation each time.
#define FLASH_START_ADDRESS ((uint32_t)0x0800C800) //ADDR_FLASH_PAGE_25
#define RAM_DO_BOOTLOADER_BYTE 0x22
#define CAN_EEPROM_FLAG 0xBB

/**
  Non Libcanard implementation of usleep  as it does not provide one by default.
*/
extern "C"  int usleep(useconds_t useconds);

typedef void (*pFunction)(void);

#ifdef LIBCANARD_MESSAGE_PARAMETERS
uavcan_protocol_param_GetSetResponse new_boolean_can_param(const char * name, bool value, bool default_value);
uavcan_protocol_param_GetSetResponse new_can_param(const char * name, bool value, bool default_value);
uavcan_protocol_param_GetSetResponse new_integer_can_param(const char * name, int64_t value, int64_t default_value, int64_t min_value = 0, int64_t max_value = 0);
uavcan_protocol_param_GetSetResponse new_can_param(const char * name, int64_t value, int64_t default_value, int64_t min_value = 0, int64_t max_value = 0);
uavcan_protocol_param_GetSetResponse new_real_can_param(const char * name, float value, float default_value, float min_value = 0, float max_value = 0);
uavcan_protocol_param_GetSetResponse new_can_param(const char * name, float value, float default_value, float min_value = 0, float max_value = 0);
uavcan_protocol_param_GetSetResponse * get_can_param_by_id(uint16_t id);

union Flash_status_t {
    uint64_t uint64_block;
    uint32_t uint32_block[2];
    uint16_t uint16_block[4];
    uint8_t uint8_block[8];
};
#endif

typedef enum Init_state_t {
  INIT_STATE_UNINITIALIZED,
  INIT_STATE_BITRATE_AUTODETECT,
  INIT_STATE_POST_SETUP
};

namespace impl_ {
    /**
     * This timeout should accommodate all operations with the application image storage
     * (which is typically based on flash memory, which is slow).
     * Image verification can take several seconds, especially if the image is invalid.
     */
    static constexpr unsigned WatchdogTimeoutMillisecond = 5000;

    static constexpr unsigned ServiceRequestTimeoutMillisecond = 1000;

    static constexpr unsigned ProgressReportIntervalMillisecond = 10000;

    namespace dsdl {

        static inline constexpr std::size_t bitlen2bytelen(std::size_t x) {
            return (x + 7) / 8;
        }

        template <std::uint32_t DataTypeID_,
        std::uint64_t DataTypeSignature_, // Not to be confused with DSDL signature
        std::size_t MaxEncodedBitLength_>
        struct MessageTypeInfo {
            static constexpr std::uint32_t DataTypeID = DataTypeID_;
            static constexpr std::uint64_t DataTypeSignature = DataTypeSignature_;

            static constexpr std::size_t MaxEncodedBitLength = MaxEncodedBitLength_;
            static constexpr std::size_t MaxSizeBytes = bitlen2bytelen(MaxEncodedBitLength_);
        };

        template <std::uint32_t DataTypeID_,
        std::uint64_t DataTypeSignature_, // Not to be confused with DSDL signature
        std::size_t MaxEncodedBitLengthRequest_,
        std::size_t MaxEncodedBitLengthResponse_>
        struct ServiceTypeInfo {
            static constexpr std::uint32_t DataTypeID = DataTypeID_;
            static constexpr std::uint64_t DataTypeSignature = DataTypeSignature_;

            static constexpr std::size_t MaxEncodedBitLengthRequest = MaxEncodedBitLengthRequest_;
            static constexpr std::size_t MaxSizeBytesRequest = bitlen2bytelen(MaxEncodedBitLengthRequest_);

            static constexpr std::size_t MaxEncodedBitLengthResponse = MaxEncodedBitLengthResponse_;
            static constexpr std::size_t MaxSizeBytesResponse = bitlen2bytelen(MaxEncodedBitLengthResponse_);
        };

        // The values have been obtained with the help of the script show_data_type_info.py from libcanard.
        using NodeStatus = MessageTypeInfo<341, 0x0f0868d0c1a7c6f1, 56>;
        using NodeIDAllocation = MessageTypeInfo<1, 0x0b2a812620a11d40, 141>;
        using LogMessage = MessageTypeInfo<16383, 0xd654a48e0c049d75, 983>;

        using GetNodeInfo = ServiceTypeInfo<1, 0xee468a8121c46a9e, 0, 3015>;
        using BeginFirmwareUpdate = ServiceTypeInfo<40, 0xb7d725df72724126, 1616, 1031>;
        using FileRead = ServiceTypeInfo<48, 0x8dcdca939f33f678, 1648, 2073>;
        using RestartNode = ServiceTypeInfo<5, 0x569e05394a3017f0, 40, 1>;

        enum class NodeHealth : std::uint8_t {
            Ok = 0,
            Warning = 1,
            Error = 2
        };

        enum class NodeMode : std::uint8_t {
            NormalOperation = 0,
            Initialization = 1,
            Maintenance = 2,
            SoftwareUpdate = 3
        };

    }

    enum class LogLevel : std::uint8_t {
        Debug,
        Info,
        Warning,
        Error
    };
}
typedef std::array<std::uint8_t, 16> UniqueID;

struct HardwareInfo {
    std::uint8_t major = 0; ///< Required field
    std::uint8_t minor = 0; ///< Required field
    UniqueID unique_id{}; ///< Required field

    typedef std::array<std::uint8_t, 255> CertificateOfAuthenticity;
    CertificateOfAuthenticity certificate_of_authenticity; ///< Optional, set length to zero if not defined
    std::uint8_t certificate_of_authenticity_length = 0;
};

enum class State {
    NoAppToBoot,
    AppUpgradeInProgress,
    BootCancelled,
    BootDelay,
    ReadyToBoot,
    NormalOperation,
    Error,
    Initialization,
};

enum class Mode {
    Normal,
    Silent,
    AutomaticTxAbortOnError
};

enum class DownloadState {
    DownloadStateIdle,
    DownloadStateDownloading,
    DownloadStateEnd
};

/**
 * Acceptance filter configuration.
 * Acceptance filters may be not supported in the underlying driver, this feature is optional.
 * Bit flags used here are the same as in libcanard.
 * The default constructor makes a filter that accepts all frames.
 */
struct AcceptanceFilterConfig {
    std::uint32_t id = 0;
    std::uint32_t mask = 0;
};



class Libcanard_module : public Driver_module {
public:

    // Fields.

    // Methods.

    // Inherited from Driver_module.
    void sync_update_unthrottled();
    void sync_update_100Hz();
    void sync_update_10Hz();
    void sync_update_1Hz();
    void handle_rx_can(const CanardRxTransfer * transfer, uint64_t data_type_signature, uint16_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len);
    static Libcanard_module& get_driver(void);
    uint32_t usec_since_boot;
    volatile bool kill_loading;
    void set_name(const char * name);
    #ifdef LIBCANARD_MESSAGE_PARAMETERS
      /*
      Allow external modules to request storing and erasing of parameters.
      At pressent, there is no instance where this will reject the saving, but there hypotheically could be in the future.
      */
      void request_save_parameters(void);
      void request_erase_parameters(void);
    #endif

    void sendLog(const impl_::LogLevel level, const std::string& txt);

    ~Libcanard_module(void);
private:
    // Fields.
    Driver_state prev_state;
    Init_state_t init_state;
    //we need a global to track the state of bit rate detection
    int current_bit_rate_index = 0;
    bool signal_reboot;
    bool force_reboot;
    std::uint64_t can_tx_count;
    std::uint32_t unused_var_can_rx_count;

    DownloadState download_state;
    bool request_next_chunk;
    std::uint64_t offset;
    std::string node_name_; //Yes. the name can be massive. twss
    State can_module_state;
    //const NodeName node_name_;
    HardwareInfo hw_info_{};
    std::uint64_t next_1hz_task_invocation_;
    std::uint64_t response_deadline;
    std::uint64_t next_progress_report_deadline;
    std::uint64_t timeout_deadline;
    uint8_t retry_count;
    //    CanardInstance canard; ///< The library instance
    //uint8_t canard_memory_pool[1024]; ///< Arena for memory allocation, used by the library

    uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
    uint8_t node_mode = UAVCAN_NODE_MODE_INITIALIZATION;
    bool had_activity_;
    bool init_done_;
    //    int MemoryPoolSize = 8192;

    alignas(std::max_align_t) std::array<std::uint8_t, MEMORY_POOL_SIZE> memory_pool_ {
    };
    uint8_t boot_delay_count;
    uint32_t last_led_update_timestamp_st_;
    CanardInstance canard_{};

    std::uint32_t can_bus_bit_rate_ = 0;
    std::uint8_t confirmed_local_node_id_ = 0; ///< This field is needed in order to avoid mutexes

    std::uint8_t remote_server_node_id_ = 0;
    std::string firmware_file_path_;

    std::uint64_t send_next_node_id_allocation_request_at_ = 0;
    std::uint8_t node_id_allocation_unique_id_offset_ = 0;

    std::uint16_t vendor_specific_status_ = 0;

    std::uint8_t node_status_transfer_id_ = 0;
    std::uint8_t node_id_allocation_transfer_id_ = 0;
    std::uint8_t log_message_transfer_id_ = 0;
    std::uint8_t file_read_transfer_id_ = 0;

    std::array<std::uint8_t, 256> read_buffer_{};
    bool transfer_started;
    bool response_recieved;
    std::uint64_t wait_deadline;
    uint8_t tmp_array[256];
    int read_result_ = 0;
    // Methods.
    #ifdef LIBCANARD_DISABLE_CAN
      bool can_enabled = false;
    #else
      bool can_enabled = true;
    #endif

    //increments if a message is received.
    uint64_t get_can_rx_count(void) const {return unused_var_can_rx_count;};
    inline void increment_can_rx_count(void) {unused_var_can_rx_count++;};


    /**
     * Initializes the CAN hardware in the specified mode.
     * Observe that this implementation needs only one acceptance filter.
     * @retval 0                Success
     * @retval negative         Error
     */
    int init(const std::uint32_t bitrate, const Mode mode, const AcceptanceFilterConfig& acceptance_filter);

    int initCAN(const std::uint32_t bitrate, const Mode mode, const AcceptanceFilterConfig& acceptance_filter = AcceptanceFilterConfig());

    /**
     * Transmits one CAN frame.
     * Timeout value can be only positive.
     *
     * @retval      1               Transmitted successfully
     * @retval      0               Timed out
     * @retval      negative        Error
     */
    int send(const CanardCANFrame& frame, const int timeout_millisec);

    /**
     * Reads one CAN frame from the RX queue.
     * Timeout value can be only positive.
     *
     * @retval      1               Read successfully
     * @retval      0               Timed out
     * @retval      negative        Error
     */
    std::pair<int, CanardCANFrame> receive(const int timeout_millisec);


    bool init_can_device(void);
    float getRandomFloat(void);
    uint64_t getMonotonicTimestampUSec(void) const;
    uint64_t get_system_time_ms(void);
    void readUniqueID(uint8_t* out_uid);
    void performCANBitRateDetection();
    void makeNodeStatusMessage(std::uint8_t* buffer) const;
    //    int download(IDownloadStreamSink& sink);
    void onTransferReception(CanardRxTransfer * const transfer);
    bool shouldAcceptTransfer(std::uint64_t* out_data_type_signature, std::uint16_t data_type_id, CanardTransferType transfer_type, std::uint8_t source_node_id);
    static void onTransferReceptionTrampoline(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransferTrampoline(const CanardInstance* ins, std::uint64_t* out_data_type_signature, std::uint16_t data_type_id, CanardTransferType transfer_type, std::uint8_t source_node_id);
    void performDynamicNodeIDAllocation();
    void poll();
    void handle1HzTasks();

#ifdef LIBCANARD_MESSAGE_NODE

    void sendNodeStatus();

#endif
#ifdef LIBCANARD_MESSAGE_FIRMWARE_UPGRADE

    bool is_boot_image_available(void);
    bool boot(void);

#endif
#ifdef LIBCANARD_MESSAGE_PARAMETERS

    bool can_param_exists(uint16_t index);
    int16_t can_param_exists_by_name(uint8_t * name, uint8_t len);
    friend uavcan_protocol_param_GetSetResponse * get_can_param_by_id(uint16_t id);
    uavcan_protocol_param_GetSetResponse * get_can_param_by_name(uint8_t * name);
    /**
      Saves the Libuavcan parameters to a permanent memory.

      @param overwrite_with_defaults when true, breaks the saving version number and reboots the device to restore defaults
      @returns true if the operation has succeeded, false if not.
    */
    bool save_can_parameters(bool overwrite_with_defaults);
    /**
      loads parameters from memory. If the version number is not correct, then the default parameters are used (and stored!)
      @returns true if the operation has succeeded, false if not.
    */
    bool load_can_parameters();

    /**
      Erases a specified page in memory.
      @param address memory address that is to be erased.
      @returns Nothing
    */
    void erase_eeprom_page(uint32_t address);
    /**
      Stores a double word at a specified place in memory.

      @param address Memory address to store data.
      @param data The actual data to be written.
      @returns Nothing.
    */
    void store_in_flash(uint32_t address, uint64_t data);

#endif
    /**
      Returns the unique id of the STM32 micro-controller.
    */
    UniqueID readUniqueID();
    /**
      performs NOP's for a defined number of microseconds.
      @param useconds The time in in microseconds to block for.
    */
    friend int usleep(uint32_t useconds);
    /**
      returns a random integer between two values. Can be used as source for random wait times.

      @param lower_bound_usec The lowest minimum wait time
      @param upper_bount_usec The highest maximum wait time
      @return a random number in the range of lower and upper _bound_sec in microseconds.
    */
    std::uint64_t getRandomDurationMicrosecond(std::uint64_t lower_bound_usec, std::uint64_t upper_bound_usec) const;
    Libcanard_module(void);
    Libcanard_module(Libcanard_module const&); // Poisoned.
    void operator=(Libcanard_module const&); // Poisoned.
};

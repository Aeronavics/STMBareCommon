
#define SMART_PORT_RESPONSE_LENGTH 	8
#define SMART_PORT_POLL_LENGTH 		2


#define  FSSP_START_STOP 			0x7E
#define  FSSP_DATA_FRAME 			0x10

// ID of sensor. Must be something that is polled by FrSky RX
#define   FSSP_SENSOR_ID1 			0x1B
#define   FSSP_SENSOR_ID2 			0x0D
#define   FSSP_SENSOR_ID3 			0x34
#define   FSSP_SENSOR_ID4 			0x67
// there are 32 ID's polled by smartport master
// remaining 3 bits are crc (according to comments in openTx code)


// from: https://github.com/yaapu/FrskyTelemetryScript/wiki/FrSky-SPort-protocol-specs 
#define SENSOR_ID_VARIO             0x00 // Physical ID 0 - Vario2 (altimeter high precision)
#define SENSOR_ID_FLVSS             0xA1 // Physical ID 1 - FLVSS Lipo sensor (can be sent with one or two cell voltages)
#define SENSOR_ID_FAS               0x22 // Physical ID 2 - FAS-40S current sensor
#define SENSOR_ID_GPS               0x83 // Physical ID 3 - GPS / altimeter (normal precision)
#define SENSOR_ID_RPM               0xE4 // Physical ID 4 - RPM
#define SENSOR_ID_SP2UH             0x45 // Physical ID 5 - SP2UART(Host)
#define SENSOR_ID_SP2UR             0xC6 // Physical ID 6 - SPUART(Remote)

// https://github.com/opentx/opentx/blob/2.3/radio/src/telemetry/frsky.h
// https://github.com/lvale/MavLink_FrSkySPort/blob/DisplayAPMPosition/MavLink_FrSkySPort/FrSkySPort.h
// https://docs.px4.io/v1.9.0/en/peripherals/frsky_telemetry.html
// OpenTx DATA IDs from /radio/src/telemetry/frsky.h
// FrSky new DATA IDs (2 bytes)
#define ALT_FIRST_ID              0x0100
#define ALT_LAST_ID               0x010f
#define VARIO_FIRST_ID            0x0110
#define VARIO_LAST_ID             0x011f
#define CURR_FIRST_ID             0x0200
#define CURR_LAST_ID              0x020f
#define VFAS_FIRST_ID             0x0210
#define VFAS_LAST_ID              0x021f
#define CELLS_FIRST_ID            0x0300
#define CELLS_LAST_ID             0x030f
#define T1_FIRST_ID               0x0400
#define T1_LAST_ID                0x040f
#define T2_FIRST_ID               0x0410
#define T2_LAST_ID                0x041f
#define RPM_FIRST_ID              0x0500
#define RPM_LAST_ID               0x050f
#define FUEL_FIRST_ID             0x0600
#define FUEL_LAST_ID              0x060f
#define ACCX_FIRST_ID             0x0700
#define ACCX_LAST_ID              0x070f
#define ACCY_FIRST_ID             0x0710
#define ACCY_LAST_ID              0x071f
#define ACCZ_FIRST_ID             0x0720
#define ACCZ_LAST_ID              0x072f
#define GPS_LONG_LATI_FIRST_ID    0x0800
#define GPS_LONG_LATI_LAST_ID     0x080f
#define GPS_ALT_FIRST_ID          0x0820
#define GPS_ALT_LAST_ID           0x082f
#define GPS_SPEED_FIRST_ID        0x0830
#define GPS_SPEED_LAST_ID         0x083f
#define GPS_COURS_FIRST_ID        0x0840
#define GPS_COURS_LAST_ID         0x084f
#define GPS_TIME_DATE_FIRST_ID    0x0850
#define GPS_TIME_DATE_LAST_ID     0x085f
#define A3_FIRST_ID               0x0900
#define A3_LAST_ID                0x090f
#define A4_FIRST_ID               0x0910
#define A4_LAST_ID                0x091f
#define AIR_SPEED_FIRST_ID        0x0a00
#define AIR_SPEED_LAST_ID         0x0a0f
#define RBOX_BATT1_FIRST_ID       0x0b00
#define RBOX_BATT1_LAST_ID        0x0b0f
#define RBOX_BATT2_FIRST_ID       0x0b10
#define RBOX_BATT2_LAST_ID        0x0b1f
#define RBOX_STATE_FIRST_ID       0x0b20
#define RBOX_STATE_LAST_ID        0x0b2f
#define RBOX_CNSP_FIRST_ID        0x0b30
#define RBOX_CNSP_LAST_ID         0x0b3f
#define SD1_FIRST_ID              0x0b40
#define SD1_LAST_ID               0x0b4f
#define ESC_POWER_FIRST_ID        0x0b50
#define ESC_POWER_LAST_ID         0x0b5f
#define ESC_RPM_CONS_FIRST_ID     0x0b60
#define ESC_RPM_CONS_LAST_ID      0x0b6f
#define ESC_TEMPERATURE_FIRST_ID  0x0b70
#define ESC_TEMPERATURE_LAST_ID   0x0b7f
#define X8R_FIRST_ID              0x0c20
#define X8R_LAST_ID               0x0c2f
#define S6R_FIRST_ID              0x0c30
#define S6R_LAST_ID               0x0c3f
#define GASSUIT_TEMP_FIRST_ID     0x0d00
#define GASSUIT_TEMP_LAST_ID      0x0d0f
#define GASSUIT_SPEED_FIRST_ID    0x0d10
#define GASSUIT_SPEED_LAST_ID     0x0d1f
#define GASSUIT_FUEL_FIRST_ID     0x0d20
#define GASSUIT_FUEL_LAST_ID      0x0d2f
#define DIY_FIRST_ID              0x5000
#define DIY_LAST_ID               0x52ff
#define DIY_STREAM_FIRST_ID       0x5000
#define DIY_STREAM_LAST_ID        0x50ff
#define FACT_TEST_ID              0xf000
#define RSSI_ID                   0xf101
#define ADC1_ID                   0xf102
#define ADC2_ID                   0xf103
#define SP2UART_A_ID              0xfd00
#define SP2UART_B_ID              0xfd01
#define BATT_ID                   0xf104
#define RAS_ID                    0xf105
#define XJT_VERSION_ID            0xf106
#define FUEL_QTY_FIRST_ID         0x0a10
#define FUEL_QTY_LAST_ID          0x0a1f


// Smart Port response packet.
#pragma pack(1)		// //https://fresh2refresh.com/c-programming/c-structure-padding/
typedef struct {
	uint8_t frameId;
	uint16_t valueId;
	uint32_t data;
	uint8_t checkSum;
} smart_port_response_packet;

typedef union {
	smart_port_response_packet spp;
	uint8_t response_raw[8];
}smart_port_response_packet_union;

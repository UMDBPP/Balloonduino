/*
 * Balloonduino.h
 * balloonduino: modified Arduino Mega
 */

#ifndef Balloonduino_h
#define Balloonduino_h

#include "Arduino.h"

// Non optional libraries
#include <avr/wdt.h> // watchdog timer
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <ccsds_xbee.h>
#include <Adafruit_Sensor.h>

// Optional hardware
#include "RTClib.h" // RTC and SoftRTC
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_MCP9808.h"
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1015.h>
#include <SSC.h>

//// Enumerations
/* Requests for HK packets to be sent to specified xbee addresses have the following format:
 *   CCSDS Command Header (8 bytes)
 *   Xbee address (1 byte)
 */
#define MAX_PKT_LEN 200

// logging flag
#define LOG_RCVD 1
#define LOG_SEND 0

/* response codes */
#define INIT_RESPONSE 0xAC
#define READ_FAIL_RESPONSE 0xAF
#define BAD_COMMAND_RESPONSE 0xBB
#define KEEPALIVE_RESPONSE 0xE4

// Built-in function codes
#define COMMAND_NOOP 0  // No action other than to increment the interface counters
#define REQUEST_PACKET_COUNTERS 10
#define COMMAND_CLEAR_PACKET_COUNTERS 15
#define REQUEST_ENVIRONMENTAL_DATA 20
#define REQUEST_POWER_DATA 30
#define REQUEST_IMU_DATA 40
#define COMMAND_REBOOT 99   // Requests reboot

//// Serial object aliases
// so that the user doesn't have to keep track of which is which
#define debug_serial Serial
#define xbee_serial Serial2

// APIDs
int CMD_APID = 100;
int HK_STAT_APID = 110;
int IMU_STAT_APID = 120;
int ENV_STAT_APID = 130;
int PWR_STAT_APID = 140;

// files
File IMULogFile;
File PWRLogFile;
File ENVLogFile;

//// Data Structures
// imu data
struct IMUData_s
{
        uint8_t system_cal;
        uint8_t accel_cal;
        uint8_t gyro_cal;
        uint8_t mag_cal;
        float accel_x;
        float accel_y;
        float accel_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float mag_x;
        float mag_y;
        float mag_z;
};

// environmental data
struct ENVData_s
{
        float bme_pres;
        float bme_temp;
        float bme_humid;
        float ssc_pres;
        float ssc_temp;
        float bno_temp;
        float mcp_temp;
};

// power data
struct PWRData_s
{
        float batt_volt;
        float i_consump;
};

struct IntCtr_s
{
        uint16_t CmdExeCtr;
        uint16_t CmdRejCtr;
        uint16_t XbeeRcvdByteCtr;
        uint16_t XbeeSentByteCtr;
};

bool xbee_enable = true;
bool rtc_enable = true;
bool bno_enable = true;
bool mcp_enable = true;
bool bme_enable = true;
bool ads_enable = true;
bool ssc_enable = true;

class Balloonduino
{
    public:
        Balloonduino::Balloonduino(void);

        bool Balloonduino::begin(uint8_t xbee_address = 0x0006, uint8_t cmd_xbee_address = 0x0002, uint8_t xbee_pan_id = 0x0B0B,
                bool use_xbee = true, int cmd_apid = 100, int hk_stat_apid = 110, int imu_stat_apid = 120, int env_stat_apid = 130, int pwr_stat_apid = 140,
                bool use_rtc = true,
                bool use_bno = true,
                bool use_mcp = true,
                bool use_bme = true,
                bool use_ads = true,
                bool use_ssc = true);

        // sensor reading
        void read_imu();
        void read_env();
        void read_pwr();

        // log data
        void log_imu();
        void log_env();
        void log_pwr();

        // pkt creation
        uint16_t create_HK_pkt();
        uint16_t create_IMU_pkt();
        uint16_t create_PWR_pkt();
        uint16_t create_ENV_pkt();

        void command_response(uint8_t data[], uint8_t data_len, File file);

        void xbee_send_and_log(uint8_t dest_addr, uint8_t data[], uint8_t data_len, File file);
        void logPkt(File file, uint8_t data[], uint8_t len, uint8_t received_flg);

        void print_time(File file);

        // Xbee values
        IntCtr_s IntCtr;
        uint8_t OutPktBuf[MAX_PKT_LEN];
        uint8_t InPktBuf[MAX_PKT_LEN];
        ENVData_s ENVData;

        //// Declare objects
        RTC_DS1307 rtc = RTC_DS1307();
        RTC_Millis softRTC;    // This is the millis()-based software RTC
        uint32_t start_millis = 0;

        Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);
        IMUData_s IMUData;

        Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

        Adafruit_BME280 bme = Adafruit_BME280();

        Adafruit_ADS1015 ads = Adafruit_ADS1015(0x4A);
        PWRData_s PWRData;

        SSC ssc = SSC(0x28, 255);
};

#endif

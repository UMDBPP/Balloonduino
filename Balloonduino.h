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

// Built-in function codes
#define COMMAND_NOOP 0  // No action other than to increment the interface counters
#define REQUEST_PACKET_COUNTERS 10
#define COMMAND_CLEAR_PACKET_COUNTERS 15
#define REQUEST_PAYLOAD_NAME 19
#define REQUEST_ENVIRONMENTAL_DATA 20
#define REQUEST_POWER_DATA 30
#define REQUEST_IMU_DATA 40
#define COMMAND_REBOOT 99   // Requests reboot

/* response codes */
#define INIT_RESPONSE 0xAC
#define READ_FAIL_RESPONSE 0xAF
#define BAD_COMMAND_RESPONSE 0xBB
#define KEEPALIVE_RESPONSE 0xE4

//// Serial object aliases
// so that the user doesn't have to keep track of which is which
#define debug_serial Serial
#define xbee_serial Serial3

/* APIDs */
#define COMMAND_APID 100
#define STATUS_APID 101
#define PACKET_COUNTER_APID 110
#define PAYLOAD_NAME_APID 119
#define ENVIRONMENTAL_PACKET_APID 120
#define POWER_PACKET_APID 130
#define IMU_PACKET_APID 140

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

// power data
struct PWRData_s
{
        float batt_volt;
        float i_consump;
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

// environmental data
struct InitStat_s
{
        uint8_t xbeeStatus;
        uint8_t rtc_running;
        uint8_t rtc_start;
        uint8_t BNO_init;
        uint8_t MCP_init;
        uint8_t BME_init;
        uint8_t SSC_init;
        uint8_t SD_detected;
};

class Balloonduino
{
    public:
        Balloonduino::Balloonduino(void);

        bool Balloonduino::begin(uint8_t xbee_address = 0x0006, uint8_t cmd_xbee_address = 0x0002, uint8_t xbee_pan_id = 0x0B0B,
                bool use_xbee = true, int cmd_apid = 100,
                int hk_stat_apid = 101, int packet_counter_apid = 110,
                int payload_name_apid = 119, int env_stat_apid = 120,
                int imu_stat_apid = 130, int pwr_stat_apid = 140,
                bool use_rtc = true,
                bool use_bno = true,
                bool use_mcp = true,
                bool use_bme = true,
                bool use_ads = true,
                bool use_ssc = true);

        // sensor reading
        void read_imu(struct IMUData_s *IMUData);
        void read_env(struct ENVData_s *ENVData);
        void read_pwr(struct PWRData_s *PWRData);

        // log data
        void log_imu(struct IMUData_s IMUData, File IMULogFile);
        void log_env(struct ENVData_s ENVData, File ENVLogFile);
        void log_pwr(struct PWRData_s PWRData, File PWRLogFile);

        // pkt creation
        uint16_t create_HK_payload(uint8_t Pkt_Buff[]);
        uint16_t create_IMU_payload(uint8_t Pkt_Buff[],
                struct IMUData_s IMUData);
        uint16_t create_PWR_payload(uint8_t Pkt_Buff[],
                struct PWRData_s PWRData);
        uint16_t create_ENV_payload(uint8_t payload[],
                struct ENVData_s ENVData);
        uint16_t create_INIT_payload(uint8_t Pkt_Buff[],
                struct InitStat_s InitStat);

        void command_response(uint8_t data[], uint8_t data_len, File file);

        void xbee_send_and_log(uint8_t dest_addr, uint8_t data[], uint8_t data_len, File file);
        void logPkt(File file, uint8_t data[], uint8_t len, uint8_t received_flg);

        void print_time(File file);

    private:

        bool xbee_enable = true;bool rtc_enable = true;bool bno_enable = true;bool mcp_enable =
                true;bool bme_enable = true;bool ads_enable = true;bool ssc_enable =
                true;

        //// Timing
        // timing counters
        uint16_t imu_read_ctr = 0;
        uint16_t pwr_read_ctr = 0;
        uint16_t env_read_ctr = 0;

        // rate setting
        // sensors will be read every X cycles
        uint16_t imu_read_lim = 10;
        uint16_t pwr_read_lim = 100;
        uint16_t env_read_lim = 100;

        //// Interface counters
        // counters to track what data comes into/out of link
        uint16_t CmdExeCtr;
        uint16_t CmdRejCtr;
        uint32_t XbeeRcvdByteCtr;
        uint32_t XbeeSentByteCtr;

        //// Declare objects
        Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);
        Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
        RTC_DS1307 rtc;  // real time clock (for logging with timestamps)
        RTC_Millis SoftRTC;   // This is the millis()-based software RTC
        Adafruit_BME280 bme;Adafruit_ADS1015 ads(0x4A);SSC ssc(0x28, 255);
        CCSDS_Xbee ccsds_xbee;

        InitStat_s InitStat;

        uint32_t start_millis = 0;

        //// Files
        // interface logging files
        File xbeeLogFile;
        File initLogFile;
        // data logging files
        File IMULogFile;
        File ENVLogFile;
        File PWRLogFile;
};

#endif

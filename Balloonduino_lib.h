/*
 * Balloonduino.h
 * balloonduino: modified Arduino Mega
 */

#ifndef Balloonduino_lib_h
#define Balloonduino_lib_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Non optional
#include <avr/wdt.h> // watchdog timer
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "CCSDS_Xbee/CCSDS.h"
#include "CCSDS_Xbee/ccsds_xbee.h"
#include "CCSDS_Xbee/ccsds_util.h"
#include <SD.h>

// Optional hardware
#ifndef BALLONDUINO_NO_BNO
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#endif

#ifndef BALLONDUINO_NO_MCP
#include "Adafruit_MCP9808.h"
#endif

#ifndef BALLONDUINO_NO_RTC
#include "RTClib.h" // RTC and SoftRTC
#endif

#ifndef BALLONDUINO_NO_BME
#include <Adafruit_BME280.h>
#endif

#ifndef BALLONDUINO_NO_ADS
#include <Adafruit_ADS1015.h>
#endif

#ifndef BALLONDUINO_NO_SSC
#include <SSC.h>
#endif

#define debug_serial Serial
#define xbee_serial Serial3
#define MAX_PKT_LEN 200

// Built-in functioncodes
#define NOOP_FCNCODE 0
#define HKREQ_FCNCODE 10
#define RESETCTR_FCNCODE 15
#define REQENV_FCNCODE 20
#define REQPWR_FCNCODE 30
#define REQIMU_FCNCODE 40
#define REBOOT_FCNCODE 99

//// Xbee setup parameters
#define XBEE_ADDR 0x0006 // XBee address for this payload 
// DO NOT CHANGE without making corresponding change in ground system definitions
#define XBEE_ID 0x0B0B // XBee PAN address (must be the same for all xbees)
// DO NOT CHANGE without changing for all xbees

#ifndef CMD_APID
#define CMD_APID 100
#endif
#ifndef HK_STAT_APID
#define HK_STAT_APID 110
#endif
#ifndef IMU_STAT_APID
#define IMU_STAT_APID 120
#endif
#ifndef ENV_STAT_APID
#define ENV_STAT_APID 130
#endif
#ifndef PWR_STAT_APID
#define PWR_STAT_APID 140
#endif


//// Data Structures
// imu data
#ifndef BALLONDUINO_NO_BNO
struct IMUData_s {
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
#endif
// environmental data
struct ENVData_s {
  float bme_pres;
  float bme_temp;
  float bme_humid;
  float ssc_pres;
  float ssc_temp;
  float bno_temp;
  float mcp_temp;
}; 
// power data
#ifndef BALLONDUINO_NO_ADS
struct PWRData_s {
  float batt_volt;
  float i_consump;
}; 
#endif
struct IntCtr_s {
  uint16_t CmdExeCtr;
  uint16_t CmdRejCtr;
  uint16_t XbeeRcvdByteCtr;
  uint16_t XbeeSentByteCtr;
}; 

class Balloonduino
{
    public:
        Balloonduino();
        void printMetersAndFeet(double meters);
        void printCelsiusAndFahrenheit(double celsius);
        void printPascalsAndAtmospheres(double pascals);
        void printAltitude();
        void printTemperature();
        void printPressure();
        void printFormattedTime();

        void begin();
        
        void command_response(uint8_t data[], uint8_t data_len, File file); 
        // pkt creation
        uint16_t create_HK_pkt();
        uint16_t create_IMU_pkt();
        uint16_t create_PWR_pkt();
        uint16_t create_ENV_pkt();

        // sensor reading
        void read_imu();
        void read_env();
        void read_pwr();

        // log data
        void log_imu(File IMULogFile);
        void log_env(File ENVLogFile);
        void log_pwr(File PWRLogFile);
        void print_time(File file);
        void logPkt(File file, uint8_t data[], uint8_t len, uint8_t received_flg);
        void xbee_send_and_log(uint8_t dest_addr, uint8_t data[], uint8_t data_len, File file);
    private:
        uint32_t milliseconds, delayMilliseconds;
        byte hours, minutes, seconds, launchTolerance;
        double altitude, temperature, pressure;
        bool isLaunched;
        IntCtr_s IntCtr;
        uint8_t OutPktBuf[MAX_PKT_LEN];
        uint8_t InPktBuf[MAX_PKT_LEN];
        ENVData_s ENVData;
        //// Declare objects
        #ifndef BALLONDUINO_NO_BNO
        Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);
        IMUData_s IMUData;
        #endif
        #ifndef BALLONDUINO_NO_MCP
        Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
        #endif
        #ifndef BALLONDUINO_NO_RTC
        RTC_DS1307 rtc = RTC_DS1307();
        RTC_Millis softrtc;   // This is the millis()-based software RTC
        uint32_t start_millis = 0;
        #endif
        #ifndef BALLONDUINO_NO_BME
        Adafruit_BME280 bme = Adafruit_BME280();
        #endif
        #ifndef BALLONDUINO_NO_ADS
        Adafruit_ADS1015 ads = Adafruit_ADS1015(0x4A);
        PWRData_s PWRData;
        #endif
        #ifndef BALLONDUINO_NO_SSC
        SSC ssc = SSC(0x28, 255); 
        #endif
        
};



#endif

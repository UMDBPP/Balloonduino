/*
 * Balloonduino.h
 * balloonduino: modified Arduino Mega
 */

#ifndef Balloonduino_h
#define Balloonduino_h

#include <Arduino.h>

#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <RTClib.h>

#define SENSORS_PRESSURE_SEALEVELHPA (1013.25F)

class Balloonduino
{
    public:
        void begin();
        double getTemperature();
        double getPressure();
        double getAltitude();
        double getHumidity();
        String getMissionTimeString();
        String getRealTimeString();
        String getSensorReadingsString();
        void log(String message);
        void displaySensorStatus();
    private:
        void updateSensorStatus(byte address, byte status);
        byte numberOfSensors = 3;
        byte sensors[3] =
            { 0, 0, 0 };    // array for sensor status (good / bad)
        RTC_DS1307 DS1307;
        Adafruit_BNO055 BNO055;
        Adafruit_BME280 BME280;
};

#endif

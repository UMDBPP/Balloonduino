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

// define pins
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

class Balloonduino
{
    public:
        void displaySensorStatus();
        void begin();
        double getTemperature();
        double getPressure();
        double getAltitude();
        double getHumidity();
        String getTemperatureString();
        String getPressureString();
        String getAltitudeString();
        String getHumidityString();
        String getMissionTimeString();
        String getRealTimeString();
        String getStatusString();
    private:
        void updateSensorStatus(byte address, bool status);
        RTC_DS1307 DS1307;
        Adafruit_BNO055 BNO055;
        Adafruit_BME280 BME280;
};

#endif

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

RTC_DS1307 DS1307;
Adafruit_BNO055 BNO055;
Adafruit_BME280 BME280;
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

class Balloonduino
{
    public:
        void begin();
        double getTemperature();
        double getPressure();
        double getAltitude();
        double getHumidity();
        void printTemperature(double celsius);
        void printPressure(double millibars);
        void printAltitude(double meters);
        void printHumidity(double percentage);
        void printFormattedTime();
        void printMET();
        void printStatusNow();
        void printStatusDuringFlight();
        void printCelsiusAndFahrenheit(double celsius);
        void printMillibarsAndAtmospheres(double millibars);
        void printMetersAndFeet(double meters);
    private:
        double altitude, baselineAltitude, temperature, pressure,
                baselinePressure, humidity;
        unsigned long milliseconds, delayMilliseconds;
        byte hours, minutes, seconds, launchTolerance;
        bool isLaunched = false;
        DateTime startTime;
};

#endif

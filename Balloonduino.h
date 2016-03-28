/*
 * Balloonduino.h
 * balloonduino: modified Arduino Mega
 */

#ifndef Balloonduino_h
#define Balloonduino_h

#include <Arduino.h>

#include <Adafruit_BME280.h>

// define pins
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

Adafruit_BME280 BME280;
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

class Balloonduino
{
    public:
        double getTemperature();
        double getPressure();
        double getAltitude();
        double getHumidity();
        Balloonduino();
        void begin();
        void printTemperature(double celsius);
        void printPressure(double millibars);
        void printAltitude(double meters);
        void printHumidity(double percentage);
        void printFormattedTime();
        void printStatusNow();
        void printStatusDuringFlight(double baselineAltitude);
        void printCelsiusAndFahrenheit(double celsius);
        void printMillibarsAndAtmospheres(double millibars);
        void printMetersAndFeet(double meters);
    private:
        double altitude, temperature, pressure, baselinePressure, humidity;
        unsigned long milliseconds, delayMilliseconds;
        byte hours, minutes, seconds, launchTolerance;
        bool isLaunched = false;
};

#endif

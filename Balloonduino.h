/*
 * Balloonduino.h
 * balloonduino: modified Arduino Mega
 */

#ifndef Balloonduino_h
#define Balloonduino_h

#include <Adafruit_BNO055>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

Adafruit_BNO055 BNO055;

class Balloonduino
{
    public:
        Balloonduino();
        void begin();
        double getAltitude();
        double getTemperature();
        double getPressure();
        void printMetersAndFeet(double meters);
        void printCelsiusAndFahrenheit(double celsius);
        void printPascalsAndAtmospheres(double pascals);
        void printAltitude();
        void printTemperature();
        void printPressure();
        void printFormattedTime();
        void printStatusNow();
        void printStatusDuringFlight();
    private:
        unsigned long milliseconds, delayMilliseconds;
        byte hours, minutes, seconds, launchTolerance;
        double altitude, temperature, pressure;
        bool isLaunched;
};

#endif

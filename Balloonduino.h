/*
 * Balloonduino.h
 * balloonduino: modified Arduino Mega
 */

#ifndef Balloonduino_h
#define Balloonduino_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Balloonduino
{
    public:
        Balloonduino();
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

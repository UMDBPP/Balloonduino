/*
 * Balloonduino.h
 * balloonduino: modified Arduino Mega
 */

#ifndef Balloonduino_h
#define Balloonduino_h

#include <Arduino.h>

class Balloonduino
{
    public:
        Balloonduino();
        double getTemperature();
        double getPressure();
        double getAltitude();
        double getHumidity();
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
        unsigned long milliseconds, delayMilliseconds;
        byte hours, minutes, seconds, launchTolerance;
        double altitude, temperature, pressure, baselinePressure, humidity;
        bool isLaunched = false;
};

#endif

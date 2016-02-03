/*
 * BalloonModuleCommonUtilities.h
 * balloonduino: modified Arduino Mega
 */

#include <SFE_BMP180.h>

SFE_BMP180 pressureSensor;

class BalloonModuleCommonUtilities
{
    public:
        BalloonModuleCommonUtilities();
        double getAltitude();
        double getTemperature();
        double getPressure();
        void printMetersAndFeet(double meters);
        void printCelsiusAndFahrenheit(double celsius);
        void printAltitude();
        void printTemperature();
        void printPressure();
        void printFormattedTime();
        void printStatusNow();
        void printStatusDuringFlight();
    private:
        unsigned long milliseconds = 0, delayMilliseconds = 1000;
        byte hours = 0, minutes = 0, seconds = 0, launchTolerance = 0;
        double altitude = 0, temperature = 0, pressure = 0;
};

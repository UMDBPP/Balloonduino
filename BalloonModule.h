/*
 * BalloonModule.h
 * balloon module: Arduino Uno with SFE_BMP180 pressure sensor attached
 * green -> SDA
 * blue -> SCL
 * white -> 3.3v
 * black -> GND
 */

#include <SFE_BMP180.h>

SFE_BMP180 pressureSensor;

class BalloonModule
{
    public:
        BalloonModule();
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
        unsigned long milliseconds, delayMilliseconds = 1000;
        byte hours = 0, minutes = 0, seconds = 0, launchTolerance = 0;
        double altitude, temperature;
};

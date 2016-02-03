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
        void printStatus();
        void printStatusAfterLaunch();
        void printTime();
        void printMetersAndFeet(double value);
        void printAltitude();
    private:
        int MET;
        byte hours = 0, minutes = 0, seconds = 0;
        unsigned long millisecondTime = 0;
        const int delayMilliseconds = 1000;    // time in milliseconds to delay (wait) each repetition
        int launchTolerance = 0;
};

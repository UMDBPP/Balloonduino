/*
 * BalloonModule.cpp
 * balloon module: Arduino Uno with SFE_BMP180 pressure sensor attached
 * green -> SDA
 * blue -> SCL
 * white -> 3.3v
 * black -> GND
 */

#include <BalloonModule.h>

// Base library type
BalloonModule::BalloonModule()
{
    // do nothing
}

// Prints altitude, returns altitude
void BalloonModule::printStatus()
{
    altitude = getAltitude();
    printTime();
    printAltitude();
    Serial.println();
}

// Same as printStatus, but only starts once launch is detected
void BalloonModule::printStatusAfterLaunch()
{
    if (isLaunched)
    {
        altitude = printStatus();
    }
    else
    {
        // Get the relative altitude difference between the new reading and the baseline reading
        altitude = getAltitude();
        
        // Check if current altitude is high enough to resume output (launch detected)
        if (altitude > 20.0)
        {
            launchTolerance++;
        }
        else
        {
            launchTolerance = 0;
        }
        if (launchTolerance > 5)
        {
            isLaunched = true;
            printTime();
            Serial.println("Launch detected. Resuming output.");
        }
    }
    delay (delayMilliseconds);
}

// Prints time in [HH:MM:SS]. Use this at the beginning of output lines to make it more readable
void BalloonModule::printTime()
{
    MET = millis() / 1000;    // convert from milliseconds to seconds
    seconds = MET % 60;
    minutes = MET / 60;
    hours = minutes / 60;
    Serial.print("[");
    if (hours < 10)
    {
        Serial.print("0");
    }
    Serial.print(hours);
    Serial.print(":");
    if (minutes < 10)
    {
        Serial.print("0");
    }
    Serial.print(minutes);
    Serial.print(":");
    if (seconds < 10)
    {
        Serial.print("0");
    }
    Serial.print(seconds);
    Serial.print("] ");
}

// Prints meters and feet equivalent to the console; for example, printMetersAndFeet(20000) prints "20000 meters (65616.96 feet)"
void BalloonModule::printMetersAndFeet(double meters)
{
    Serial.print(meters);
    Serial.print(" meters (");
    Serial.print(meters * 3.28084);
    Serial.print(" feet)");
}

// prints current altitude in meters and feet
void BalloonModule::printAltitude()
{
    Serial.print("Altitude is ");
    printMetersAndFeet (altitude);
    Serial.print(" above launch site. ");
}

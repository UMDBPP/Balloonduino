/*
 * Balloonduino.cpp
 * balloonduino: modified Arduino Mega
 */

#include <Balloonduino.h>

// Base library type
Balloonduino::Balloonduino()
{
    milliseconds = 0;
    delayMilliseconds = 1000;
    hours = 0;
    minutes = 0;
    seconds = 0;
    launchTolerance = 0;
    altitude = 0;
    temperature = 0;
    pressure = 0;
    isLaunched = false;
}

double Balloonduino::getAltitude()
{
    // TODO altitude function for Balloonduino
    return altitude;
}

double Balloonduino::getTemperature()
{
    // TODO temperature function for Balloonduino
    return temperature;
}

double Balloonduino::getPressure()
{
    // TODO pressure function for Balloonduino
    return pressure;
}

// TODO other Balloonduino sensors

// Given a meter value, prints meters and feet equivalent to the console without newline
// for example, printMetersAndFeet(20000) prints "20000 meters (65616.96 feet)"
void Balloonduino::printMetersAndFeet(double meters)
{
    Serial.print(meters);
    Serial.print(" meters (");
    Serial.print(meters * 3.28084);
    Serial.print(" feet)");
}

// Given a Celsius value, prints Celsius and Fahrenheit equivalent to the console without newline
// for example, printCelsiusAndFahrenheit(20.0) prints "20.0 Celsius (68.0 Fahrenheit)"
void Balloonduino::printCelsiusAndFahrenheit(double celsius)
{
    Serial.print(celsius);
    Serial.print(" Celsius (");
    Serial.print(celsius * 1.8 + 32);
    Serial.print(" Fahrenheit)");
}

// Given a Pascal value, prints Pascals and atmospheres equivalent to the console without newline
// for example, printPascalsAndAtmospheres(101325) prints "101325 Pascals (1 atm)"
void Balloonduino::printPascalsAndAtmospheres(double pascals)
{
    Serial.print(pascals);
    Serial.print(" Pascals (");
    Serial.print(pascals / 101325);
    Serial.print(" atm)");
}

// prints current altitude in meters and feet
void Balloonduino::printAltitude(double altitude)
{
    Serial.print("Altitude is ");
    printMetersAndFeet(altitude);
    Serial.print(" above launch site. ");
}

// prints current temperature in Celsius and Fahrenheit
void Balloonduino::printTemperature(double temperature)
{
    Serial.print("Temperature is ");
    printCelsiusAndFahrenheit(temperature);
    Serial.print(". ");
}

// prints current pressure in millibars
void Balloonduino::printPressure(double pressure)
{
    Serial.print("Pressure is ");
    printPascalsAndAtmospheres(pressure);
    Serial.print(". ");
}

// Prints current millisecond time in [HH:MM:SS] without newline
void Balloonduino::printFormattedTime()
{
    milliseconds = millis() / 1000;    // convert from milliseconds to seconds
    seconds = milliseconds % 60;
    minutes = milliseconds / 60;
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

// Prints current status of module to console
void Balloonduino::printStatusNow()
{
    altitude = getAltitude();
    temperature = getTemperature();
    pressure = getPressure();

    printFormattedTime();
    printAltitude (altitude);
    printTemperature (temperature);
    printPressure (pressure);
    Serial.println();
}

// prints status only after launch, otherwise does nothing
void Balloonduino::printStatusDuringFlight()
{
    // Get the relative altitude difference between the new reading and the baseline reading
    altitude = getAltitude();
    if (isLaunched)
    {
        printStatusNow();
    }
    else
    {

        // Check if current altitude is high enough to resume output (launch detected)
        if (altitude > 20.0)
        {
            launchTolerance++;
        }
        else
        {
            launchTolerance--;
        }
        if (launchTolerance > 5)
        {
            isLaunched = true;
            printFormattedTime();
            Serial.println("Launch detected. Resuming output.");
        }
    }
    delay (delayMilliseconds);
}

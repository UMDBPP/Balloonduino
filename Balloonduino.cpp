/*
 * Balloonduino.cpp
 * balloonduino: modified Arduino Mega
 */

#include <Balloonduino.h>

#ifdef useBME280
void Balloonduino::begin()
{
    delayMilliseconds = 1000;
    isLaunched = false;

    Serial.println("Initializing BME280...");

    if (BME280.begin())
    {
        Serial.println("BME280 initialized successfully.");
        Serial.println("Now attempting baseline pressure reading...");
    }
    else
    {
        Serial.println("BME280 failed (is it disconnected?)");
        Serial.println("System going to sleep.");
        while (1)
        {
            // infinite loop to pause forever
        }
    }
    // Get baseline pressure
    baselinePressure = getPressure();

    // Print baseline pressure and temperature
    Serial.print("Baseline pressure is ");
    Serial.print(baselinePressure);
    Serial.println(" mb.");
    Serial.print("Temperature is ");
    Serial.print(temperature);
    Serial.println(" C.");
}

double Balloonduino::getTemperature()
{
    temperature = BME280.readTemperature();
    return temperature;
}

double Balloonduino::getPressure()
{
    pressure = BME280.readPressure();
    return pressure;
}

double Balloonduino::getAltitude()
{
    getPressure();
    altitude = BME280.readAltitude();
    return altitude;
}

double Balloonduino::getHumidity()
{
    humidity = BME280.readHumidity();
    return humidity;
}
#endif

Balloonduino::Balloonduino()
{
    // do nothing
}

// prints current temperature in Celsius and Fahrenheit
void Balloonduino::printTemperature(double celsius)
{
    Serial.print("Temperature ");
    Serial.print(celsius);
    Serial.print(" C");
}

// prints current pressure in millibars
void Balloonduino::printPressure(double millibars)
{
    Serial.print("Pressure ");
    Serial.print(millibars);
    Serial.print(" mb");
}

// prints current altitude in meters and feet
void Balloonduino::printAltitude(double meters)
{
    Serial.print("Altitude ");
    Serial.print(meters);
    Serial.print(" meters");
}

void Balloonduino::printHumidity(double percentage)
{
    Serial.print("Humidity ");
    Serial.print(percentage);
    Serial.print("%");
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
    printFormattedTime();
    printTemperature (temperature);
    Serial.print(" | ");
    printAltitude (altitude);
    Serial.print(" | ");
    printPressure (pressure);
    Serial.print(" | ");
    printHumidity (humidity);
    Serial.println();
}

// prints status only after launch, otherwise does nothing
void Balloonduino::printStatusDuringFlight(double baselineAltitude)
{
    if (isLaunched)
    {
        printStatusNow();
    }
    else
    {
        // Check if current altitude is high enough to resume output (launch detected)
        if ((altitude - baselineAltitude) > 20.0)
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
void Balloonduino::printMillibarsAndAtmospheres(double millibars)
{
    Serial.print(millibars);
    Serial.print(" mb (");
    Serial.print(millibars / 1013.2501);
    Serial.print(" atm)");
}

// Given a meter value, prints meters and feet equivalent to the console without newline
// for example, printMetersAndFeet(20000) prints "20000 meters (65616.96 feet)"
void Balloonduino::printMetersAndFeet(double meters)
{
    Serial.print(meters);
    Serial.print(" meters (");
    Serial.print(meters * 3.28084);
    Serial.print(" feet)");
}


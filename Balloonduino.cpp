/*
 * Balloonduino.cpp
 * balloonduino: modified Arduino Mega
 */

#include <Balloonduino.h>

void Balloonduino::begin()
{
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
    Serial.print(getTemperature());
    Serial.println(" C.");

    Serial.println("Initializing BNO055...");

    if (BNO055.begin())
    {
        Serial.println("BNO055 initialized successfully.");
    }
    else
    {
        Serial.println("BNO055 failed (is it disconnected?)");
        Serial.println("System going to sleep.");
        while (1)
        {
            // infinite loop to pause forever
        }
    }

    Serial.println("Initializing DS1307...");

    if (DS1307.begin())
    {
        Serial.println("DS1307 initialized successfully.");
    }
    else
    {
        Serial.println("DS1307 failed (is it disconnected?)");
        Serial.println("System going to sleep.");
        while (1)
        {
            // infinite loop to pause forever
        }
    }

    if (!DS1307.isrunning())
    {
        Serial.println("DS1307 is NOT running!");
        // following line sets the RTC to the date & time this sketch was compiled
        DS1307.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
    else
    {
        startTime = DS1307.now();
        printRealTime();
    }
}

double Balloonduino::getTemperature()
{
    return BME280.readTemperature();
}

double Balloonduino::getPressure()
{
    return BME280.readPressure();
}

double Balloonduino::getAltitude()
{
    getPressure();
    altitude = BME280.readAltitude(SENSORS_PRESSURE_SEALEVELHPA);
    return altitude;
}

double Balloonduino::getHumidity()
{
    return BME280.readHumidity();
}

void Balloonduino::printTemperature(double celsius)
{
    Serial.print("T ");
    Serial.print(celsius);
    Serial.print("C. ");
}

void Balloonduino::printPressure(double millibars)
{
    Serial.print("P ");
    Serial.print(millibars);
    Serial.print("mb. ");
}

void Balloonduino::printAltitude(double meters)
{
    Serial.print("A ");
    Serial.print(meters);
    Serial.print("m. ");
}

void Balloonduino::printHumidity(double percentage)
{
    Serial.print("H ");
    Serial.print(percentage);
    Serial.print("%. ");
}

// prints time of day as [HH:MM:SS]
void Balloonduino::printRealTime()
{
    Serial.print("[");
    Serial.print(DS1307.now().hour(), DEC);
    Serial.print(":");
    Serial.print(DS1307.now().minute(), DEC);
    Serial.print(":");
    Serial.print(DS1307.now().second(), DEC);
    Serial.print("] ");
}

// Prints current millisecond time as [HH:MM:SS]
void Balloonduino::printMissionTime()
{
    byte seconds = millis() % 60, minutes = millis() / 60;
    byte hours = minutes / 60;

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

// Prints status report to console
void Balloonduino::printStatus()
{
    printRealTime();
    printTemperature (getTemperature());Serial
    .print(" | ");
    printAltitude (getAltitude());Serial
    .print(" | ");
    printPressure (getPressure());Serial
    .print(" | ");
    printHumidity (getHumidity());Serial
    .println();
}

// prints status only after launch, otherwise does nothing
void Balloonduino::printStatusDuringFlight()
{
    if (isLaunched)
    {
        printStatus();
    }
    else
    {
        // Check if current altitude is high enough to resume output
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
            printRealTime();
            Serial.println("Launch detected. Resuming output.");
        }
    }
    delay (delayMilliseconds);
}

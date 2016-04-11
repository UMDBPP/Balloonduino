/*
 * Balloonduino.cpp
 * balloonduino: modified Arduino Mega
 */

#include <Balloonduino.h>

byte numberOfSensors = 3;
bool sensors[3] =
    { true, true, true };    // array for sensor status (running / error)

// Displays sensor errors to pin 13 (built in LED), blinks once for OK status, twice otherwise
void displaySensorErrors()
{
    digitalWrite(13, LOW);
    delay(500);
    for (byte index; index < numberOfSensors; index++)
    {
        digitalWrite(13, HIGH);
        delay(125);
        digitalWrite(13, LOW);
        delay(125);
        if (sensors[index])
        {
            digitalWrite(13, HIGH);
            delay(125);
            digitalWrite(13, LOW);
            delay(125);
        }
        else
        {
            delay(250);
        }
        delay(250);
    }
    delay(500);
}

// Initializes all sensors. Needs to be run before any measurements can be taken.
void Balloonduino::begin()
{
// initialize pressure / temperature / humidity sensor
    Serial.println(
            "Initializing BME280 pressure, temperature, and humidity sensor...");
    if (BME280.begin())
    {
        Serial.println("BME280 initialized successfully.");
        Serial.println("Now attempting baseline reading...");
    }
    else
    {
        Serial.println("BME280 failed (is it disconnected?)");
        Serial.println("System going to sleep.");
        while (1)
        {
        }
    }
// Print baseline pressure and temperature
    Serial.println("Pressure is " + getPressureString());
    Serial.println("Temperature is " + getTemperatureString());
    Serial.println("Humidity is " + getHumidityString());

// initialize orientation sensor
    Serial.println("Initializing BNO055 orientation sensor...");
    if (BNO055.begin())
    {
        Serial.println("BNO055 initialized successfully.");
    }
    else
    {
        Serial.println("BNO055 failed (is it disconnected?)");

    }

// initialize real time clock
    Serial.println("Initializing DS1307 real time clock...");
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
        Serial.println("Current time is " + getRealTimeString());
    }
}

// Returns temperature double in degrees Celsius
double Balloonduino::getTemperature()
{
    return BME280.readTemperature();
}

// Returns pressure double in millibars
double Balloonduino::getPressure()
{
    return BME280.readPressure();
}

// Returns altitude double in meters
double Balloonduino::getAltitude()
{
    return BME280.readAltitude();
}

// Returns humidity double in percentage
double Balloonduino::getHumidity()
{
    return BME280.readHumidity();
}

// Returns temperature string as "-60C"
String Balloonduino::getTemperatureString()
{
    return String(getTemperature) + "C";
}

// Returns pressure string as "500mb"
String Balloonduino::getPressureString()
{
    return String(getPressure()) + "mb";
}

// Returns altitude string as "1050m"
String Balloonduino::getAltitudeString()
{
    return String(getAltitude()) + "m";
}

// Returns humidity as "75%"
String Balloonduino::getHumidityString()
{
    return String(getHumidity()) + "%";
}

// Returns current millisecond time string as "T+HH:MM:SS"
String Balloonduino::getMissionTimeString()
{
    byte seconds = millis() % 60, minutes = millis() / 60;
    byte hours = minutes / 60;

    return "T+" + ((hours < 10) ? "0" : "") + String(hours) + ":"
            + ((minutes < 10) ? "0" : "") + String(minutes) + ":"
            + ((seconds < 10) ? "0" : "") + String(seconds);
}

// Returns current time of day string as "hh:mm:ss a"
String Balloonduino::getRealTimeString()
{
    return String(DS1307.now().hour()) + ":" + String(DS1307.now().minute())
            + ":" + String(DS1307.now().second());
}

// Returns status report string as "[HH:MM:SS] | -60C | 1050m | 500mb | 75% |"
String Balloonduino::getStatusString()
{
    return getRealTimeString() + getTemperatureString() + " | "
            + getAltitudeString() + " | " + getPressureString() + " | "
            + getHumidityString() + " | ";
}

// Private function, updates sensor status array
void updateSensorStatus(byte address, bool status)
{
    sensors[address] = status;
}

/*
 * Balloonduino.cpp
 * balloonduino: modified Arduino Mega
 */

#include <Balloonduino.h>

// Displays sensor errors to pin 13 (built in LED), blinks once for OK status, twice otherwise
void Balloonduino::displaySensorStatus()
{
    // set built in LED to off for half a second
    digitalWrite(13, LOW);
    delay(500);

    // iterate through the sensor statuses in the sensor status array
    for (byte index = 0; index < numberOfSensors; index++)
    {
        // blink LED once for 125 millisecond interval
        digitalWrite(13, HIGH);
        delay(125);
        digitalWrite(13, LOW);
        delay(125);

        // if sensor has an error, blink LED again, otherwise wait 250 milliseconds with LED off
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

        // wait an additional 250 milliseconds between each sensor in the iteration
        delay(250);
    }

    // wait an additional 250 milliseconds between each iteration completion and the start of the next iteration
    delay(250);
}

// Initializes all sensors. Needs to be run once in setup before any measurements can be taken.
void Balloonduino::begin()
{
    // Initialize pressure / temperature / humidity sensor
    print("Initializing BME280 pressure, temperature, and humidity sensor...");
    if (BME280.begin())
    {
        updateSensorStatus(0, 0);
        print("BME280 initialized successfully.");
        print("Now attempting baseline reading...");
        // Print baseline pressure and temperature
        print("Pressure is " + String(getPressure()) + "mb.");
        print("Temperature is " + String(getTemperature()) + "C.");
        print("Humidity is " + String(getHumidity()) + "%.");
    }
    else
    {
        updateSensorStatus(0, 1);
        print("BME280 failed (is it disconnected?)");
    }

    // Initialize orientation sensor
    print("Initializing BNO055 orientation sensor...");
    if (BNO055.begin())
    {
        updateSensorStatus(1, 0);
        print("BNO055 initialized successfully.");
    }
    else
    {
        updateSensorStatus(1, 1);
        print("BNO055 failed (is it disconnected?)");
    }

    // Initialize real time clock
    print("Initializing DS1307 real time clock...");

    // RTCLib library will always return successful initialization, regardless of whether connected or not
    DS1307.begin();

    // Query if it is running to get status
    if (!DS1307.isrunning())
    {
        print("DS1307 failed (is it disconnected?)");
        updateSensorStatus(2, 1);

        // following line sets the RTC to the date & time this sketch was compiled
        DS1307.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    else
    {
        print("DS1307 initialized successfully.");
        print("Current time is " + getRealTimeString());
    }

    // log completion and display sensor status for one iteration
    print("Initialization completed. Check console for errors.");
    displaySensorStatus();
}

// Returns temperature as a double in degrees Celsius
double Balloonduino::getTemperature()
{
    return BME280.readTemperature();
}

// Returns pressure as adouble in millibars
double Balloonduino::getPressure()
{
    return BME280.readPressure();
}

// Returns altitude as a double in meters
double Balloonduino::getAltitude()
{
    return BME280.readAltitude(SENSORS_PRESSURE_SEALEVELHPA);
}

// Returns humidity as a double in percentage
double Balloonduino::getHumidity()
{
    return BME280.readHumidity();
}

// Returns current millisecond time as a string in format "T+HH:MM:SS"
String Balloonduino::getMissionTimeString()
{
    byte seconds = millis() % 60, minutes = millis() / 60;
    byte hours = minutes / 60;

    String out = "T+";
    if (hours < 10)
    {
        out += "0";
    }
    out += String(hours) + ":";
    if (minutes < 10)
    {
        out += "0";
    }
    out += String(minutes) + ":";
    if (seconds < 10)
    {
        out += "0";
    }
    out += String(seconds);
    return out;
}

// Returns current time of day as a string in format "hh:mm:ss"
String Balloonduino::getRealTimeString()
{
    return String(DS1307.now().hour()) + ":" + String(DS1307.now().minute())
            + ":" + String(DS1307.now().second());
}

// Returns status report as a string in format "[hh:mm:ss] | -60C | 1050m | 500mb | 75% |"
String Balloonduino::getStatusString()
{
    return getRealTimeString() + String(getTemperature()) + "C | "
            + String(getAltitude()) + "m | " + String(getPressure()) + "mb | "
            + String(getHumidity()) + "% | ";
}

// Private function, prints to serial
void Balloonduino::print(String message)
{
    Serial.println("[" + getMissionTimeString() + "] " + message);
}

// Private function, updates sensor status array
void Balloonduino::updateSensorStatus(byte address, byte status)
{
    sensors[address] = status;
}

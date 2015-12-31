/*
 * BalloonModule.cpp
 * balloon module: Arduino Uno with SFE_BMP180 pressure sensor attached to 
 * green -> SDA, blue -> SCL, white -> 3.3v, black -> GND
 */

#include <BalloonModule.h>

// Base library type
BalloonModule::BalloonModule()
{
    // do nothing
}

// Initialize pressure sensor (start readings). This should be the first line of setup.
void BalloonModule::initialize()
{
    Serial.begin(9600);
    
    printTime();
    Serial.println("Initializing...");
    
    if (pressureSensor.begin())
    {
        printTime();
        if (pressureSensor.begin())
        {
            Serial.println("Pressure sensor successfully initialized.");
            printTime();
            Serial.println("Now attempting baseline pressure reading...");
        }
        else
        {
            Serial.println(
                    "Pressure sensor initialization failed (is it disconnected?)");
            printTime();
            Serial.println("System going to sleep.");
            while (1)
            {
                // infinite loop to pause forever
            }
        }
        
        delay(1000);
        
        // Get baseline pressure
        baselinePressure = getPressure();
        
        // Print baseline pressure
        printTime();
        Serial.print("Baseline pressure is ");
        Serial.print(baselinePressure);
        Serial.print(" mb.");
        Serial.println();
        
        printTime();
        Serial.println(
                "System is ready for launch. Output will resume when launch is detected.");
    }
    delay(3000);
}

// Main looping code to detect and print altitude. Starts once launch is detected
void BalloonModule::printStatusAfterLaunch()
{
    // Get the relative altitude difference between the new reading and the baseline reading
    altitude = module.getRelativeAltitude();
    
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
    
    if (isLaunched)
    {
        printTime();
        module.printAltitude();
        Serial.println();
    }
    
    delay (delayMilliseconds);
}

// Returns current pressure reading using temperature
double BalloonModule::getPressure()
{
    char status;
    
    // You must first get a temperature measurement to perform a pressure reading.
    
    /*
     * Start a temperature measurement:
     * If request is successful, the number of ms to wait is returned.
     * If request is unsuccessful, 0 is returned.
     */
    status = pressureSensor.startTemperature();
    
    if (status != 0)
    {
        // Wait for the measurement to complete:
        delay(status);
        
        /*
         * Retrieve the completed temperature measurement:
         * Note that the measurement is stored in the variable 'temperature'.
         * Use '&temperature' to provide the address of temperature to the function.
         * Function returns 1 if successful, 0 if failure.
         */
        status = pressureSensor.getTemperature(temperature);
        
        if (status != 0)
        {
            /*
             * Start a pressure measurement:
             * The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
             * If request is successful, the number of ms to wait is returned.
             * If request is unsuccessful, 0 is returned.
             */
            status = pressureSensor.startPressure(3);
            
            if (status != 0)
            {
                // Wait for the measurement to complete:
                delay(status);
                
                /*
                 * Retrieve the completed pressure measurement:
                 * Note that the measurement is stored in the variable 'pressure'.
                 * Use '&pressure' to provide the address of pressure.
                 * Note also that the function requires the previous temperature measurement.
                 * (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
                 * Function returns 1 if successful, 0 if failure.
                 */
                status = pressureSensor.getPressure(pressure, temperature);
                
                if (status != 0)
                {
                    return (pressure);
                }
                else
                {
                    Serial.println("Error retrieving pressure measurement.");
                    return 0.0;
                }
            }
            else
            {
                Serial.println("Error starting pressure measurement.");
                return 0.0;
            }
        }
        else
        {
            Serial.println(
                    "Error retrieving required temperature measurement.");
            return 0.0;
        }
    }
    else
    {
        Serial.println("Error starting required temperature measurement.");
        return 0.0;
    }
}

// Returns current altitude difference from baseline reading using current pressure
double BalloonModule::getRelativeAltitude()
{
    return pressureSensor.altitude(getPressure(), baselinePressure);
}

// Prints time in [HH:MM:SS]
void BalloonModule::printTime()
{
    MET = (double) millis() / 1000.0;    // convert from milliseconds to seconds
    seconds = (int) MET % 60;
    minutes = MET / 60;
    hours = minutes / 60;
    Serial.print("[");
    if (hours < 10)
    {
        Serial.print("0");
    }
    Serial.print((int) hours);
    Serial.print(":");
    if (minutes < 10)
    {
        Serial.print("0");
    }
    Serial.print((int) minutes);
    Serial.print(":");
    if (seconds < 10)
    {
        Serial.print("0");
    }
    Serial.print((int) seconds);
    Serial.print("] ");
}

// Prints meters and feet equivalent to the console; for example, printMetersAndFeet(20000) prints "20000 meters (65616.96 feet)"
void BalloonModule::printMetersAndFeet(double value)
{
    Serial.print(value);
    Serial.print(" meters (");
    Serial.print(value * 3.28084);
    Serial.print(" feet)");
}

// prints current altitude
void BalloonModule::printAltitude()
{
    Serial.print("Altitude is ");
    printMetersAndFeet (altitude);
    Serial.print(" above launch site. ");
}

#include "Balloonduino.h"

Balloonduino balloonduino;

void setup()
{
    balloonduino.begin(0x0006);
}

void loop()
{
    balloonduino.log_imu();
    balloonduino.log_pwr();
    balloonduino.log_env();

    delay(500);
}

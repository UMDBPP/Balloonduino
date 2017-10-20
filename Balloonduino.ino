#include <Balloonduino.h>

void setup()
{
    Balloonduino balloonduino = Balloonduino();
    balloonduino.begin(xbee_address = 0x0006);
}

void loop()
{
    balloonduino.log_imu();
    balloonduino.log_pwr();
    balloonduino.log_env();

    delay(500);
}

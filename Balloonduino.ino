#include <Balloonduino.h>

#define XBEE_ADDR 0x0006
#define XBEE_PAN_ID 0x0B0B
#define LINK_XBEE_ADDRESS 0x0002

Balloonduino balloonduino = Balloonduino();

void setup()
{
    balloonduino.begin();
}

void loop()
{
    // increment read counters
    imu_read_ctr++;
    pwr_read_ctr++;
    env_read_ctr++;

    // read sensors if time between last read
    if (imu_read_ctr > imu_read_lim)
    {
        read_imu (&IMUData);
        log_imu(IMUData, IMULogFile);
        imu_read_ctr = 0;
    }
    if (pwr_read_ctr > pwr_read_lim)
    {
        read_pwr (&PWRData);
        log_pwr(PWRData, PWRLogFile);
        pwr_read_ctr = 0;
    }
    if (env_read_ctr > env_read_lim)
    {
        read_env (&ENVData);
        log_env(ENVData, ENVLogFile);
        env_read_ctr = 0;
    }
}

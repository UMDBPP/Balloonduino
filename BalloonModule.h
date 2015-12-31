/*
 BalloonModule.h
 */

SFE_BMP180 pressureSensor;

class BalloonModule
{
    public:
        BalloonModule();    // base type
        
        char begin();

        double getAltitude(double P, double P0);
};

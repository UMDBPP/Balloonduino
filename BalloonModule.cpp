/*
 BalloonModule.cpp
 */

#include <BalloonModule.h>
#include <SFE_BMP180.h>

BalloonModule::BalloonModule()
// Base library type
{
}

char BalloonModule::begin()
{
    
}

double BalloonModule::getAltitude(double P, double P0)
{
    return (44330.0 * (1 - pow(P / P0, 1 / 5.255)));
}

#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#include <Servo.h>
#include <Sensors.cpp>

const int altitudeThreshold; //Meters
bool airbrakeDeployed;

void initializeAirbrake();

openAirbrake(bool airbrakeDeployed);
closeAirbrake(bool airbrakeDeployed);

#endif
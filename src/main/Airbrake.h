#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#include <Servo.h>
#include <Sensors.cpp>

void initializeAirbrake();
void openAirbrake(bool airbrakeDeployed, float currentAltitude);
void closeAirbrake(bool airbrakeDeployed, float currentAltitude);

#endif
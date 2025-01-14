#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#include <Servo.h>
#include <Sensors.cpp>

const int airbrake1_pin;
const int airbrake2_pin;
const int alt_threshold; //Meters
bool airbrake_deployed;

const int airbrake_open;
const int airbrake_closed;

openAirbrake(bool &airbrake_deployed);
closeAirbrake(bool &airbrake_deployed);

#endif
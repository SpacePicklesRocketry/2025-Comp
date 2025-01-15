#ifndef PARACHUTE_H
#define PARACHUTE_H

#include <Servo.h>
#include <Sensors.cpp>

const int door_servo_pin;
const int parachute_servo_pin;
const int alt_threshold; //Meters
bool altitude_passed;
bool parachute_deployed;

const int door_open;
const int parachute_open;
const int close_servos;

void deploy_parachute(bool &altitude_passed, bool &door_opened);
void close_parachute_servos(bool &altitude_passed, bool &door_opened);

#endif
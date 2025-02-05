#ifndef PARACHUTE_H
#define PARACHUTE_H

#include <Servo.h>
#include <Sensors.cpp>

void deployParachute(bool altitudePassed, bool doorOpened);
void closeParachuteServo(bool altitudePassed, bool doorOpened);

#endif
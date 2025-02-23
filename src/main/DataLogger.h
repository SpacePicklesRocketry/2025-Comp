#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <Arduino.h>
#include "Sensors.h"
#include "Airbrake.h"

void initializeSDCard();
void createLogFile();
// void logData(const SensorData& data, const AirbrakeStatus& airbrake);
void logData(const SensorData& data);


#endif
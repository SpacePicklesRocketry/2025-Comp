#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <Arduino.h>
#include "Sensors.h" // Include Sensors.h to use the SensorData structure

// Function prototypes
void initializeSDCard();
void createLogFile();
void logData(const SensorData& data);

#endif
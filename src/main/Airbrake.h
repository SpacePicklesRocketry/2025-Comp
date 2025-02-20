#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#include <Arduino.h>
#include "Sensors.h"

struct AirbrakeStatus {
    bool airbrakesDeployed;
};

// Globals
extern AirbrakeStatus airbrakeStatus;

void initializeAirbrake();
void updateAirbrake(const SensorData& sensorData); 
void setDeploymentDelay(float delaySeconds); 
void deployAirbrakes();

#endif 
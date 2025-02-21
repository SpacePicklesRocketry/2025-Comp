#ifndef PARACHUTE_H
#define PARACHUTE_H

#include <Arduino.h>
#include "Sensors.h"

struct ParachuteStatus {
    bool parachuteDeployed;
};

// Globals
extern ParachuteStatus parachuteStatus;

void initializeParachute();
void updateParachute(const SensorData& sensorData);
void setDeploymentDelayParachute(float delaySeconds);
void deployParachute();

#endif // PARACHUTE_H
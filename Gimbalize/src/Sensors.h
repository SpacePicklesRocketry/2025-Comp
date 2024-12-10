#ifndef SENSORS_H
#define SENSORS_H

#include "KalmanFilter.h"

struct SensorData {
  float angleX, angleY, angleZ;
  float temperature, pressure, altitude;
  unsigned long timestamp;
};

void initializeSensors();
SensorData readSensors(float deltaTime, KalmanFilter& kalmanX, KalmanFilter& kalmanY, KalmanFilter& kalmanZ);

#endif
#ifndef SENSORS_H
#define SENSORS_H

#include <SimpleKalmanFilter.h>

#define GRAVITY 9.81

struct SensorData {
    unsigned long timestamp;
    float accelX, accelY, accelZ;  // Acceleration (m/s^2)
    float gyroX, gyroY, gyroZ;    // Angular velocity (deg/s)
    float roll, pitch, yaw; // Roll, pitch, yaw (degrees)
    float velocityX, velocityY, velocityZ; // Velocity (m/s)
    float positionX, positionY, positionZ; // Position (m)
    float altitude; // Altitude (m)
    float rateOfChange;
};

// m_ea, e_ea, q
extern SimpleKalmanFilter altitudeFilter;
extern SimpleKalmanFilter gyroFilter;

void initializeSensors();
SensorData readSensors(float deltaTime, SensorData& previousData);
float readAltitudeFromBMP();

#endif
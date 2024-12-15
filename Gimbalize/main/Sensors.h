#ifndef SENSORS_H
#define SENSORS_H

#include "KalmanFilter.h"
#include <SimpleKalmanFilter.h>


#define GRAVITY 9.81          // Acceleration due to gravity (m/s^2)
#define ACCEL_THRESHOLD 0.05  // Acceleration noise threshold (m/s^2)
#define GYRO_THRESHOLD 0.1    // Gyroscope noise threshold (deg/s)

// Structure to store sensor data
struct SensorData {
    unsigned long timestamp;
    float accelX, accelY, accelZ;  // Acceleration (m/s^2)
    float gyroX, gyroY, gyroZ;    // Angular velocity (deg/s)
    float angleX, angleY, angleZ; // Roll, pitch, yaw (degrees)
    float velocityX, velocityY, velocityZ; // Velocity (m/s)
    float positionX, positionY, positionZ; // Position (m)
};

// Declare Kalman filter objects (extern)
extern SimpleKalmanFilter kalmanVelX;
extern SimpleKalmanFilter kalmanVelY;
extern SimpleKalmanFilter kalmanVelZ;
extern SimpleKalmanFilter kalmanPosX;
extern SimpleKalmanFilter kalmanPosY;
extern SimpleKalmanFilter kalmanPosZ;

// Function prototypes
void initializeSensors();
SensorData readSensors(float deltaTime, SensorData& previousData);
void getRawData(float& accelX, float& accelY, float& accelZ, float& gyroX, float& gyroY, float& gyroZ);

#endif
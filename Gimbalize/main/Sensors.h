#ifndef SENSORS_H
#define SENSORS_H

#include <SimpleKalmanFilter.h>

// Constants
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
    float altitude; // Altitude (m)
    float rateOfChange;
};

// Declare Kalman filter object for altitude (extern)
extern SimpleKalmanFilter kalmanAltitude;

// Function prototypes
void initializeSensors();
SensorData readSensors(float deltaTime, SensorData& previousData);
float readAltitudeFromBMP(); // Function to read altitude from BMP sensor
void getRawData(float& accelX, float& accelY, float& accelZ, float& gyroX, float& gyroY, float& gyroZ);

#endif
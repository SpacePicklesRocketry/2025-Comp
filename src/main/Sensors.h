#ifndef SENSORS_H
#define SENSORS_H

#include <SimpleKalmanFilter.h>

#define GRAVITY 9.81
#define LIFTOFF_ACCELERATION_THRESHOLD 30 // TODO
#define APOGEE_DETECTION_THRESHOLD 2 //TODO

struct SensorData {
    unsigned long timestamp;
    float accelX, accelY, accelZ;  // Acceleration (m/s^2)
    float gyroX, gyroY, gyroZ;    // Angular velocity (deg/s)
    float roll, pitch, yaw; // Roll, pitch, yaw (degrees)
    float velocityX, velocityY, velocityZ; // Velocity (m/s)
    float positionX, positionY, positionZ; // Position (m)
    float altitude; // Altitude (m)
    bool liftoffDetected; // Flag for liftoff detection
    unsigned long liftoffTime; // Timestamp of liftoff
    bool apogeeDetected; // Flag for apogee detection
    unsigned long apogeeTime; // Timestamp of apogee
    float rateOfChange;  //(m/s)
    double latitude;
    double longitude;
    int satellites;
};

// Kalman Filters
extern SimpleKalmanFilter altitudeFilter;
extern SimpleKalmanFilter gyroFilter;

void initializeSensors();
SensorData readSensors(float deltaTime, SensorData& previousData);
float readAltitudeFromBMP();

#endif 
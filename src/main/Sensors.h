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
    bool liftoffDetected; // Flag for liftoff detection
    unsigned long liftoffTime; // Timestamp of liftoff
    bool apogeeDetected; // Flag for apogee detection
    unsigned long apogeeTime; // Timestamp of apogee
    float rateOfChange;  //(m/s)
};

// Kalman Filters
extern SimpleKalmanFilter altitudeFilter;
extern SimpleKalmanFilter gyroFilter;

//motor types to determin threshold values
enum MotorType {
    F26,
    F15,
    F44
};

//Motor type global
extern MotorType motor;

extern int LIFTOFF_ACCELERATION_THRESHOLD;
extern float APOGEE_DETECTION_THRESHOLD;

void thresholdConfig(MotorType motor);

void initializeSensors();
SensorData readSensors(float deltaTime, SensorData& previousData);
float readAltitudeFromBMP();

#endif 
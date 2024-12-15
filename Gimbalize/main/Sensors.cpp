#include "Sensors.h"
#include <MKRIMU.h>
#include <math.h>
#include <SimpleKalmanFilter.h>

// Define Kalman filter objects
SimpleKalmanFilter kalmanVelX(0.5, 0.75, 100.0); // Velocity X-axis
SimpleKalmanFilter kalmanVelY(0.5, 0.75, 100.0); // Velocity Y-axis
SimpleKalmanFilter kalmanVelZ(0.5, 0.75, 100.0); // Velocity Z-axis

SimpleKalmanFilter kalmanPosX(0.5, 2.0, 100.0); // Position X-axis
SimpleKalmanFilter kalmanPosY(0.5, 2.0, 100.0); // Position Y-axis
SimpleKalmanFilter kalmanPosZ(0.5, 2.0, 100.0); // Position Z-axis

void initializeSensors() {
    if (!IMU.begin()) {
        Serial.println("Failed to initialize MKRIMU");
        while (1) delay(10); // Halt on failure
    }
    Serial.println("MKRIMU Initialized");
}

SensorData readSensors(float deltaTime, SensorData& previousData) {
    SensorData data;

    float accelX_raw = 0, accelY_raw = 0, accelZ_raw = 0;
    float gyroX_raw = 0, gyroY_raw = 0, gyroZ_raw = 0;

    // Read gyroscope data
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX_raw, gyroY_raw, gyroZ_raw);

        // Convert gyroscope readings to degrees per second
        data.gyroX = gyroX_raw * 180.0 / M_PI;
        data.gyroY = gyroY_raw * 180.0 / M_PI;
        data.gyroZ = gyroZ_raw * 180.0 / M_PI;

        // Update roll, pitch, yaw using gyroscope data
        data.angleX = kalmanVelX.updateEstimate(previousData.angleX + data.gyroX * deltaTime);
        data.angleY = kalmanVelY.updateEstimate(previousData.angleY + data.gyroY * deltaTime);
        data.angleZ = kalmanVelZ.updateEstimate(previousData.angleZ + data.gyroZ * deltaTime);
    }

    // Read accelerometer data
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX_raw, accelY_raw, accelZ_raw);

        // Gravity compensation using roll and pitch
        float gravityX = GRAVITY * sin(data.angleY * M_PI / 180.0); // Pitch
        float gravityY = -GRAVITY * sin(data.angleX * M_PI / 180.0); // Roll
        float gravityZ = GRAVITY * cos(data.angleX * M_PI / 180.0) * cos(data.angleY * M_PI / 180.0);

        // Subtract gravity from raw accelerometer values
        data.accelX = accelX_raw * GRAVITY - gravityX;
        data.accelY = accelY_raw * GRAVITY - gravityY;
        data.accelZ = accelZ_raw * GRAVITY - gravityZ;

        // Threshold small accelerations to reduce noise
        if (fabs(data.accelX) < ACCEL_THRESHOLD) data.accelX = 0.0;
        if (fabs(data.accelY) < ACCEL_THRESHOLD) data.accelY = 0.0;
        if (fabs(data.accelZ) < ACCEL_THRESHOLD) data.accelZ = 0.0;

        // Integrate acceleration to calculate velocity
        float rawVelocityX = previousData.velocityX + data.accelX * deltaTime;
        float rawVelocityY = previousData.velocityY + data.accelY * deltaTime;
        float rawVelocityZ = previousData.velocityZ + data.accelZ * deltaTime;

        // Use Kalman filters for velocity
        data.velocityX = kalmanVelX.updateEstimate(rawVelocityX);
        data.velocityY = kalmanVelY.updateEstimate(rawVelocityY);
        data.velocityZ = kalmanVelZ.updateEstimate(rawVelocityZ);

        // Reset velocity when stationary
        if (fabs(data.accelX) < ACCEL_THRESHOLD && fabs(data.gyroX) < GYRO_THRESHOLD) data.velocityX = 0.0;
        if (fabs(data.accelY) < ACCEL_THRESHOLD && fabs(data.gyroY) < GYRO_THRESHOLD) data.velocityY = 0.0;
        if (fabs(data.accelZ) < ACCEL_THRESHOLD && fabs(data.gyroZ) < GYRO_THRESHOLD) data.velocityZ = 0.0;

        // Integrate velocity to calculate position
        float rawPositionX = previousData.positionX + data.velocityX * deltaTime;
        float rawPositionY = previousData.positionY + data.velocityY * deltaTime;
        float rawPositionZ = previousData.positionZ + data.velocityZ * deltaTime;

        // Use Kalman filters for position
        data.positionX = kalmanPosX.updateEstimate(rawPositionX);
        data.positionY = kalmanPosY.updateEstimate(rawPositionY);
        data.positionZ = kalmanPosZ.updateEstimate(rawPositionZ);

        // Reset position when stationary
        if (data.velocityX == 0.0 && data.velocityY == 0.0 && data.velocityZ == 0.0) {
            data.positionX = previousData.positionX;
            data.positionY = previousData.positionY;
            data.positionZ = previousData.positionZ;
        }
    }

    data.timestamp = millis();
    return data;
}

// Function to get raw sensor data
void getRawData(float& accelX, float& accelY, float& accelZ, float& gyroX, float& gyroY, float& gyroZ) {
    // Initialize values to zero
    accelX = accelY = accelZ = 0;
    gyroX = gyroY = gyroZ = 0;

    // Read accelerometer data
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
    }

    // Read gyroscope data
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
    }

    // Convert gyroscope data to degrees/second
    gyroX *= 180.0 / M_PI;
    gyroY *= 180.0 / M_PI;
    gyroZ *= 180.0 / M_PI;
}
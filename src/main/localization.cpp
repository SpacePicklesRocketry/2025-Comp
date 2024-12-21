#include "localization.h"
#include <Arduino.h>
#include <math.h>

RocketLocalization::RocketLocalization() {
    memset(x, 0, sizeof(x)); // Initialize state vector to 0
    memset(P, 0, sizeof(P)); // Initialize covariance matrix to 0

    for (int i = 0; i < STATE_DIM; i++) {
        F[i][i] = 1.0; // Identity for simple state updates
        Q[i][i] = 0.1; // Process noise (tune as needed)
    }

    for (int i = 0; i < MEASUREMENT_DIM; i++) {
        H[i][i] = 1.0; // Direct measurement mapping
        R[i][i] = 0.1; // Measurement noise (tune as needed)
    }
}

void RocketLocalization::initialize() {
    initializeSensors(); // Initialize sensor hardware
    Serial.println("Initializing Kalman filter...");
}

void RocketLocalization::predict(float dt) {
    // Update linear states
    for (int i = 0; i < STATE_DIM; i++) {
        float newState = 0;
        for (int j = 0; j < STATE_DIM; j++) {
            newState += F[i][j] * x[j];
        }
        x[i] = newState;
    }

    // Update orientation using gyroscope
    x[6] += x[3] * dt; // Roll
    x[7] += x[4] * dt; // Pitch
    x[8] += x[5] * dt; // Yaw
}

void RocketLocalization::update(const SensorData& data, float deltaTime) {
    float epsilon = 1e-6; // Small value to prevent division by zero

    // Calculate roll and pitch from accelerometer
    float rollAccel = atan2(data.accelY, data.accelZ + epsilon) * 180.0 / M_PI; // Roll in degrees
    float pitchAccel = atan2(-data.accelX, sqrt(data.accelY * data.accelY + data.accelZ * data.accelZ) + epsilon) * 180.0 / M_PI;

    // Fuse accelerometer and gyroscope data using Kalman filter
    x[6] = kalmanX.update(rollAccel, data.gyroX, deltaTime); // Roll
    x[7] = kalmanY.update(pitchAccel, data.gyroY, deltaTime); // Pitch
    x[8] = kalmanZ.update(x[8] + data.gyroZ * deltaTime, data.gyroZ, deltaTime); // Yaw
}

float* RocketLocalization::getState() {
    return x; // Return the state vector
}
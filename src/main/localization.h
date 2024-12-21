#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "KalmanFilter.h"
#include "Sensors.h"

#define STATE_DIM 9
#define MEASUREMENT_DIM 4

class RocketLocalization {
private:
    float x[STATE_DIM];       // State vector
    float P[STATE_DIM][STATE_DIM]; // Covariance matrix
    float F[STATE_DIM][STATE_DIM]; // State transition matrix
    float Q[STATE_DIM][STATE_DIM]; // Process noise matrix
    float H[MEASUREMENT_DIM][STATE_DIM]; // Measurement matrix
    float R[MEASUREMENT_DIM][MEASUREMENT_DIM]; // Measurement noise matrix

public:
    KalmanFilter kalmanX, kalmanY, kalmanZ; // Kalman filters for orientation
    RocketLocalization();
    void initialize();
    void predict(float dt);
    void update(const SensorData& data, float deltaTime);
    float* getState(); // Access the state vector
};

#endif
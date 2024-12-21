#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
  Q_angle = 0.001;
  Q_gyro = 0.003;
  R_angle = 0.03;
  bias = 0;
  angle = 0;
  P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
}

float KalmanFilter::update(float newAngle, float newRate, float deltaTime) {
  angle += deltaTime * (newRate - bias);
  P[0][0] += deltaTime * (deltaTime * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= deltaTime * P[1][1];
  P[1][0] -= deltaTime * P[1][1];
  P[1][1] += Q_gyro * deltaTime;

  float S = P[0][0] + R_angle;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}
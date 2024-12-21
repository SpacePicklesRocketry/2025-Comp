#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
private:
  float Q_angle;
  float Q_gyro;
  float R_angle;
  float bias;
  float angle;
  float P[2][2];

public:
  KalmanFilter();
  float update(float newAngle, float newRate, float deltaTime);
};

#endif
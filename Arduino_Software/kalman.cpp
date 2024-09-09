#include "kalman.h"

Kalman::KalmanFilter(float processNoise, float sensorNoise, float estimatedError, Vector3 initialValue) {
  this->Q = processNoise;
  this->R = sensorNoise;
  this->P = estimatedError;
  this->X = initialValue;
  this->K = Vector3();
}

Vector3 Kalman::update(Vector3 measurement) {
  // Prediction update
  P = P + Q;
  // Measurement update
  K.x = P / (P + R);
  K.y = P / (P + R);
  K.z = P / (P + R);

  X.x = X.x + K.x * (measurement.x - X.x);
  X.y = X.y + K.y * (measurement.y - X.y);
  X.z = X.z + K.z * (measurement.z - X.z);

  P = (1 - K.x) * P;  // Since P is scalar, we just use one value

  return X;
}
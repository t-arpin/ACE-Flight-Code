/*
  Kalman filter for orientation
 */

#include "VectorDefinition.h"

class Kalman {
  public:
    KalmanFilter(float processNoise, float sensorNoise, float estimatedError, Vector3 initialValue);
    Vector3 update(Vector3 measurement);

  private:
    float Q;  // Process noise covariance
    float R;  // Measurement noise covariance
    float P;  // Estimation error covariance
    Vector3 K;  // Kalman gain
    Vector3 X;  // Value
};
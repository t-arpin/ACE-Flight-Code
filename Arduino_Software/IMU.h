/*
  Inertial measurement unit MPU6050
  
  IMU = inertial measurement unit
  
  Outputs the following data: angular velocity and acceleration
 */

#include <MPU6050.h>
#include <Wire.h>
#include "VectorDefinition.h"

class IMU {
  public:
    IMU();
    Vector3 getGyros();
    Vector3 getAcceleration();
    Vector3 getOrientation();
  private:
    MPU6050 mpu;

    int i = 1;
    int16_t ax, ay, az, gx, gy, gz;

    double timeStep, time, timePrev;
    double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;

    double gyroScale = 131;
};
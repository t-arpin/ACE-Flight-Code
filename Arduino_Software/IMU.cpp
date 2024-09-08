#include "IMU.h"

IMU::IMU() {
  Wire.begin();
  mpu.initialize();
  // Verify connection
  if (!mpu.testConnection()) {
    Serial.println("ERROR : MPU6050 IMU sensor disconected");
  }
}

Vector3 IMU::getGyros() {
  mpu.getRotation(&gx, &gy, &gz);
  Vector3 Vector(gx, gy, gz);
  return Vector;
}

Vector3 IMU::getAcceleration() {
  mpu.getAcceleration(&ax, &ay, &az);
  Vector3 Vector(ax, ay, az);
  return Vector;
}

Vector3 IMU::getOrientation() {
  
  // set up time for integration
  timePrev = time;
  time = millis();
  timeStep = (time - timePrev) / 1000; // time-step in s

  // collect readings
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // apply gyro scale from datasheet
  gsx = gx/gyroScale;   gsy = gy/gyroScale;   gsz = gz/gyroScale;

  // calculate accelerometer angles
  arx = (180/3.141592) * atan(ax / sqrt(square(ay) + square(az))); 
  ary = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
  arz = (180/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);

  // set initial values equal to accel values
  if (i == 1) {
    grx = arx;
    gry = ary;
    grz = arz;
  }
  // integrate to find the gyro angle
  else{
    grx = grx + (timeStep * gsx);
    gry = gry + (timeStep * gsy);
    grz = grz + (timeStep * gsz);
  }  

  // apply filter
  rx = (0.96 * arx) + (0.04 * grx);
  ry = (0.96 * ary) + (0.04 * gry);
  rz = (0.96 * arz) + (0.04 * grz);

  i = i + 1;
  Vector3 Vector(rx, ry, rz);
  return Vector;
}
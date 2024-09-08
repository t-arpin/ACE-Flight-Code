#include "telem.h"

Telem::Telem() {
  //add code
}

void Telem::logData(float flightTime, float gyroX, float gyroY, float gyroZ, float accelerationX, float accelerationY, float accelerationZ, float orientationX, float orientationY, float orientationZ, float temperature, float pressure, float altitude, float yServo, float zServo, float voltage, String state) {
  Serial.print(flightTime);
  Serial.print(",");
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.print(gyroZ);
  Serial.print(",");
  Serial.print(accelerationX);
  Serial.print(",");
  Serial.print(accelerationY);
  Serial.print(",");
  Serial.print(accelerationZ);
  Serial.print(",");
  Serial.print(orientationX);
  Serial.print(",");
  Serial.print(orientationY);
  Serial.print(",");
  Serial.print(orientationZ);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.print(altitude);
  Serial.print(",");
  Serial.print(yServo);
  Serial.print(",");
  Serial.print(zServo);
  Serial.print(",");
  Serial.print(voltage);
  //Serial.print(",");
  //Serial.println(state);
}
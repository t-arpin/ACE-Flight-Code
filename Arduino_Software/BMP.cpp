#include "BMP.h"

BMP::BMP() {
  if (!bmp.begin()) {
    Serial.println("ERROR : BMP180 BARO sensor disconected");
  }
}

float BMP::readTemperature() {
  return bmp.readTemperature();
}

float BMP::readPressure() {
  return bmp.readPressure();
}

float BMP::readAltitude(float seaLevelhPa) {
  return bmp.readAltitude(seaLevelhPa * 100);
}

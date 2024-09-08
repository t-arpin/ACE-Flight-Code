/*
 Barometric Pressure & Altitude Sensor BMP180
 
 Outputs the following data: temperature, athmospheric pressure and altitude (which is calculated with the pressure at sea level)
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>

class BMP {
  public:
    BMP();
    
    float readTemperature();
    float readPressure();
    float readAltitude(float seaLevelhPa);

  private:
    Adafruit_BMP085 bmp;
};
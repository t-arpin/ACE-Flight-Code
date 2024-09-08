/*
 * Telemetry controler
 * 
 * Logs and transmits important data
 */

#include <SD.h>
#include <SPI.h>

class Telem {
  public:    
    Telem();

    void logData(float flightTime, float gyroX, float gyroY, float gyroZ, float accelerationX, float accelerationY, float accelerationZ, float orientationX, float orientationY, float orientationZ, float temperature, float pressure, float altitude, float yServo, float zServo, float voltage, String state);
};
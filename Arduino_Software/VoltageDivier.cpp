#include "VoltageDivider.h"

float VoltageDivider::getBoardVoltage() {
  int voltageDividerValue = analogRead(A2);

  //        R1 + R2
  // Vin = --------- * Vout
  //          R2

  float voltage = ((2000 + 1000)/1000)*(voltageDividerValue*5.0 / 1023.0);
  return voltage;
}
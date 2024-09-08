/*
 * Voltage divider
 * 
 * Calculates the on-board voltage by using a voltage divider connected to an analoge input pin
 */

#include <Arduino.h>

#define VOLTAGEDIVIDER_PIN A2

class VoltageDivider {
  public:
    float getBoardVoltage();
};
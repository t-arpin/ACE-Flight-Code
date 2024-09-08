/*
 * Passive buzzer
 * 
 * Used to indicate states and errors via 
 */

#include <Arduino.h>

#define BUZZER_PIN 8

class Buzzer {
  public:
    Buzzer();

    void playTone(int tonehz, int duration);
    void startup();
    void critialError();
};
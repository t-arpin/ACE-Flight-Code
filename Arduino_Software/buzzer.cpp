#include "buzzer.h"

Buzzer::Buzzer() {
  
}

void Buzzer::playTone(int tonehz, int duration){
  tone(BUZZER_PIN, tonehz);
  delay(duration);
  noTone(BUZZER_PIN);
}

void Buzzer::startup() {
  int melody[] = {523, 659, 784, 880, 988, 784, 659, 523};  // Frequencies in Hertz
  int noteDurations[] = {150, 100, 200, 100, 150, 100, 150, 50};  // Durations in milliseconds
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);
    delay(noteDuration); // Add the duration as a delay
  }
  noTone(BUZZER_PIN); // Turn off buzzer
}

void Buzzer::critialError() {
  tone(BUZZER_PIN, 440);
}
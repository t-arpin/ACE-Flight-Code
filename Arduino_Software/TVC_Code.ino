#include "StateMachine.h"

StateMachine *stateMachine;

void setup() {
  Serial.begin(9600);
  stateMachine = new StateMachine();
}

void loop() {
  stateMachine->stateMachineLoop();
}

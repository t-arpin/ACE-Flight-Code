#include "StateMachine.h"

StateMachine::StateMachine() {
  activeState = LAUNCH_PAD_IDLE; // Default state
  Serial.println("LAUNCH_PAD_IDLE");
}

void StateMachine::stateMachineLoop() {
  switch (activeState) {
    case LAUNCH_PAD_IDLE:
      rocket.padIdle();
      // Detect lift off -> Ascent flight and activate TVC
      if (liftOffCheck()) {
        // Save the time of lift off
        rocket.flightStartTime = millis();
        activeState = ASCENT;
        Serial.println("ASCENT");
      }
      break;
    case ASCENT:
      // TCV activated
      rocket.ascent();
      // Detect Burnout -> Coast phase
      if (burnoutCheck()) {
        activeState = COAST;
        Serial.println("COAST");
      }
      break;
    case COAST:
      // TVC centered at 0 deg
      rocket.coast();
      // Detect max apogee -> Balistic descent and eject Parachute
      if (maxApogeeCheck()) {
        rocket.maxApogee();
        activeState = BALISTIC;
        Serial.println("BALISTIC");
      }
      break;
    case BALISTIC:
      rocket.balistic();
      // Log Data
      // Detect chutes
      if (chuteCheck()) {
        activeState = STABLE;
        Serial.println("STABLE");
      }
      break;
    case STABLE:
      rocket.stable();
      // Detect landing
      if (landedCheck()) {
        activeState = LANDED;
        Serial.println("LANDED");
      }
      break;
    case LANDED:
      // Do nothing
      break;
  }
}

// Check if rocket has launched by looking at the acceleration on vertical axis (x-axis)
bool StateMachine::liftOffCheck() {
  if (rocket.acceleration.y < LIFT_THRESHOLD) {
    return true;
  }
  return false;
}

// Check if rocket has reached burnout by comparing the burn time to flight time
bool StateMachine::burnoutCheck() {
  if (rocket.flightTime > BURN_TIME) {
    return true;
  }
  return false;
}

// Check if rocket has reached apogee by looking at the accelaration on vertical axis (x-axis) and barometric altitude
bool StateMachine::maxApogeeCheck() {
  const float ALTITUDE_CHANGE_THRESHOLD = -0.5;  // Threshold for altitude decrease (meters)
  const float ACCELERATION_THRESHOLD = -2.0;  // Threshold for vertical acceleration (m/s^2)
  const int STABLE_PERIOD = 5;  // Number of consecutive stable readings required

  static bool altitudeDecreasing = false;
  static bool accelerationIncreasing = false;
  static int descentCount = 0;

  if (rocket.flightTime <= APOGEE_THRESHOLD_MIN){
    return false;
  }
  if (rocket.flightTime >= APOGEE_THRESHOLD_MAX){
    return true;
  }
  
  // Check for descent phase
  if (rocket.altitude < rocket.previousAltitude + ALTITUDE_CHANGE_THRESHOLD) {
    altitudeDecreasing = true;
  } else {
    altitudeDecreasing = false;
  }
  
  // Check for increasing acceleration during descent
  if (altitudeDecreasing && rocket.acceleration.x > ACCELERATION_THRESHOLD) {
    accelerationIncreasing = true;
  } else {
    accelerationIncreasing = false;
  }
  
  // Count consecutive stable periods during descent
  if (altitudeDecreasing && accelerationIncreasing) {
    descentCount++;
  } else {
    descentCount = 0;
  }
  
  // Check if descent criteria are met
  if (descentCount >= STABLE_PERIOD) {
    return true;  // Apogee detected
  } else {
    return false;  // Not yet at apogee
  }
}

// Check if chute has been deployed
bool StateMachine::chuteCheck() {
  
  if (rocket.chute /* && rocket.velocity.x < CHUTE_THRESHOLD*/) {
    return true;
  }
  return false;
}

// Check if rocket has landed by looking at the time it takes to land
bool StateMachine::landedCheck() {
  if (rocket.flightTime > LANDING_TIME) {
    return true;
  }
  return false;
}
/*
  -- Nominal Flight States --

  LAUNCH_PAD_IDLE
    Check for acceleration of the rocket on vertical axis 
  ASCENT
    TVC loop is active
  BURNOUT/COAST
    Decelaration detected
    TVC disabled and locked to 0 deg
  BALISTIC DESCENT
    Negative acceleration
    Deploy chute at set altitude or time
  STABLE DESCENT
    Chute deployed
    Within safe recovery velocity
  LANDED
    No movement on IMU
    No active code (computer waiting to be turned off)

  -- Emergency Flight States --
  TOO_SHARP_ANGLE
    Activate if angle is to sharp
    TVC disabled and locked to 0 deg
    Tries to fire chute when declared safe
  TERRAIN
    Activitate if Altitude less than set threashold during Balistic descent
    Fires remaining pyro charges
*/

#include <Arduino.h>
#include <Wire.h>
#include "Rocket.h"

#define LIFT_THRESHOLD -5           // Acceleration on vertical axis/x-axis (negative value = acceleration upwards) (in m/s^2)
#define BURN_TIME 3.7               // Estimated motor burn time
#define APOGEE_THRESHOLD_MIN 8.0    // Start of apogee detect window
#define APOGEE_THRESHOLD_MAX 10.0   // End of apogee detect window
#define CHUTE_THRESHOLD 5
#define LANDING_TIME  20            // Estimated time to landing

enum State {
  LAUNCH_PAD_IDLE,
  ASCENT,
  COAST,
  BALISTIC,
  STABLE,
  LANDED
};

enum EmgyState {
  TOO_SHARP_ANGLE,
  TERRAIN
};

class StateMachine {
  public:
    StateMachine();
    void stateMachineLoop();
    bool liftOffCheck();
    bool burnoutCheck();
    bool maxApogeeCheck();
    bool chuteCheck();
    bool landedCheck();

  private:
    Rocket rocket;
    State activeState;
};

/*
  Rocket code file
  
  Contains functions that will be run in a specific state
  Functions are called in StateMachine.cpp
  Header file contains all constants that are specific to the model rocket
*/

#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
#include "PID.h"
#include "buzzer.h"
#include "telem.h"
#include "VoltageDivider.h"
#include "kalman.h"
#include "IMU.h"
#include "BMP.h"

#define MASS 0.65                   // Mass of the vehicle in kg

#define Y_SERVO_PIN 10              // Y Servo (y axis labeled on IMU) = Y Servo (labeled on pcb)
#define Z_SERVO_PIN 9               // Z Servo (z axis labeled on IMU) = X Servo (labeled on pcb)

#define Y_SERVO_MIDDLE 78           // Y Servo horn vertical position (motor mount is vertical)
#define Z_SERVO_MIDDLE 87           // Z Servo horn vertical position (motor mount is vertical)

#define LED 13                      // LED Pin

// Servo ratio is calculated by making multiple images of different servo horn angles and comparing the orientation of the horn and the motor mount
#define SERVO_RATIO 3.1             // Servo horn to motor mount ratio (if horn is turned 1 deg, motor mount will turn 3 deg)
#define MAX_SERVO_ROTATION 30       // Motor mount can only be rotated 10 deg => the servos are allowed to rotate 30 deg

#define DEG2RAD 0.01745329251f      // Convert degrees to radians by multiplying with this number
#define RAD2DEG 57.2957795131f      // Convert radians to degrees by multiplying with this number

#define MIN_VOLTAGE 3               // Minimum board voltage to complete startup (in V)

#define SEA_LEVEL_PRESSURE 1013.25  // Standard atmospheric pressure at sea level (in hPa)

class Rocket {
  public:
    Rocket();

    // Angular Velocity Vector
    // Three axis of rotation speed in radians per second (rad/s)
    Vector3 gyros;

    // Acceleration Vector
    // Three axis of acceleration (gravity + linear motion) in m/s^2
    Vector3 acceleration;

    // Absolute Orientation Vector
    // Three axis orientation data based on a 360° sphere (Euler Vector in deg)
    Vector3 orientation;

    // Relative Orientation Vector
    // Three axis orientation data (roll, pitch and yaw) in rad
    Vector3 relativeOrientation;

    float flightTime = 0.0f; // in s
    float flightStartTime = 0.0f; // in ms
    float altitude;
    float previousAltitude;
    bool chute = false;
    
    void padIdle();
    void ascent();
    void coast();
    void maxApogee();
    void balistic();
    void stable();
    void landed();

  private:
    VoltageDivider voltageDivider;
    Buzzer buzzer;
    Telem telem;
    IMU imu;
    BMP bmp;
    
    Kalman kalmanAcc(0.01, 0.1, 0.1, Vector3());
    Kalman kalmanGyro(0.01, 0.1, 0.1, Vector3());
    Kalman kalmanOri(0.01, 0.1, 0.1, Vector3());
    Kalman kalmanAlt(0.01, 0.1, 0.1, Vector3(0, 0, 0));
    
    // PID Values were tuned with a Simulink simulation and verified with AeroVECTOR https://github.com/GuidodiPasquo/AeroVECTOR
    // PID-Controller for calculation of motor mount orientation on y-axis
    PID yPID = PID(0.6f, 0.05f, 0.125f, 10.0f);
    // PID-Controller for calculation of motor mount orientation on z-axis
    PID zPID = PID(0.6f, 0.05f, 0.125f, 10.0f);

    Servo yServo;
    Servo zServo;
    
    unsigned long currentTime = 0.0;
    unsigned long previousTime = 0.0;
    float deltaTime = 0.0f;
};
#include "Rocket.h"

Rocket::Rocket() {
  // Startup melody
  buzzer.startup();

  // Attach and initialize servos
  yServo.attach(Y_SERVO_PIN);
  zServo.attach(Z_SERVO_PIN);

  yServo.write(Y_SERVO_MIDDLE);
  zServo.write(Z_SERVO_MIDDLE);

  kalmanAcc.KalmanFilter(0.01f, 0.1f, 0.1f, Vector3());
  kalmanGyro.KalmanFilter(0.01f, 0.1f, 0.1f, Vector3());
  kalmanOri.KalmanFilter(0.01f, 0.1f, 0.1f, Vector3());
  kalmanAlt.KalmanFilter(0.01f, 0.1f, 0.1f, Vector3(0, 0, 0));

  // Check on-board voltage
  if(voltageDivider.getBoardVoltage() < MIN_VOLTAGE) {
    Serial.println("Error: on-board voltage too low");
    while (true) {
      // on-board voltage too low, so do not do anything more - stay stuck here and play alarm
      buzzer.critialError();
      
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
    }
  }
}

void Rocket::padIdle() {
  acceleration = kalmanAcc.update(imu.getAcceleration());
}

void Rocket::ascent() {
  flightTime = (millis() - flightStartTime) / 1000.0f;

  // ----- TVC ALGORITHM -----
  float deltaTime = (micros() - previousTime) / 1000000.0f;
  Vector3 gyros = kalmanGyro.update(imu.getGyros());
  relativeOrientation += gyros * deltaTime;

  float yPIDoutput = yPID.update(relativeOrientation.y) * RAD2DEG;
  float zPIDoutput = zPID.update(relativeOrientation.z) * RAD2DEG;

  yPIDoutput = constrain(yPIDoutput * SERVO_RATIO, -MAX_SERVO_ROTATION, MAX_SERVO_ROTATION);
  zPIDoutput = constrain(zPIDoutput * SERVO_RATIO, -MAX_SERVO_ROTATION, MAX_SERVO_ROTATION);

  yServo.write(yPIDoutput + Y_SERVO_MIDDLE);
  zServo.write(zPIDoutput + Z_SERVO_MIDDLE);

  previousTime = micros();

  logData("ASCENT");
}

void Rocket::coast() {
  flightTime = (millis() - flightStartTime) / 1000.0f;

  yServo.write(Y_SERVO_MIDDLE);
  zServo.write(Z_SERVO_MIDDLE);

  logData("DESCENT");
}

void Rocket::maxApogee() {
  chute = true; // simulate chute deploy
}

void Rocket::balistic() {
  flightTime = (millis() - flightStartTime) / 1000.0f;

  logData("DESCENT");
}

void Rocket::stable() {
  flightTime = (millis() - flightStartTime) / 1000.0f;

  logData("DESCENT");
}

void Rocket::logData(const char* phase) {
  Vector3 gyros = kalmanGyro.update(imu.getGyros());
  acceleration = kalmanAcc.update(imu.getAcceleration());
  Vector3 orientation = kalmanOri.update(imu.getOrientation());

  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  Vector3 altitudeVector = kalmanAlt.update(Vector3(bmp.readAltitude(SEA_LEVEL_PRESSURE), 0, 0));
  float altitude = altitudeVector.x;
  float voltage = voltageDivider.getBoardVoltage();
  
  telem.logData(flightTime, gyros.x, gyros.y, gyros.z, acceleration.x, acceleration.y, acceleration.z, orientation.x, orientation.y, orientation.z, temperature, pressure, altitude, yServo.read(), zServo.read(), voltage, phase);
}
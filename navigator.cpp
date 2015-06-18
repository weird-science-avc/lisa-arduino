#include "navigator.h"

Navigator::Navigator(int logLevel) {
  this->logLevel = logLevel;
}

void Navigator::attachSpeedServo(int pin) {
  this->speedServo.attach(pin);
  // Initialize motor
  this->speedServo.writeMicroseconds(0);
}

void Navigator::attachSteeringServo(int pin) {
  this->steeringServo.attach(pin);
  this->steeringServo.writeMicroseconds(STEERING_CENTER_SERVO);
}

SPEED Navigator::getSpeed() {
  return this->speed;
}

STEERING Navigator::getSteering() {
  return this->steering;
}

void Navigator::reset() {
}

void Navigator::fullStop() {
  setSpeed(SPEED_STOPPED);
}

bool Navigator::update(Position p, Waypoint w) {
  Vector v = getVector(p.x, p.y, w.x, w.y);

  // Now we should make adjustments to get to the waypoint we're aiming at
  SPEED oldSpeed = this->speed;
  STEERING oldSteering = this->steering;
  adjustSpeed(v.d);
  adjustSteering(p.r, v.r);

  // Output what we're doing if debug
  if (this->logLevel >= LOG_LEVEL_VERBOSE || oldSpeed != speed || oldSteering != steering) {
    if (this->logLevel >= LOG_LEVEL_DEBUG) {
      Serial.print("NAVIGATION: dist=");
      Serial.print(v.d);
      Serial.print(",dir=");
      Serial.print(v.r * 180.0 / PI);
      Serial.print(", SPEED: ");
      Serial.print(this->speed == SPEED_STOPPED ? "stopped" : this->speed == SPEED_LOW ? "low" : "high");
      Serial.print(", STEERING: ");
      Serial.println(this->steering == STEERING_LEFT ? "left" : this->steering == STEERING_CENTER ? "center" : "right");
    }
    return true;
  }
  return false;
}

void Navigator::adjustSpeed(float distance) {
  SPEED newSpeed = (distance > APPROACH_DELTA) ? SPEED_HIGH : SPEED_LOW;
  newSpeed = SPEED_HIGH;
  setSpeed(newSpeed);
}

void Navigator::setSpeed(SPEED s) {
  int servoValue = SPEED_STOPPED_SERVO;
  if (s == SPEED_LOW) {
    servoValue = SPEED_LOW_SERVO;
  } else if (s ==  SPEED_HIGH) {
    servoValue = SPEED_HIGH_SERVO;
  }

  //if (LOG_NAVIGATION_DEBUG) {
  //  Serial.print("SPEED SERVO: ");
  //  Serial.println(servoValue);
  //}
  this->speedServo.writeMicroseconds(servoValue);
  this->speed = s;
}

void Navigator::adjustSteering(float orientation, float targetOrientation) {
  float delta = targetOrientation - orientation;
  // Get delta between [-pi,pi]
  delta = normalizeRadians(delta);
  STEERING newSteering;
  if (abs(delta) < ORIENTATION_DELTA) {
    newSteering = STEERING_CENTER;
  } else if (delta < 0) {
    newSteering = STEERING_RIGHT;
  } else {
    newSteering = STEERING_LEFT;
  }

  setSteering(newSteering);
}

void Navigator::setSteering(STEERING s) {
  int servoValue = STEERING_CENTER_SERVO;
  if (s == STEERING_LEFT) {
    servoValue = STEERING_LEFT_SERVO;
  } else if (s == STEERING_RIGHT) {
    servoValue = STEERING_RIGHT_SERVO;
  }

  //if (LOG_NAVIGATION_DEBUG) {
  //  Serial.print("STEERING SERVO: ");
  //  Serial.println(servoValue);
  //}
  this->steeringServo.writeMicroseconds(servoValue);
  this->steering = s;
}

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
      serialPrintPosition("NAVIGATION: p=", p);
      Serial.print(", dist=");
      Serial.print(v.d);
      Serial.print(", dir=");
      Serial.print(v.r * 180.0 / PI);
      Serial.print(", SPEED: ");
      Serial.print(this->speed == SPEED_STOPPED ? "stopped" : this->speed == SPEED_LOW ? "low" : "high");
      Serial.print(", STEERING: ");
      Serial.println(this->steering == STEERING_LEFT_FULL ? "left-full" :
        this->steering == STEERING_LEFT_HALF ? "left-half" :
        this->steering == STEERING_CENTER ? "center" :
        this->steering == STEERING_RIGHT_HALF ? "right-half" : "right-full"
      );
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

  // Protect always
  servoValue = int(max(min(SPEED_MAX_SERVO, servoValue), SPEED_MIN_SERVO));
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

  // Discrete steering
  //STEERING newSteering;
  //if (abs(delta) < ORIENTATION_DELTA) {
  //  newSteering = STEERING_CENTER;
  //} else if (delta < -25.0 * PI / 180.0) {
  //  newSteering = STEERING_RIGHT_FULL;
  //} else if (delta > 25.0 * PI / 180.0) {
  //  newSteering = STEERING_LEFT_FULL;
  //} else if (delta < 0.0) {
  //  newSteering = STEERING_RIGHT_HALF;
  //} else {
  //  newSteering = STEERING_LEFT_HALF;
  //}
  //setSteering(newSteering);

  // Continuous steering
  float newSteering;
  if (abs(delta) < ORIENTATION_DELTA) { //.017
    Serial.println("STEERING Center: ");
    newSteering = STEERING_CENTER_SERVO;
  } else if (delta <= - ORIENTATION_DELTA && delta >= -MAX_STEERING_THRESHOLD) {
    newSteering = STEERING_CENTER_SERVO - (-delta/MAX_STEERING_THRESHOLD) * STEERING_RIGHT_FULL_SERVO_RANGE;
  } else if (delta < -MAX_STEERING_THRESHOLD) {
    newSteering = STEERING_RIGHT_FULL_SERVO;
  } else if (delta >=  ORIENTATION_DELTA && delta <= MAX_STEERING_THRESHOLD) {
     newSteering = (delta/MAX_STEERING_THRESHOLD) * STEERING_LEFT_FULL_SERVO_RANGE + STEERING_CENTER_SERVO;
  } else if (delta > MAX_STEERING_THRESHOLD) {
    newSteering = STEERING_LEFT_FULL_SERVO;
  }
  setSteering(newSteering);
}

void Navigator::setSteering(float servoValue) {
  // Discrete steering
  //int servoValue = STEERING_CENTER_SERVO;
  //if (s == STEERING_LEFT_FULL) {
  //  servoValue = STEERING_LEFT_FULL_SERVO;
  //} else if (s == STEERING_LEFT_HALF) {
  //  servoValue = STEERING_LEFT_HALF_SERVO;
  //} else if (s == STEERING_RIGHT_HALF) {
  //  servoValue = STEERING_RIGHT_HALF_SERVO;
  //} else if (s == STEERING_RIGHT_FULL) {
  //  servoValue = STEERING_RIGHT_FULL_SERVO;
  //}

  // Protect always
  servoValue = int(max(min(STEERING_MAX_SERVO, servoValue), STEERING_MIN_SERVO));
  //if (this->logLevel >= LOG_LEVEL_DEBUG) {
  //  Serial.print("STEERING SERVO: ");
  //  Serial.println(servoValue);
  //}
  this->steeringServo.writeMicroseconds(servoValue);
  // FIXME: Save speed and feedback into mocking
  //this->steering = servoValue;
}

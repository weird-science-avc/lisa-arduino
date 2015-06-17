#include "navigator.h"

Navigator::Navigator(int ledPin) {
  this->ledPin = ledPin;
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

float Navigator::getVelocity() {
  // TODO: Change to switch if supported
  if (this->speed == SPEED_STOPPED) {
    return 0.0;
  } else if (this->speed == SPEED_LOW) {
    return SPEED_LOW_VELOCITY;
  } else { // SPEED_HIGH
    return SPEED_HIGH_VELOCITY;
  }
}

float Navigator::getTurnRadius() {
  // TODO: Change to switch if supported
  if (this->steering == STEERING_CENTER) {
    return NAN;
  } else if (this->steering == STEERING_LEFT) {
    return STEERING_LEFT_TURN_RADIUS;
  } else { // STEERING_RIGHT
    return STEERING_RIGHT_TURN_RADIUS;
  }
}

void Navigator::start(Waypoint* waypoints, int waypointsLength) {
  // Save our waypoints and set our index
  this->waypoints = waypoints;
  this->waypointsLength = waypointsLength;
  this->waypointIndex = 0;
  Serial.print("*** STARTING NAVIGATION -- ");
  Serial.print(this->waypointsLength);
  Serial.println(" waypoints ***");
  serialPrintWaypoint("START WAYPOINT: ", this->waypointIndex, this->waypoints[waypointIndex]);
  this->running = true;
}

bool Navigator::isRunning() {
  return this->running;
}

void Navigator::update(Position p) {
  // Get our current waypoint and calculate a vector to it
  Waypoint waypoint = this->waypoints[this->waypointIndex];
  Vector waypointVector = getVector(p.x, p.y, waypoint.x, waypoint.y);

  // If there's a next waypoint, think about promoting it
  if (this->waypointIndex + 1 < this->waypointsLength) {
    Waypoint nextWaypoint = waypoints[this->waypointIndex + 1];
    Vector nextWaypointVector = getVector(p.x, p.y, nextWaypoint.x, nextWaypoint.y);
    Vector waypointToWaypointVector = getVector(waypoint.x, waypoint.y, nextWaypoint.x, nextWaypoint.y);
    // If the distance to our next waypoint is less than the distance between them, we've passed by so promote
    // TODO: Phil suggested we could actual remove current waypoint's tolerance from our distance to next as well and it'd be okay
    if (nextWaypointVector.d < waypointToWaypointVector.d) {
      // TODO: Calculate time to finish waypoint
      serialPrintWaypoint("FINISH WAYPOINT: ", this->waypointIndex, waypoint);
      this->waypointIndex++;
      waypoint = nextWaypoint;
      waypointVector = nextWaypointVector;
      serialPrintWaypoint("START WAYPOINT: ", this->waypointIndex, waypoint);
    }
  }

  // Now whatever we're aiming for, see if we're there
  if (haveArrived(waypoint, waypointVector.d)) {
    // TODO: Calculate time to finish waypoint
    serialPrintWaypoint("FINISH WAYPOINT: ", this->waypointIndex, waypoint);

    // Move to next waypoint, detect end
    this->waypointIndex++;
    if (this->waypointIndex >= this->waypointsLength) {
      // TODO: Calculate time to finish navigation
      Serial.println("**** ENDING NAVIGATION ****");
      setSpeed(SPEED_STOPPED);
      this->running = false;
      return;
    }

    // Not end, so update waypoint pointers
    waypoint = this->waypoints[this->waypointIndex];
    waypointVector = getVector(p.x, p.y, waypoint.x, waypoint.y);
    serialPrintWaypoint("START WAYPOINT: ", this->waypointIndex, waypoint);
  }

  // Now we should make adjustments to get to the waypoint we're aiming at
  adjustSpeed(waypointVector.d);
  adjustSteering(p.r, waypointVector.r);

  // Output what we're doing
  // TODO: Control with LOG_LEVEL system
  //Serial.print("waypointVector: dist=");
  //Serial.print(waypointVector.d);
  //Serial.print(",dir=");
  //Serial.print(waypointVector.r * 180.0 / PI);
  //Serial.print(", SPEED: ");
  //Serial.print(this->speed == SPEED_STOPPED ? "stopped" : this->speed == SPEED_LOW ? "low" : "high");
  //Serial.print(", STEERING: ");
  //Serial.println(this->steering == STEERING_LEFT ? "left" : this->steering == STEERING_CENTER ? "center" : "right");
}

void Navigator::adjustSpeed(float distance) {
  SPEED newSpeed = (distance > APPROACH_DELTA) ? SPEED_HIGH : SPEED_LOW;
  setSpeed(newSpeed);
}

void Navigator::setSpeed(SPEED s) {
  int servoValue = SPEED_STOPPED_SERVO;
  if (s == SPEED_LOW) {
    servoValue = SPEED_LOW_SERVO;
  } else if (s ==  SPEED_HIGH) {
    servoValue = SPEED_HIGH_SERVO;
  }

  // TODO: Control with LOG_LEVEL system
  //Serial.print("SPEED SERVO: ");
  //Serial.println(servoValue);
  this->speedServo.writeMicroseconds(servoValue);
  this->speed = s;
}

void Navigator::adjustSteering(float orientation, float targetOrientation) {
  float delta = targetOrientation - orientation;
  // Get smallest delta by reducing/increasing outside [-pi,pi] by 2pi
  delta += (delta > PI) ? -2 * PI : (delta < -PI) ? 2 * PI : 0;
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

  // TODO: Control with LOG_LEVEL system
  //Serial.print("STEERING SERVO: ");
  //Serial.println(servoValue);
  this->steeringServo.writeMicroseconds(servoValue);
  this->steering = s;
}

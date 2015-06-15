#include "navigator.h"

Navigator::Navigator(int ledPin) {
  ledPin = ledPin;
}

void Navigator::attachSpeedServo(int pin) {
  speedServo.attach(pin);
  // Initialize motor
  speedServo.writeMicroseconds(0);
}

void Navigator::attachSteeringServo(int pin) {
  steeringServo.attach(pin);
  steeringServo.writeMicroseconds(STEERING_CENTER_SERVO);
}

float Navigator::getVelocity() {
  // TODO: Change to switch if supported
  if (speed == SPEED_STOPPED) {
    return 0.0;
  } else if (speed == SPEED_LOW) {
    return SPEED_LOW_VELOCITY;
  } else { // SPEED_HIGH
    return SPEED_HIGH_VELOCITY;
  }
}

float Navigator::getTurnRadius() {
  // TODO: Change to switch if supported
  if (steering == STEERING_CENTER) {
    return NAN;
  } else if (steering == STEERING_LEFT) {
    return STEERING_LEFT_TURN_RADIUS;
  } else { // STEERING_RIGHT
    return STEERING_RIGHT_TURN_RADIUS;
  }
}

void Navigator::start(Waypoint waypoints[]) {
  // Save our waypoints and set our index
  waypoints = waypoints;
  waypointIndex = 0;
  Serial.println("*** STARTING NAVIGATION ***");
  serialPrintWaypoint("START WAYPOINT: ", waypointIndex, waypoints[waypointIndex]);
  running = true;
}

bool Navigator::isRunning() {
  return running;
}

void Navigator::update(Position p) {
  // Get our current waypoint and calculate a vector to it
  Waypoint waypoint = waypoints[waypointIndex];
  Vector waypointVector = getVector(p.x, p.y, waypoint.x, waypoint.y);

  // If there's a next waypoint, think about promoting it
  int waypointsLength = sizeof(waypoints) / sizeof(Waypoint);
  if (waypointIndex + 1 < waypointsLength) {
    Waypoint nextWaypoint = waypoints[waypointIndex + 1];
    Vector nextWaypointVector = getVector(p.x, p.y, nextWaypoint.x, nextWaypoint.y);
    Vector waypointToWaypointVector = getVector(waypoint.x, waypoint.y, nextWaypoint.x, nextWaypoint.y);
    // If the distance to our next waypoint is less than the distance between them, we've passed by so promote
    // TODO: Phil suggested we could actual remove current waypoint's tolerance from our distance to next as well and it'd be okay
    if (nextWaypointVector.d < waypointToWaypointVector.d) {
      serialPrintWaypoint("FINISH WAYPOINT: ", waypointIndex, waypoint);
      waypointIndex++;
      waypoint = nextWaypoint;
      waypointVector = nextWaypointVector;
      serialPrintWaypoint("START WAYPOINT: ", waypointIndex, waypoint);
    }
  }

  // We didn't promote, so if we're at our waypoint, let's call it quits
  if (haveArrived(waypoint, waypointVector.d)) {
    Serial.println("**** ENDING NAVIGATION ****");
    setSpeed(SPEED_STOPPED);
    running = false;
    return;
  }

  // Now we should make adjustments to get to the waypoint we're aiming at
  adjustSpeed(waypointVector.d);
  adjustSteering(p.r, waypointVector.r);

  // Output what we're doing
  Serial.print("waypointVector: dist=");
  Serial.print(waypointVector.d);
  Serial.print(",dir=");
  Serial.print(waypointVector.r * 180.0 / PI);
  Serial.print(", SPEED: ");
  Serial.print(speed == SPEED_STOPPED ? "stopped" : speed == SPEED_LOW ? "low" : "high");
  Serial.print(" m/s, STEERING: ");
  Serial.println(steering == STEERING_LEFT ? "left" : steering == STEERING_CENTER ? "center" : "right");
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

  Serial.print("SPEED SERVO: ");
  Serial.println(servoValue);
  speedServo.writeMicroseconds(servoValue);
  speed = s;
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

  Serial.print("STEERING SERVO: ");
  Serial.println(servoValue);
  steeringServo.writeMicroseconds(servoValue);
  steering = s;
}

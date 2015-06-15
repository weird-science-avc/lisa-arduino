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
  return velocity;
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
  Vector waypointVector = getVector(p, waypoint);

  // Check with our new position if we've arrived and if so move to the next waypoint
  // TODO: Let's make this smarter so that we have a nextWaypoint too and decide to sometimes just move forward
  if (haveArrived(waypoint, waypointVector.d)) {
    serialPrintWaypoint("FINISH WAYPOINT: ", waypointIndex, waypoint);

    // Move to the next waypoint
    waypointIndex++;

    // If we'd done, just boogie out
    if (waypointIndex >= sizeof(waypoints)/sizeof(Waypoint)) {
      Serial.println("**** ENDING NAVIGATION ****");
      // TODO: Can we break here? Or maybe we should go way past it?
      setVelocity(0.0);
      running = false;
      return;
    }

    // Not done, so get a new waypoint and waypointVector
    waypoint = waypoints[waypointIndex];
    waypointVector = getVector(p, waypoint);
    serialPrintWaypoint("START WAYPOINT: ", waypointIndex, waypoint);
  }

  // Now we should make adjustments to get to the waypoint we're aiming at
  adjustVelocity(waypointVector.d);
  adjustSteering(p.r, waypointVector.r);

  // Output what we're doing
  Serial.print("waypointVector: dist=");
  Serial.print(waypointVector.d);
  Serial.print(",dir=");
  Serial.print(waypointVector.r * 180.0 / PI);
  Serial.print(", VELOCITY: ");
  Serial.print(velocity);
  Serial.print(" m/s, STEERING: ");
  Serial.println(steering == STEERING_LEFT ? "left" : steering == STEERING_CENTER ? "center" : "right");
}

void Navigator::adjustVelocity(float distance) {
  // TODO: Instead of this just go slow/fast depending on where we are
  float targetVelocity = distance / APPROACH_DELTA * MAX_VELOCITY;
  float newVelocity = min(max(targetVelocity, MIN_VELOCITY), MAX_VELOCITY);
  setVelocity(newVelocity);
}

void Navigator::setVelocity(float v) {
  if (v < MIN_VELOCITY) {
    speedServo.writeMicroseconds(0);
  } else {
    v = min(v, MAX_VELOCITY);
    // TODO: Move these values into speed_calibration so they can be modified
    int servoValue = int(v * 95.964 + 659.19);
    // Double safeguard on servo value right before write
    servoValue = min(max(servoValue, MIN_SPEED_SERVO), MAX_SPEED_SERVO);
    Serial.print("SPEED SERVO: ");
    Serial.println(servoValue);
    speedServo.writeMicroseconds(servoValue);
    velocity = v;
  }
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

  // Double safeguard on servo value right before write
  servoValue = min(max(servoValue, STEERING_MIN_SERVO), STEERING_MAX_SERVO);
  Serial.print("STEERING SERVO: ");
  Serial.println(servoValue);
  steeringServo.writeMicroseconds(servoValue);
  steering = s;
}

#include "navigator.h"

void serialPrintPosition(char* prefix, Position p) {
  Serial.print(prefix);
  Serial.print("(");
  Serial.print(p.x);
  Serial.print(",");
  Serial.print(p.y);
  Serial.print("):");
  Serial.println(p.r);
}

void serialPrintLocation(char* prefix, Location l) {
  Serial.print(prefix);
  Serial.print("(");
  Serial.print(l.x);
  Serial.print(",");
  Serial.print(l.y);
  Serial.println(")");
}

Navigator::Navigator(Servo speedServo, Servo steeringServo) {
  speedServo = speedServo;
  steeringServo = steeringServo;
}

void Navigator::NavigateTo(Position start, Location target) {
  Position position = start;
  float velocity = 0;
  int steering = 0;
  Vector targetVector;

  serialPrintPosition("START: ", position);
  serialPrintLocation("END: ", target);
  int timeElapsedMs = 0;
  int timestamp = millis();
  while (true) {
    // Get the direct vector to our target from where we are
    targetVector = vectorToTarget(position, target);
    Serial.print("targetVector: ");
    Serial.print(targetVector.d);
    Serial.print(",");
    Serial.println(targetVector.r);

    // If we're close enough, then set thelet people know and exit
    if (haveArrived(targetVector.d)) {
      Serial.println("GOAL ACHIEVED!!!");
      // Fully stop since we're at our final location
      setVelocity(0.0);
      return;
    }

    // We're not close enough, so let's adjust velocity and steering if necessary
    velocity = calculateNewVelocity(velocity, targetVector.d);
    steering = calculateNewSteering(steering, targetVector.r - position.r);
    setVelocity(velocity);
    setSteering(steering);

    // Print out decisions
    serialPrintPosition("POSITION: ", position);
    Serial.print("VELOCITY: ");
    Serial.print(velocity);
    Serial.println(" m/s");
    Serial.print("STEERING: ");
    Serial.println(steering);

    // Now wait 100ms, then calculate a new position and repeate
    delay(100);

    // Track timestamps so we know actual elapsed time
    int newTimestamp = millis();
    timeElapsedMs = newTimestamp - timestamp;
    timestamp = newTimestamp;

    // Get the distance traveled and turnRadius from converstions for now
    float distance = velocity * timeElapsedMs / 1000.0; // m
    float turnRadius = turnRadiusFromSteering(steering); // m

    // Use that information to calculate our new position before we kick off a new loop
    position = calculateNewPosition(position, distance, turnRadius);
  }
}

Vector Navigator::vectorToTarget(Position p, Location t) {
  float deltaY = p.y - t.y;
  float deltaX = p.x - t.x;
  Vector v;
  v.d = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  v.r = atan2(deltaY, deltaX);
  return v;
}

bool Navigator::haveArrived(float distance) {
  return distance < ARRIVE_DELTA;
}

// turnRadius of 0.0 or NAN means straight
Position Navigator::calculateNewPosition(Position position, float distance, float turnRadius) {
  float xDelta = 0.0;
  float yDelta = 0.0;
  float rDelta = 0.0;

  // Figure out position deltas based on straight or banked travel
  if (turnRadius == NAN || turnRadius == 0.0) {
    // Straight motion
    xDelta = distance * cos(position.r);
    yDelta = distance * sin(position.r);
  } else {
    // Banked motion
    // Get the rDelta
    rDelta = distance / turnRadius;
    // Calculate x and y deltas as if we were at (0,0) pointed along x-axis
    // NOTE: x is 'up' for us, so that's governed by Sin
    float xDeltaOrigin = turnRadius * sin(rDelta);
    float yDeltaOrigin = turnRadius * (-cos(rDelta) + 1.0);

    // Now we need to rotate those deltas around (0,0) by our current orientation so they're correct
    float sinR = sin(position.r);
    float cosR = cos(position.r);
    xDelta = xDeltaOrigin * cosR - yDeltaOrigin * sinR;
    yDelta = xDeltaOrigin * sinR + yDeltaOrigin * cosR;
  }

  return Position{position.x + xDelta, position.y + yDelta, position.r + rDelta};
}

float Navigator::calculateNewVelocity(float velocity, float distance) {
  float targetVelocity = distance / APPROACH_DELTA * MAX_VELOCITY;
  float newVelocity = min(max(targetVelocity, MIN_VELOCITY), MAX_VELOCITY);
  // TODO: Consider enforcing a maximum change
  return newVelocity;
}

// s = 0 => servoValue == 0
// s = MIN_VELOCITY to MAX_VELOCITY according to calibration function
void Navigator::setVelocity(float velocity) {
  if (velocity < MIN_VELOCITY) {
    speedServo.writeMicroseconds(0);
  } else {
    velocity = min(velocity, MAX_VELOCITY);
    int servoValue = int(velocity * 95.964 + 659.19);
    // Double safeguard on servo value right before write
    servoValue = min(max(servoValue, MIN_SPEED_SERVO), MAX_SPEED_SERVO);
    Serial.print("SPEED SERVO: ");
    Serial.println(servoValue);
    speedServo.writeMicroseconds(servoValue);
  }
}

float Navigator::calculateNewSteering(float steering, float angle) {
  float targetSteering = angle * 180.0 / pi;
  float newSteering = min(max(targetSteering, -MAX_STEERING), MAX_STEERING);
  // TODO: Consider enforcing a maximum change
  return newSteering;
}

float Navigator::turnRadiusFromSteering(float steering) {
  float turnRadius = NAN;
  if (steering < 0) { // right
    steering = min(abs(steering), MAX_STEERING);
    // Calculate turnRadius from right linear equation
    turnRadius = -(RIGHT_M_TURN_RADIUS_FROM_STEERING * steering + RIGHT_B_TURN_RADIUS_FROM_STEERING);
  } else if (steering > 0) {
    steering = min(steering, MAX_STEERING);
    // Calculate servoValue from left curve
    turnRadius = LEFT_M_TURN_RADIUS_FROM_STEERING * steering + LEFT_B_TURN_RADIUS_FROM_STEERING;
  }
  return turnRadius;
}

// s = 0.0 => servoValue == CENTER_STEERING
// s > 0 (left) => according to left calibration function
// s < 0 (right) => according to right calibration function
void Navigator::setSteering(float steering) {
  int servoDelta = 0;
  if (steering < 0) { // right
    steering = min(abs(steering), MAX_STEERING);
    // Calculate servoValue from right curve
    servoDelta = -int(RIGHT_M_SERVO_DELTA_FROM_STEERING * steering);
  } else if (steering > 0) {
    steering = min(steering, MAX_STEERING);
    // Calculate servoValue from left curve
    servoDelta = int(LEFT_M_SERVO_DELTA_FROM_STEERING * steering);
  }
  int servoValue = int(CENTER_STEERING_SERVO) + servoDelta;

  // Double safeguard on servo value right before write
  servoValue = min(max(servoValue, MIN_STEERING_SERVO), MAX_STEERING_SERVO);
  Serial.print("STEERING SERVO: ");
  Serial.println(servoValue);
  steeringServo.writeMicroseconds(servoValue);
}

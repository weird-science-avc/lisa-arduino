#include "navigator.h"

void serialPrintPosition(char* prefix, Position p) {
  Serial.print(prefix);
  Serial.print("(");
  Serial.print(p.x);
  Serial.print(",");
  Serial.print(p.y);
  Serial.print("):");
  Serial.println(p.r * 180.0 / PI);
}

void serialPrintLocation(char* prefix, Location l) {
  Serial.print(prefix);
  Serial.print("(");
  Serial.print(l.x);
  Serial.print(",");
  Serial.print(l.y);
  Serial.println(")");
}

//Servo speedServo, steeringServo;

Navigator::Navigator() { //Servo speedServo, Servo steeringServo) {
  //speedServo = speedServo;
  //steeringServo = steeringServo;
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

void Navigator::resetSteering() {
  // Initialize steering
  steeringServo.writeMicroseconds(STEERING_MAX_SERVO);
  delay(1000);
  steeringServo.writeMicroseconds(STEERING_MIN_SERVO);
  delay(1000);
  steeringServo.writeMicroseconds(STEERING_CENTER_SERVO);
  delay(500);
}

void Navigator::NavigateTo(Position start, Location target) {
  Position position = start;
  float velocity = 0;
  STEERING steering = STEERING_CENTER;
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
    Serial.println(targetVector.r * 180.0 / PI);

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
    Serial.print("VELOCITY: ");
    Serial.print(velocity);
    Serial.println(" m/s");
    Serial.print("STEERING: ");
    Serial.println(steering == STEERING_LEFT ? "left" : steering == STEERING_CENTER ? "center" : "right");

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
    serialPrintPosition("POSITION: ", position);
  }
}

Vector Navigator::vectorToTarget(Position p, Location t) {
  float deltaY = t.y - p.y;
  float deltaX = t.x - p.x;
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
  if (isnan(turnRadius) || turnRadius == 0.0) {
    //Serial.println("Straight motion");
    xDelta = distance * cos(position.r);
    yDelta = distance * sin(position.r);
  } else {
    //Serial.println("Banked motion");
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

STEERING Navigator::calculateNewSteering(STEERING steering, float angle) {
  if (angle < ORIENTATION_DELTA) {
    return STEERING_CENTER;
  } else if (angle < 0) {
    return STEERING_RIGHT;
  } else {
    return STEERING_LEFT;
  }
}

float Navigator::turnRadiusFromSteering(STEERING steering) {
  // TODO: Change to switch if supported
  if (steering == STEERING_CENTER) {
    return NAN;
  } else if (steering == STEERING_LEFT) {
    return STEERING_LEFT_TURN_RADIUS;
  } else { // STEERING_RIGHT
    return STEERING_LEFT_TURN_RADIUS;
  }
}

void Navigator::setSteering(STEERING steering) {
  int servoValue = STEERING_CENTER_SERVO;
  if (steering == STEERING_LEFT) {
    servoValue = STEERING_LEFT_SERVO;
  } else if (steering == STEERING_RIGHT) {
    servoValue = STEERING_RIGHT_SERVO;
  }

  // Double safeguard on servo value right before write
  servoValue = min(max(servoValue, STEERING_MIN_SERVO), STEERING_MAX_SERVO);
  Serial.print("STEERING SERVO: ");
  Serial.println(servoValue);
  steeringServo.writeMicroseconds(servoValue);
}

#include <Servo.h>
#include "helpers.h"

// PIN ASSIGNMENTS
const int SPEED_SERVO_PIN = 9;
const int STEERING_SERVO_PIN = 10;
const int BUTTON_PIN = 12;
const int LED_PIN = 13;

// Servo controls for speed and steering
Servo speedServo, steeringServo;

void setup() {
  // Start serial at 9600 baud
  Serial.begin(9600);

  // Setup pins correctly
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Attach servos to pins
  speedServo.attach(SPEED_SERVO_PIN);
  steeringServo.attach(STEERING_SERVO_PIN);

  // Initialize to stopped
  speedServo.writeMicroseconds(0);
}

// This is NavigateTo function
void loop() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
    // Make sure our light is off, turn on to warn, turn off to work
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
    delay(3000);
    digitalWrite(LED_PIN, LOW);

    // Navigate to our location
    NavigateTo(Position{0.0, 0.0, 0.0}, Location{10.0, 0.0});

    // Let them know we made it
    digitalWrite(LED_PIN, HIGH);
  }
}

void NavigateTo(Position start, Location target) {
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

// TODO: Would like to move these out to separate C files but not working?

// CALIBRATION
const float MIN_SPEED_SERVO = 660; // slowest non-zero
const float MAX_SPEED_SERVO = 1080; // fastest non-zero
const float MIN_STEERING_SERVO = 1150; // all right, 66" turn radius
// 1379 - 116" turn radius right
const float MAX_STEERING_SERVO = 2120; // all left, 77"
// 1864 - 146" turn radius left
const float CENTER_STEERING_SERVO = 1608; //(MAX_STEERING_SERVO + MIN_STEERING_SERVO) / 2;

// INTERNAL REFERNCE
const float MIN_VELOCITY = 0.25; // m/s
const float MAX_VELOCITY = 2.0; // m/s
const float MAX_STEERING = 30; // degrees 2120(left)

const float ARRIVE_DELTA = 0.05;
const float APPROACH_DELTA = 1.0;

// TODO: Do we need this more accurate
const float pi = 3.14;

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

Vector vectorToTarget(Position p, Location t) {
  float deltaY = p.y - t.y;
  float deltaX = p.x - t.x;
  Vector v;
  v.d = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  v.r = atan2(deltaY, deltaX);
  return v;
}

bool haveArrived(float distance) {
  return distance < ARRIVE_DELTA;
}

// turnRadius of 0.0 or NAN means straight
Position calculateNewPosition(Position position, float distance, float turnRadius) {
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

float calculateNewVelocity(float velocity, float distance) {
  float targetVelocity = distance / APPROACH_DELTA * MAX_VELOCITY;
  float newVelocity = min(max(targetVelocity, MIN_VELOCITY), MAX_VELOCITY);
  // TODO: Consider enforcing a maximum change
  return newVelocity;
}

// s = 0 => servoValue == 0
// s = MIN_VELOCITY to MAX_VELOCITY according to calibration function
void setVelocity(float velocity) {
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

int calculateNewSteering(float steering, float angle) {
  float targetSteering = angle * 180.0 / pi;
  float newSteering = min(max(targetSteering, -MAX_STEERING), MAX_STEERING);
  // TODO: Consider enforcing a maximum change
  return int(newSteering);
}

//const float MIN_SPEED_SERVO = 660; // slowest non-zero
//const float MAX_SPEED_SERVO = 1080; // fastest non-zero
//const float MIN_STEERING_SERVO = 1150; // all right, 66" turn radius
//// 1379 - 116" turn radius right
//const float MAX_STEERING_SERVO = 2120; // all left, 77"
//// 1864 - 146" turn radius left
//const float CENTER_STEERING_SERVO = 1608; //(MAX_STEERING_SERVO + MIN_STEERING_SERVO) / 2;
//
//// Used for calculations
//const M_STEERING_RIGHT = (116.0 - 66.0) * 0.0254 / (1379.0 - 1150.0);  // ~ 0.005545
//const B_STEERING_RIGHT = 66.0 - M_STEERING_RIGHT * 1150;               // ~ 59.623
//const M_STEERING_LEFT = (146.0 - 77.0) * 0.0254 / (1864.0 - 2120.0);   // ~ -0.006846
//const B_STEERING_LEFT = 77.0 - M_STEERING_LEFT * 2120.0;               // ~ 62.486
//const RIGHT_MAX_STEERING_SERVO = ; // to match 77" turn radius and be '30' degree turn

float turnRadiusFromSteering(int steering) {
  return 0.0;
}


// s = 0 => servoValue == CENTER_STEERING
// s = 0 to MAX_STEERING (left) => CENTER_STEERING_SERVO to MAX_STEERING_SERVO (linear)
// s = 0 to -MAX_STEERING (right) => CENTER_STEERING_SERVO to MIN_STEERING_SERVO (linear)
void setSteering(int steering) {
  //steeringServo.writeMicroseconds(CENTER_STEERING_SERVO);
  steeringServo.writeMicroseconds(1864);
}


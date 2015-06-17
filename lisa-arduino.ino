#include <Servo.h>
#include "position_tracker.h"
#include "navigator.h"

// PIN ASSIGNMENTS
const int SPEED_SERVO_PIN = 9;
const int STEERING_SERVO_PIN = 10;
const int BUTTON_PIN = 12;
const int LED_PIN = 13;

// Create helper objects
PositionTracker tracker;
Navigator navigator(LED_PIN);


// Global tick counter for wheel encoder
volatile int gWheelEncoderTicks = 0;

void isrWheelEncoder() {
  // Increment
  gWheelEncoderTicks++;
}

// This is called once
void setup() {
  // Attach to interrupt 0 for wheel encoder
  attachInterrupt(0, isrWheelEncoder, CHANGE);

  // Start serial at 9600 baud
  Serial.begin(9600);

  // Setup pins correctly
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // PositionTracker initialization

  // Navigator initialization
  navigator.attachSpeedServo(SPEED_SERVO_PIN);
  navigator.attachSteeringServo(STEERING_SERVO_PIN);
}

Waypoint waypoints[] = {
  Waypoint{4.0, 0.0, 0.1},
  Waypoint{6.0, 2.0, 0.1},
  Waypoint{6.0, 4.0, 0.1},
  Waypoint{4.0, 6.0, 0.1},
  Waypoint{2.0, 6.0, 0.1},
  Waypoint{0.0, 4.0, 0.1},
  Waypoint{0.0, 0.0, 0.1}
};
int waypointsLength =  sizeof(waypoints) / sizeof(Waypoint);

long lastPositionTimestamp = 0;
long lastNavigationTimestamp = 0;
Position position;
void loop() {
  // Always calculate new timestamp and delta so we can use in loop
  long timestamp;

  // Do any operations that we need faster (like fast sensor polling) on each loop iteration
  bool buttonPush = digitalRead(BUTTON_PIN) == HIGH;

  // Track position (every X ms always)
  timestamp = millis();
  long positionTimeElapsedMs = timestamp - lastPositionTimestamp;
  float velocity = navigator.getVelocity();
  float turnRadius = navigator.getTurnRadius();
  if (positionTimeElapsedMs > 10) {
    lastPositionTimestamp = timestamp;
    position = tracker.update(velocity * float(positionTimeElapsedMs) / 1000.0, turnRadius);
  }

  // If we're navigating, update navigation at a certain frequency with current position
  if (navigator.isRunning()) {
    // Adjust navigation (every X ms)
    timestamp = millis();
    long navigationTimeElapsedMs = timestamp - lastNavigationTimestamp;
    if (navigationTimeElapsedMs > 100) {
      lastNavigationTimestamp = timestamp;
      navigator.update(position);
      // If velocity and/or turnRadius changed, force a position update
      float newVelocity = navigator.getVelocity();
      float newTurnRadius = navigator.getTurnRadius();
      if (newVelocity != velocity || newTurnRadius != turnRadius) {
        timestamp = millis();
        long positionTimeElapsedMs = timestamp - lastPositionTimestamp;
        lastPositionTimestamp = timestamp;
        position = tracker.update(velocity * float(positionTimeElapsedMs) / 1000.0, turnRadius);
      } 
    }
  } else {
    // Otherwise potentially start running if button is pushed
    if (buttonPush) {
      digitalWrite(LED_PIN, HIGH);
      delay(3000);
      digitalWrite(LED_PIN, LOW);
      position = tracker.reset();
      navigator.start(waypoints, waypointsLength);
    }
  }
}

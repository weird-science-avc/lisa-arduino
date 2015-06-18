#include <Servo.h>
#include "position_tracker.h"
#include "navigator.h"

// PIN ASSIGNMENTS
const int SPEED_SERVO_PIN = 9;
const int STEERING_SERVO_PIN = 10;
const int BUTTON_PIN = 12;
const int LED_PIN = 13;
const int WHEEL_ENCODER_INT = 0; // pin 2

// TODO: Move into a class
const int MOCK_WHEEL_ENCODER_PIN = 8;
const bool MOCK_IMU = false;
void handleMocks();

// Create helper objects
PositionTracker tracker;
Navigator navigator;

// Global tick counter for wheel encoder
volatile int gWheelEncoderTicks = 0;
volatile float gPitch = 0.0;
volatile float gRoll = 0.0;
volatile float gYaw = 0.0;

void isrWheelEncoder() {
  // Increment
  gWheelEncoderTicks++;
}

// This is called once
void setup() {
  // Attach to interrupt 0 for wheel encoder
  attachInterrupt(WHEEL_ENCODER_INT, isrWheelEncoder, CHANGE);

  // Start serial at 9600 baud
  Serial.begin(115200);

  // Setup pins correctly
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOCK_WHEEL_ENCODER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // PositionTracker initialization

  // Navigator initialization
  navigator.attachSpeedServo(SPEED_SERVO_PIN);
  navigator.attachSteeringServo(STEERING_SERVO_PIN);
}

Waypoint waypoints[] = {
  Waypoint{15.0, 0.0, 0.1},
  //Waypoint{6.0, 2.0, 0.1},
  //Waypoint{6.0, 4.0, 0.1},
  //Waypoint{4.0, 6.0, 0.1},
  //Waypoint{2.0, 6.0, 0.1},
  //Waypoint{0.0, 4.0, 0.1},
  //Waypoint{0.0, -3.0, 3.0} // To ensure we go past the finish but with a wide variance, set past, but allow big tolerance
};
int waypointsLength =  sizeof(waypoints) / sizeof(Waypoint);
int waypointIndex;

// TODO: Move out into class
bool canPromoteWaypoint(Position p);
bool haveArrivedWaypoint(Position p);

long startTimestamp = 0;
long lastPositionTimestamp = 0;
long lastNavigationTimestamp = 0;
Position position;
void loop() {
  handleMocks();
  
  // Always calculate new timestamp and delta so we can use in loop
  long timestamp = millis();

  // Do any operations that we need faster (like fast sensor polling) on each loop iteration
  bool buttonPush = digitalRead(BUTTON_PIN) == HIGH;
  
  // If we push the button while running, stop
  if (navigator.isRunning() && (buttonPush || (timestamp - startTimestamp > 30000))) {
    navigator.stop();
    if (LOG_POSITION_DEBUG) { tracker.debugOff(); }
    // Blink lights 3 times to let people know we're done
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(5000);
    return;
  }

  // Track position (every X ms always)
  timestamp = millis();
  long positionTimeElapsedMs = timestamp - lastPositionTimestamp;
  if (positionTimeElapsedMs > 100) {
    lastPositionTimestamp = timestamp;
    position = tracker.update();
  }
  
  // Get current waypoint
  Waypoint waypoint = waypoints[waypointIndex];
  // Check to see if we're at a waypoint or can promote if we're running
  if (navigator.isRunning()) {
    bool canPromote = canPromoteWaypoint(position);
    bool haveArrived = haveArrivedWaypoint(position);
    if (canPromote || haveArrived) {
      // Print arrival info
      if (LOG_NAVIGATION_INFO) {
        // TODO: Calculate time to finish waypoint
        Serial.print("FINISH WAYPOINT");
        if (canPromote && !haveArrived) {
          Serial.print("(PROMOTION)");
        }
        serialPrintWaypoint(": ", waypointIndex, waypoint);
        serialPrintPosition(" (position:", position);
        Serial.println(")");
      }

      // Try to move to next waypoint an
      waypointIndex++;
      if (waypointIndex < waypointsLength) {
        waypoint = waypoints[waypointIndex];
        if (LOG_NAVIGATION_INFO) {
          Serial.print("START WAYPOINT");
          if (canPromote && !haveArrived) {
            Serial.print("(PROMOTION)");
          }
          serialPrintWaypoint(": ", waypointIndex, waypoint);
          serialPrintPosition(" (position:", position);
          Serial.println(")");
        }

      } else { // Finish
        navigator.stop();
        if (LOG_POSITION_DEBUG) { tracker.debugOff(); }
        if (LOG_NAVIGATION_INFO) {
          // TODO: Calculate time to finish navigation
          Serial.println("**** END NAVIGATION ****");
        }
        // Blink lights 3 times to let people know we're done
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
      }
    }
  }

  // If we're navigating, update navigation at a certain frequency with current position
  if (navigator.isRunning()) {
    // Adjust navigation (every X ms)
    timestamp = millis();
    long navigationTimeElapsedMs = timestamp - lastNavigationTimestamp;
    if (positionTimeElapsedMs > 100) {
    //if (navigationTimeElapsedMs > 100) {
      lastNavigationTimestamp = timestamp;
      bool navigationChanged = navigator.update(position, waypoint);
      // If navigation has changed, force an update
      if (navigationChanged) {
        timestamp = millis();
        long positionTimeElapsedMs = timestamp - lastPositionTimestamp;
        lastPositionTimestamp = timestamp;
        position = tracker.update();
      }
    }

  } else {
    // Otherwise potenially start running if button is pushed
    if (buttonPush) {
      digitalWrite(LED_PIN, HIGH);
      delay(3000);
      digitalWrite(LED_PIN, LOW);
      position = tracker.reset();
      timestamp = millis();
      lastPositionTimestamp = timestamp;
      lastNavigationTimestamp = timestamp;
      Serial.print("*** STARTING NAVIGATION -- ");
      Serial.print(waypointsLength);
      Serial.println(" waypoints ***");
      startTimestamp = timestamp;
      waypointIndex = 0;
      if (LOG_POSITION_DEBUG) { tracker.debugOn(); }
      navigator.start();
    }
  }
}

// If IMU data available read it
void serialEvent() {
  if (!MOCK_IMU) {
    gRoll = Serial.parseFloat();
    gPitch = Serial.parseFloat();
    gYaw = Serial.parseFloat();
    char garbage[20];
    Serial.readBytesUntil('\0', garbage, 20);
    //Serial.print("gYaw: ");
    //Serial.println(gYaw);
  }
}

long lastMockWheelEncoderTimestamp = 0;
void handleMocks() {
  long timestamp = millis();
  
  // MOCK: toggle wheel encoder based on speed so we can mock without the care if we like
  SPEED speed = navigator.getSpeed();
  long mockWheelEncoderElapsedMs = timestamp - lastMockWheelEncoderTimestamp;
  if ((speed == SPEED_HIGH && mockWheelEncoderElapsedMs > SPEED_HIGH_VELOCITY / WHEEL_ENCODER_M_DISTANCE_FROM_TICKS) ||
      (speed == SPEED_LOW && mockWheelEncoderElapsedMs > SPEED_LOW_VELOCITY / WHEEL_ENCODER_M_DISTANCE_FROM_TICKS)) {

    lastMockWheelEncoderTimestamp = timestamp;
    digitalWrite(MOCK_WHEEL_ENCODER_PIN, !digitalRead(MOCK_WHEEL_ENCODER_PIN));

    // MOCK: update orientation based on some correlation to steering and speed so we can test
    STEERING steering = navigator.getSteering();
    if (MOCK_IMU) {
      // NOTE: IMU goes backwards, where positive is right, negative left, so adjust
      if (steering == STEERING_LEFT) {
        gYaw = gYaw - (WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * 1.0 / STEERING_LEFT_TURN_RADIUS) * 180.0 / PI;
      } else if (steering == STEERING_RIGHT) {
        gYaw = gYaw + (WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * 1.0 / STEERING_RIGHT_TURN_RADIUS) * 180.0 / PI;
      }
    }
  }
}

// TODO: Move out to class
bool canPromoteWaypoint(Position p) {
  // If there's a next waypoint, think about promoting it
  if (waypointIndex + 1 < waypointsLength) {
    Waypoint waypoint = waypoints[waypointIndex];
    Waypoint nextWaypoint = waypoints[waypointIndex + 1];
    Vector nextWaypointVector = getVector(p.x, p.y, nextWaypoint.x, nextWaypoint.y);
    Vector waypointToWaypointVector = getVector(waypoint.x, waypoint.y, nextWaypoint.x, nextWaypoint.y);
    // If the distance to our next waypoint is less than the distance between them, we've passed by so promote
    // TODO: Phil suggested we could actual remove current waypoint's tolerance from our distance to next as well and it'd be okay
    return nextWaypointVector.d < waypointToWaypointVector.d;
  }
  return false;
}

bool haveArrivedWaypoint(Position p) {
  Waypoint w = waypoints[waypointIndex];
  Vector v = getVector(p.x, p.y, w.x, w.y);
  return v.d < w.tolerance;
}


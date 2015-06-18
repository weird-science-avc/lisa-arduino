#include <Servo.h>
#include "position_tracker.h"
#include "waypoint_manager.h"
#include "navigator.h"

// PIN ASSIGNMENTS
const int SPEED_SERVO_PIN = 9;
const int STEERING_SERVO_PIN = 10;
const int BUTTON_PIN = 12;
const int LED_PIN = 13;
const int WHEEL_ENCODER_INT = 0; // pin 2

// TODO: Move into a class
const int MOCK_WHEEL_ENCODER_PIN = 8;
const bool MOCK_IMU = true;
void handleMockSensors();

// Create helper objects
PositionTracker tracker(LOG_LEVEL_INFO);
WaypointManager manager(LOG_LEVEL_INFO);
Navigator navigator(LOG_LEVEL_INFO);
int logLevel = LOG_LEVEL_INFO;

// Global values for wheel encoder and IMU
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

  // WaypointManager initialization
  Waypoint waypoints[] = {
    Waypoint{4.0, 0.0, 0.1},
    Waypoint{6.0, 2.0, 0.1},
    Waypoint{6.0, 4.0, 0.1},
    Waypoint{4.0, 6.0, 0.1},
    Waypoint{2.0, 6.0, 0.1},
    Waypoint{0.0, 4.0, 0.1},
    Waypoint{0.0, -3.0, 3.0}
  };
  manager.setWaypoints(waypoints, sizeof(waypoints) / sizeof(Waypoint));

  // Navigator initialization
  navigator.attachSpeedServo(SPEED_SERVO_PIN);
  navigator.attachSteeringServo(STEERING_SERVO_PIN);
}

const long EMERGENCY_TIMEOUT_MS = 30000;
const long UPDATE_FREQUENCY_MS = 100;

bool started = false;
long startedTimestamp = 0;
long lastUpdatedTimestamp = 0;
void loop() {
  // Always handle mock sensors
  handleMockSensors();

  // If we've travelled longer than our cutoff that means emergency stop
  if (started && (millis() - startedTimestamp > EMERGENCY_TIMEOUT_MS)) {
    emergencyStop();
    return;
  }

  // Next deal with button pushes (action then immediate exit)
  if (digitalRead(BUTTON_PIN) == HIGH) {
    if (started) {
      emergencyStop();
    }
    else {
      start();
    }
    return;
  }

  // If not started, nothing else is necessary to do
  if (!started) {
    return;
  };

  // We are started, update every so often
  long timestamp = millis();
  if (millis() - lastUpdatedTimestamp > UPDATE_FREQUENCY_MS) {
    // Update position, waypoint, check done, and navigation
    Position position = tracker.update();
    Waypoint* waypoint = manager.getWaypoint(position);
    if (waypoint == NULL) {
      stop();
    } else {
      navigator.update(position, *waypoint);
      lastUpdatedTimestamp = millis();
    }
  }
}

// If IMU data available read it
// TODO: Consider moving this to PositionTracker and just have it read and wait for data
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

void start() {
  // Give 3s LED to warn about starting
  blink(LED_PIN, 1, 3000, 0);
  if (logLevel >= LOG_LEVEL_INFO) {
    Serial.print("*** STARTING NAVIGATION -- ");
    Serial.print(manager.length());
    Serial.println(" waypoints ***");
  }
  long timestamp = millis();
  startedTimestamp = timestamp;
  lastUpdatedTimestamp = timestamp;
  tracker.reset();
  manager.reset();
  navigator.reset();
  started = true;
}

void stop() {
  navigator.fullStop();
  started = false;
  if (logLevel >= LOG_LEVEL_INFO) {
    // TODO: Calculate time to finish navigation
    long duration = millis() - startedTimestamp;
    Serial.print("**** END NAVIGATION -- ");
    Serial.print(float(duration) / 1000.0);
    Serial.println("s ****");
  }
  blink(LED_PIN, 3, 100, 100);
}

void emergencyStop() {
  navigator.fullStop();
  started = false;

  if (logLevel >= LOG_LEVEL_INFO) {
    // TODO: Calculate time to finish navigation
    long duration = millis() - startedTimestamp;
    Serial.print("**** END NAVIGATION (EMERGECY) -- ");
    Serial.print(float(duration) / 1000.0);
    Serial.println("s ****");
  }
  blink(LED_PIN, 5, 1000, 1000);
}

long lastMockWheelEncoderTimestamp = 0;
void handleMockSensors() {
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
      while (gYaw > 180.0) { gYaw -= 360.0; }
      while (gYaw < -180.0) { gYaw += 360.0; }
    }
  }
}

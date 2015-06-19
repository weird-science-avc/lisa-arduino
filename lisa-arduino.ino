#include <Servo.h>
#include "position_tracker.h"
#include "waypoint_manager.h"
#include "navigator.h"

// *** DEFINE COURSE HERE ***
const long EMERGENCY_TIMEOUT_MS = 1 * 60 * 1000;
Waypoint waypoints[] = {
  //Waypoint{10, 0, 0.3},
  // Left 10m square
  /*
  Waypoint{8, 0, 0.3},
  Waypoint{10, 2, 0.3},
  Waypoint{10, 8, 0.3},
  Waypoint{8, 10, 0.3},
  Waypoint{2, 10, 0.3},
  Waypoint{0, 8, 0.3},
  Waypoint{0, -3, 3},
  */

  // Right 10m square
  /*
  Waypoint{8, 0, 0.3},
  Waypoint{10, -2, 0.3},
  Waypoint{10, -8, 0.3},
  Waypoint{8, -10, 0.3},
  Waypoint{2, -10, 0.3},
  Waypoint{0, -8, 0.3},
  Waypoint{0, 3, 3},
  */

  // Figure 8
  Waypoint{3, -3, 0.3},
  Waypoint{6, 0, 0.3},
  Waypoint{9, 3, 0.3},
  Waypoint{12, 0, 0.3},
  Waypoint{9, -3, 0.3},
  Waypoint{6, 0, 0.3},
  Waypoint{3, 3, 0.3},
  Waypoint{0, 0, 0.3},
  Waypoint{-4, 0, 3},
};

// PIN ASSIGNMENTS
const int SPEED_SERVO_PIN = 9;
const int STEERING_SERVO_PIN = 10;
const int BUTTON_PIN = 12;
const int LED_PIN = 13;
const int WHEEL_ENCODER_INT = 0; // pin 2

// TODO: Move into a class
const long MOCK_WHEEL_ENCODER_PIN = 8;
const bool MOCK_IMU = false;
void handleMockSensors();

// Create helper objects
PositionTracker tracker(LOG_LEVEL_SILENT);
WaypointManager manager(LOG_LEVEL_SILENT);
Navigator navigator(LOG_LEVEL_SILENT);
int logLevel = LOG_LEVEL_SILENT;

// Global values for wheel encoder and IMU
volatile long gWheelEncoderTicks = 0;
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
  manager.setWaypoints(waypoints, sizeof(waypoints) / sizeof(Waypoint));

  // Navigator initialization
  navigator.attachSpeedServo(SPEED_SERVO_PIN);
  navigator.attachSteeringServo(STEERING_SERVO_PIN);
}

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
    manager.update(position);
    if (manager.finished()) {
      stop();
    } else {
      navigator.update(position, manager.getWaypoint());
      lastUpdatedTimestamp = millis();
    }
  }
}

// If IMU data available read it
void serialEvent() {
  if (!MOCK_IMU) {
    float roll = Serial.parseFloat();
    float pitch = Serial.parseFloat();
    float yaw = Serial.parseFloat();
    char garbage[20];
    Serial.readBytesUntil('\0', garbage, 20);
    //Serial.print("gYaw: ");
    //Serial.println(gYaw);
    // NOTE: Sometimes the IMU spikes changes so limit the size we believe for a given fast loop rad
    //if (abs(yaw - gYaw) < IMU_MAX_DELTA_DEGREES) {
      gYaw = yaw;
    //}
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
  tracker.reset();
  manager.reset();
  navigator.reset();
  long timestamp = millis();
  startedTimestamp = timestamp;
  lastUpdatedTimestamp = timestamp;
  started = true;
}

void stop() {
  navigator.fullStop();
  started = false;
  if (logLevel >= LOG_LEVEL_INFO) {
    // TODO: Calculate time to finish navigation
    long timestamp = millis();
    long duration = timestamp - startedTimestamp;
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
const long mockHighSpeedMsPerTick = long(WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * 1000.0 / SPEED_HIGH_VELOCITY);
const long mockLowSpeedMsPerTick = long(WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * 1000.0 / SPEED_LOW_VELOCITY);
void handleMockSensors() {
  long timestamp = millis();

  // MOCK: toggle wheel encoder based on speed so we can mock without the care if we like
  SPEED speed = navigator.getSpeed();
  long mockWheelEncoderElapsedMs = timestamp - lastMockWheelEncoderTimestamp;
  if ((speed == SPEED_HIGH && mockWheelEncoderElapsedMs > mockHighSpeedMsPerTick) ||
      (speed == SPEED_LOW && mockWheelEncoderElapsedMs > mockLowSpeedMsPerTick)) {

    lastMockWheelEncoderTimestamp = timestamp;
    digitalWrite(MOCK_WHEEL_ENCODER_PIN, !digitalRead(MOCK_WHEEL_ENCODER_PIN));

    // MOCK: update orientation based on some correlation to steering and speed so we can test
    STEERING steering = navigator.getSteering();
    if (MOCK_IMU) {
      // NOTE: IMU goes backwards, where positive is right, negative left, so adjust
      if (steering == STEERING_LEFT) {
        gYaw = gYaw - (WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * 1.0 / STEERING_LEFT_TURN_RADIUS) * 180.0 / PI;
      } else if (steering == STEERING_RIGHT) {
        gYaw = gYaw - (WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * 1.0 / STEERING_RIGHT_TURN_RADIUS) * 180.0 / PI;
      }
      while (gYaw > 180.0) { gYaw -= 360.0; }
      while (gYaw < -180.0) { gYaw += 360.0; }
    }
  }
}

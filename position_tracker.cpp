#include "position_tracker.h"

PositionTracker::PositionTracker(int logLevel) {
  this->logLevel = logLevel;
}

Position PositionTracker::reset() {
  this->position = Position{0, 0, 0};
  this->lastWheelEncoderTicks = gWheelEncoderTicks;
  this->lastYaw = gYaw;
  if (this->logLevel >= LOG_LEVEL_DEBUG) {
    serialPrintlnPosition("POSITION(RESET): ", this->position);
  }
  return this->position;
}

Position PositionTracker::update() {
  // Figure out IMU's latest orientation, figure out rDelta, and updated stored value
  // NOTE: IMU has right positive, so do last - now instead of more normal now - last to make left positive again.
  float rDelta = normalizeRadians((this->lastYaw - gYaw) * PI / 180.0);
  // NOTE: Sometimes the IMU spikes changes so limit the size we believe
  if (abs(rDelta) > IMU_MAX_DELTA_RADIANS) {
    // Skip this position update and get next time
    return this->position;
  }
  // Successful read, store as last read
  this->lastYaw = gYaw;

  // Figure out wheel encoder delta, update stored value and calculate distance
  int wheelEncoderTicks = gWheelEncoderTicks;
  int wheelEncoderDelta = wheelEncoderTicks - this->lastWheelEncoderTicks;
  lastWheelEncoderTicks = wheelEncoderTicks;
  float distance = WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * float(wheelEncoderDelta);

  // Figure out position deltas based on straight or banked travel
  float xDelta = 0.0;
  float yDelta = 0.0;
  if (rDelta == 0.0) {
    //Serial.println("Straight motion");
    xDelta = distance * cos(this->position.r);
    yDelta = distance * sin(this->position.r);
  } else {
    //Serial.println("Banked motion");
    // Get the turn radius from the distance and angle change
    float turnRadius = distance / rDelta;
    // Calculate x and y deltas as if we were at (0,0) pointed along x-axis
    // NOTE: x is 'up' for us, so that's governed by Sin
    float xDeltaOrigin = turnRadius * sin(rDelta);
    float yDeltaOrigin = turnRadius * (-cos(rDelta) + 1.0);

    // Now we need to rotate those deltas around (0,0) by our current orientation so they're correct
    float sinR = sin(this->position.r);
    float cosR = cos(this->position.r);
    xDelta = xDeltaOrigin * cosR - yDeltaOrigin * sinR;
    yDelta = xDeltaOrigin * sinR + yDeltaOrigin * cosR;
  }

  // Update position and return
  Position newPosition;
  newPosition.x = this->position.x + xDelta;
  newPosition.y = this->position.y + yDelta;
  newPosition.r = normalizeRadians(this->position.r + rDelta);
  // TODO: Write equality overloads for Position
  if (this->logLevel >= LOG_LEVEL_DEBUG && (this->logLevel >= LOG_LEVEL_VERBOSE || newPosition.x != position.x || newPosition.y != this->position.y || newPosition.r != this->position.r)) {
    serialPrintlnPosition("POSITION: ", newPosition);
    Serial.print("WHEEL ENCODER: ");
    Serial.println(this->lastWheelEncoderTicks);
  }
  this->position = newPosition;
  return this->position;
}

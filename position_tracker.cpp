#include "Arduino.h"
#include "position_tracker.h"

Position PositionTracker::reset() {
  this->position = Position{0, 0, 0};
  this->lastWheelEncoderTicks = gWheelEncoderTicks;
  this->lastYaw = gYaw;
  if (LOG_POSITION_DEBUG) {
    serialPrintlnPosition("POSITION(RESET): ", this->position);
    Serial.print("WHEEL ENCODER: ");
    Serial.println(this->lastWheelEncoderTicks);
  }
  return this->position;
}

// turnRadius of 0.0 or NAN means straight
Position PositionTracker::update(float distance, float turnRadius) {
  // Figure out wheel encoder delta, update stored value and calculate distance
  int wheelEncoderTicks = gWheelEncoderTicks;
  int wheelEncoderDelta = wheelEncoderTicks - this->lastWheelEncoderTicks;
  lastWheelEncoderTicks = wheelEncoderTicks;
  distance = WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * float(wheelEncoderDelta);

  // Figure out IMU's latest orientation, updated stored value and calculate turnRadius
  float yaw = gYaw;
  if (yaw != this->lastYaw) {
    //Serial.print("yaw: ");
    //Serial.print(yaw);
    //Serial.print(", lastYaw: ");
    //Serial.println(this->lastYaw);
    // IMU has right positive, so switch our math to make left positive again.
    float yawDelta = this->lastYaw - yaw;
    this->lastYaw = yaw;
    turnRadius = distance / (yawDelta * PI / 180.0);
  }

  // Figure out position deltas based on straight or banked travel
  float xDelta = 0.0;
  float yDelta = 0.0;
  float rDelta = 0.0;
  if (isnan(turnRadius) || turnRadius == 0.0) {
    //Serial.println("Straight motion");
    xDelta = distance * cos(this->position.r);
    yDelta = distance * sin(this->position.r);
  } else {
    //Serial.println("Banked motion");
    // Get the rDelta
    rDelta = distance / turnRadius;
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
  newPosition.r = this->position.r + rDelta;
  // Keep orientation from [0,2PI]
  if (position.r < 0) {
    newPosition.r += 2 * PI;
  } else if (position.r > 2 * PI) {
    newPosition.r -= 2 * PI;
  }
  // TODO: Write equality overloads for Position
  if (LOG_POSITION_DEBUG && (newPosition.x != position.x || newPosition.y != this->position.y || newPosition.r != this->position.r)) {
    serialPrintlnPosition("POSITION: ", newPosition);
    Serial.print("WHEEL ENCODER: ");
    Serial.println(this->lastWheelEncoderTicks);
  }
  this->position = newPosition;
  return this->position;
}

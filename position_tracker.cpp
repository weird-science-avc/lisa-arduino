#include "Arduino.h"
#include "position_tracker.h"

// NOTE: If/when we have a functioning IMU we can use it to track angle change
// and then calculate turnRadius = distance / angle and ignore the one
// calculated from steering as well

void PositionTracker::reset() {
  position = Position{0, 0, 0};
  lastWheelEncoderTicks = gWheelEncoderTicks;
}

// turnRadius of 0.0 or NAN means straight
Position PositionTracker::update(float distance, float turnRadius) {
  // Figure out wheel encoder delta and update stored value
  int wheelEncoderTicks = gWheelEncoderTicks;
  int wheelEncoderDelta = wheelEncoderTicks - lastWheelEncoderTicks;
  lastWheelEncoderTicks = wheelEncoderTicks;

  // Override distance since we have a better measurement from wheel encoder;
  // TODO: When happy with wheel encoder, remove passed in distance
  distance = WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * float(wheelEncoderDelta);

  // Figure out position deltas based on straight or banked travel
  float xDelta = 0.0;
  float yDelta = 0.0;
  float rDelta = 0.0;
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

  // Update position and return
  position.x += xDelta;
  position.y += yDelta;
  position.r += rDelta;
  // Keep orientation from [0,2PI]
  if (position.r < 0) {
    position.r += 2 * PI;
  } else if (position.r > 2 * PI) {
    position.r -= 2 * PI;
  }
  serialPrintPosition("POSITION: ", position);
  return position;
}

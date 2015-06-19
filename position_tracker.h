#ifndef PositionTracker_h
#define PositionTracker_h

#include "Arduino.h"
#include "data_types.h"

// Access global wheel encoder ticks
extern volatile long gWheelEncoderTicks;
extern volatile float gPitch;
extern volatile float gRoll;
extern volatile float gYaw;

const float IMU_MAX_DELTA_DEGREES = 10.0;

// TODO: Put the actual things measured in here and use to compute M so that it's documented
const float WHEEL_ENCODER_M_DISTANCE_FROM_TICKS = 0.0544737;

class PositionTracker
{
  public:
    PositionTracker(int logLevel);

    Position reset();
    Position update();

  private:
    int logLevel = 0;
    Position position;
    long lastWheelEncoderTicks = 0;
    float lastYaw = 0.0;
};

#endif

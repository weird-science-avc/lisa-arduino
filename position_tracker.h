#ifndef PositionTracker_h
#define PositionTracker_h

#include "data_types.h"

// Access global wheel encoder ticks
extern volatile int gWheelEncoderTicks;
extern volatile float gPitch;
extern volatile float gRoll;
extern volatile float gYaw;

// TODO: Put the actual things measured in here and use to compute M so that it's documented
const float WHEEL_ENCODER_M_DISTANCE_FROM_TICKS = 0.0544737;

class PositionTracker
{
  public:
    Position reset();
    Position update(float distance, float turnRadius);

  private:
    Position position;
    int lastWheelEncoderTicks = 0;
    float lastYaw = 0.0;
};

#endif

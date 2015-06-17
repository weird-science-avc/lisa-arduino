#ifndef PositionTracker_h
#define PositionTracker_h

#include "data_types.h"

// Access global wheel encoder ticks
extern volatile int gWheelEncoderTicks;

const float WHEEL_ENCODER_M_DISTANCE_FROM_TICKS = 0.0544737;

class PositionTracker
{
  public:
    void reset();
    Position update(float distance, float turnRadius);

  private:
    Position position;
    int lastWheelEncoderTicks = 0;
};

#endif

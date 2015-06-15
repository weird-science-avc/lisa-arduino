#ifndef PositionTracker_h
#define PositionTracker_h

#include "data_types.h"

// Access global wheel encoder ticks
extern volatile int gWheelEncoderTicks;

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

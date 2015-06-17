#ifndef PositionTracker_h
#define PositionTracker_h

#include "data_types.h"

class PositionTracker
{
  public:
    Position reset();
    Position update(float distance, float turnRadius);

  private:
    Position position;
};

#endif

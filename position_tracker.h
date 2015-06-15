#ifndef PositionTracker_h
#define PositionTracker_h

#include "data_types.h"

class PositionTracker
{
  public:
    void reset();
    Position update(float distance, float turnRadius);

  private:
    Position position;
};

#endif

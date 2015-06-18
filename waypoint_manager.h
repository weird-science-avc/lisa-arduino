#ifndef WaypointManager_h
#define WaypointManager_h

#include "data_types.h"

class WaypointManager
{
  public:
    WaypointManager(int logLevel);

    void reset();
    int length();

    void setWaypoints(Waypoint* waypoints, int waypointsLength);
    // Returns NULL when finished
    Waypoint* getWaypoint(Position p);

  private:
    int logLevel = 0;
    Waypoint* waypoints = 0;
    int waypointsLength = 0;
    int waypointIndex = 0;
};

#endif

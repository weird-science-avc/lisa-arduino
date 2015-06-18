#include "Arduino.h"
#include "waypoint_manager.h"

WaypointManager::WaypointManager(int logLevel) {
  this->logLevel = logLevel;
}

void WaypointManager::reset() {
  this->waypointIndex = -1;
}

int WaypointManager::length() {
  return this->waypointsLength;
}

void WaypointManager::setWaypoints(Waypoint* waypoints, int waypointsLength) {
  // Copy the waypoint data over so when its dereferenced all is okay
  this->waypoints = new Waypoint[waypointsLength];
  this->waypointsLength = waypointsLength;
  for (int i = 0; i < waypointsLength; i++) {
    this->waypoints[i] = waypoints[i];
  }
}

Waypoint* WaypointManager::getWaypoint(Position p) {
  Waypoint w0;
  // Detect reset state so we can print a message sometimes
  if (this->waypointIndex < 0) {
    this->waypointIndex = 0;
    w0 = this->waypoints[this->waypointIndex];
    if (this->logLevel >= LOG_LEVEL_INFO) {
      serialPrintlnWaypoint("START WAYPOINT: ", this->waypointIndex, w0);
    }

  } else {
    w0 = this->waypoints[this->waypointIndex];
  }
  Vector v0 = getVector(p.x, p.y, w0.x, w0.y);

  // TODO: We can probably abstract this more to allow skipping even more waypoints when applicable
  if (v0.d < w0.tolerance) { // Arrived
    if (this->logLevel >= LOG_LEVEL_INFO) {
      serialPrintlnWaypoint("FINISH WAYPOINT (ARRIVED): ", this->waypointIndex, w0);
    }
    // Move forward, check all done, update return vars
    this->waypointIndex++;
    if (this->waypointIndex >= this->waypointsLength) { return NULL; }
    w0 = this->waypoints[this->waypointIndex];
    v0 = getVector(p.x, p.y, w0.x, w0.y);
    if (this->logLevel >= LOG_LEVEL_INFO) {
      serialPrintlnWaypoint("START WAYPOINT: ", this->waypointIndex, w0);
    }

  } else if (this->waypointIndex + 1 < this->waypointsLength) { // Attempt promotion
    Waypoint w1 = this->waypoints[waypointIndex + 1];
    Vector v1 = getVector(p.x, p.y, w1.x, w1.y);
    Vector v01 = getVector(w0.x, w0.y, w1.x, w1.y);
    if (v1.d < v01.d) { // p -> w1 smaller than w0 -> w1, promote
      if (this->logLevel >= LOG_LEVEL_INFO) {
        serialPrintlnWaypoint("FINISH WAYPOINT (PROMOTION): ", this->waypointIndex, w0);
      }
      // Move forward, update return vars
      this->waypointIndex++;
      w0 = w1;
      v0 = v1;
      if (this->logLevel >= LOG_LEVEL_INFO) {
        serialPrintlnWaypoint("START WAYPOINT: ", this->waypointIndex, w0);
      }
    }
  }

  return &w0;
}

#ifndef DataTypes_h
#define DataTypes_h

struct Waypoint {
  float x;
  float y;
  float tolerance;
};

struct Location {
  float x;
  float y;
};

struct Position {
  float x;
  float y;
  float r;
};

struct Vector {
  float r;
  float d;
};

void serialPrintWaypoint(char* prefix, int index, Waypoint w);
void serialPrintPosition(char* prefix, Position p);
void serialPrintLocation(char* prefix, Location l);

// Returns a vector with magnitude and angle [0, 2PI]
Vector getVector(Position p, Waypoint w);
bool haveArrived(Waypoint w, float distance);

#endif

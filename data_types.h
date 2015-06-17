#ifndef DataTypes_h
#define DataTypes_h

struct Location {
  float x;
  float y;
};

struct Waypoint {
  float x;
  float y;
  float tolerance;
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

void serialPrintLocation(char* prefix, Location l);
void serialPrintWaypoint(char* prefix, int index, Waypoint w);
void serialPrintPosition(char* prefix, Position p);
void serialPrintVector(char* prefix, Vector v);

// Returns a vector with magnitude and angle [0, 2PI]
Vector getVector(float x0, float y0, float x1, float y1);
bool haveArrived(Waypoint w, float distance);

#endif

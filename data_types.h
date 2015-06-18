#ifndef DataTypes_h
#define DataTypes_h

const int LOG_LEVEL_SILENT = 1;
const int LOG_LEVEL_WARN = 1;
const int LOG_LEVEL_INFO = 2;
const int LOG_LEVEL_DEBUG = 3;
const int LOG_LEVEL_VERBOSE = 4;

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
void serialPrintlnLocation(char* prefix, Location l);
void serialPrintWaypoint(char* prefix, int index, Waypoint w);
void serialPrintlnWaypoint(char* prefix, int index, Waypoint w);
void serialPrintPosition(char* prefix, Position p);
void serialPrintlnPosition(char* prefix, Position p);
void serialPrintVector(char* prefix, Vector v);
void serialPrintlnVector(char* prefix, Vector v);

void blink(int pin, int count, int onMs, int offMs);

// Returns a vector with magnitude and angle [-PI, PI]
Vector getVector(float x0, float y0, float x1, float y1);

// Moves the vector to [-PI, PI]
float normalizeRadians(float r);

#endif

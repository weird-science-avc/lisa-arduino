#include "Arduino.h"
#include "data_types.h"

void serialPrintLocation(char* prefix, Location l) {
  Serial.print(prefix);
  Serial.print("(");
  Serial.print(l.x);
  Serial.print(",");
  Serial.print(l.y);
  Serial.println(")");
}

void serialPrintWaypoint(char* prefix, int index, Waypoint w) {
  Serial.print(prefix);
  Serial.print(index);
  Serial.print(" -- (");
  Serial.print(w.x);
  Serial.print(",");
  Serial.print(w.y);
  Serial.println(")");
}

void serialPrintPosition(char* prefix, Position p) {
  Serial.print(prefix);
  Serial.print("(");
  Serial.print(p.x);
  Serial.print(",");
  Serial.print(p.y);
  Serial.print("):");
  Serial.println(p.r * 180.0 / PI);
}

void serialPrintVector(char* prefix, Vector v) {
  Serial.print(prefix);
  Serial.print(v.d);
  Serial.print("@");
  Serial.println(v.r * 180.0 / PI);
}

Vector getVector(float x0, float y0, float x1, float y1) {
  float deltaY = y1 - y0;
  float deltaX = x1 - x0;
  Vector v;
  v.d = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  v.r = atan2(deltaY, deltaX);
  if (v.r < 0) {
    v.r += 2 * PI;
  }
  return v;
}

bool haveArrived(Waypoint w, float distance) {
  return distance < w.tolerance;
}

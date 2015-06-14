#ifndef Navigator_h
#define Navigator_h

#include "Arduino.h"
#include <Servo.h>
#include "speed_calibration.h"
#include "steering_calibration.h"

// INTERNAL REFERNCE
const float MIN_VELOCITY = 0.25; // m/s
const float MAX_VELOCITY = 2.0; // m/s

// Angle (radians) under which we won't make steering corrections
const float ORIENTATION_DELTA = 5.0 * PI / 180.0;
// Distance (m) at which to say we're close enough to the target to say we're there
const float ARRIVE_DELTA = 0.1;
// Distance (m) at which we start slowing down to stop at the target
const float APPROACH_DELTA = 1.0;

// Use enum for steering; see steering_calibration.h for details
enum STEERING {
  STEERING_LEFT,
  STEERING_CENTER,
  STEERING_RIGHT
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

void serialPrintPosition(char* prefix, Position p);
void serialPrintLocation(char* prefix, Location l);

class Navigator
{
  public:
    Navigator(Servo speedServo, Servo steeringServo);
    void NavigateTo(Position start, Location target);

  private:
    Servo speedServo, steeringServo;

    Vector vectorToTarget(Position p, Location t);
    bool haveArrived(float distance);

    // turnRadius of 0.0 or NAN means straight
    float turnRadiusFromSteering(STEERING steering);
    Position calculateNewPosition(Position position, float distance, float turnRadius);
    float calculateNewVelocity(float velocity, float distance);
    STEERING calculateNewSteering(STEERING steering, float angle);

    // s = 0 => servoValue == 0
    // s = MIN_VELOCITY to MAX_VELOCITY according to calibration function
    void setVelocity(float velocity);


    void setSteering(STEERING steering);
};

#endif

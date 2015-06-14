#ifndef Navigator_h
#define Navigator_h

#include "Arduino.h"
#include <Servo.h>
#include "speed_calibration.h"
#include "steering_calibration.h"

// INTERNAL REFERNCE
const float MIN_VELOCITY = 0.25; // m/s
const float MAX_VELOCITY = 2.0; // m/s
const float MAX_STEERING = 30; // degrees 2120(left)

const float ARRIVE_DELTA = 0.05;
const float APPROACH_DELTA = 1.0;

// TODO: Do we need this more accurate
const float pi = 3.14;

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
    float turnRadiusFromSteering(float steering);
    Position calculateNewPosition(Position position, float distance, float turnRadius);
    float calculateNewVelocity(float velocity, float distance);
    float calculateNewSteering(float steering, float angle);

    // s = 0 => servoValue == 0
    // s = MIN_VELOCITY to MAX_VELOCITY according to calibration function
    void setVelocity(float velocity);


    // s = 0.0 => servoValue == CENTER_STEERING
    // s > 0 (left) => according to left calibration function
    // s < 0 (right) => according to right calibration function
    void setSteering(float steering);
};

#endif

#ifndef Navigator_h
#define Navigator_h

#include "Arduino.h"
#include <Servo.h>
#include "speed_calibration.h"
#include "steering_calibration.h"
#include "data_types.h"

// Angle (radians) under which we won't make steering corrections
const float ORIENTATION_DELTA = 1.0 * PI / 180.0;
// Distance (m) at which we slow down
const float APPROACH_DELTA = 1.0;

enum SPEED {
  SPEED_STOPPED,
  SPEED_LOW,
  SPEED_HIGH
};

enum STEERING {
  STEERING_LEFT,
  STEERING_CENTER,
  STEERING_RIGHT
};

class Navigator
{
  public:
    Navigator(int logLevel);

    void attachSpeedServo(int pin);
    void attachSteeringServo(int pin);

    // Start navigation amongst waypoints
    void reset();
    void fullStop();

    // Tells the navigator to update its course given a position and target waypoint
    bool update(Position p, Waypoint w);

    SPEED getSpeed();
    STEERING getSteering();

  private:
    int logLevel = 0;

    SPEED speed = SPEED_STOPPED;
    STEERING steering = STEERING_CENTER;
    Servo speedServo, steeringServo;

    void adjustSpeed(float distance);
    void adjustSteering(float orientation, float targetOrientation);

    void setSpeed(SPEED speed);
    void setSteering(float servoValue);
};

#endif

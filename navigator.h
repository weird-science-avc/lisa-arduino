#ifndef Navigator_h
#define Navigator_h

#include "Arduino.h"
#include <Servo.h>
#include "speed_calibration.h"
#include "steering_calibration.h"
#include "data_types.h"

// Angle (radians) under which we won't make steering corrections
const float ORIENTATION_DELTA = 5.0 * PI / 180.0;
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
    Navigator(int ledPin);
    void attachSpeedServo(int pin);
    void attachSteeringServo(int pin);

    float getVelocity();
    float getTurnRadius();

    // Start navigation amongst waypoints
    void start(Waypoint waypoints[], int waypointsLength);
    bool isRunning();

    // Tells the navigator to update its course, providing relavant external information as necessary
    void update(Position p);

  private:
    int ledPin;
    Waypoint* waypoints = 0;
    int waypointsLength = 0;
    int waypointIndex = 0;
    bool running = false;
    SPEED speed = SPEED_STOPPED;
    STEERING steering = STEERING_CENTER;
    Servo speedServo, steeringServo;

    void adjustSpeed(float distance);
    void adjustSteering(float orientation, float targetOrientation);

    void setSpeed(SPEED speed);
    void setSteering(STEERING steering);
};

#endif

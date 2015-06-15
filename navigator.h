#ifndef Navigator_h
#define Navigator_h

#include "Arduino.h"
#include <Servo.h>
#include "speed_calibration.h"
#include "steering_calibration.h"
#include "data_types.h"

// INTERNAL REFERNCE
const float MIN_VELOCITY = 0.25; // m/s
const float MAX_VELOCITY = 2.0; // m/s

// Angle (radians) under which we won't make steering corrections
const float ORIENTATION_DELTA = 5.0 * PI / 180.0;
// Distance (m) at which to say we're close enough to the target to say we're there
const float ARRIVE_DELTA = 0.1;
// Distance (m) at which we start slowing down to stop at the target
const float APPROACH_DELTA = 1.0;

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
    void start(Waypoint waypoints[]);
    bool isRunning();

    // Tells the navigator to update its course, providing relavant external information as necessary
    void update(Position p);

  private:
    int ledPin;
    Waypoint waypoints[];
    int waypointIndex = 0;
    bool running = false;
    float velocity;
    STEERING steering;
    Servo speedServo, steeringServo;

    // TODO: Consider going back to speed as an enum like steering, having specific values
    void adjustVelocity(float distance);
    void adjustSteering(float orientation, float targetOrientation);

    void setVelocity(float velocity);
    void setSteering(STEERING steering);
};

#endif

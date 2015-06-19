// TODO: Consider changing to #define to avoid taking dataspace

const float STEERING_MIN_SERVO = 1150; // all right
const float STEERING_MIN_TURN_RADIUS_INCH = 66;
const float STEERING_MAX_SERVO = 2120; // all left
const float STEERING_MAX_TURN_RADIUS_INCH = 77;

const float INCH_TO_M = 0.0254;

// FIXME: This math isn't quite right because the relationship between servo
// delta and steering can be linear (since we define it), but the relationship
// from servo delta (and therefore steering) to turn radius is asymptotic like
// an inverse function or something. For now though we'll just ignore all this
// and pick to either turn 0, fixed left or fixed right, where we have the
// direct correlation to left and right determined already.
const float STEERING_CENTER_SERVO = 1608; // Not (STEERING_MAX_SERVO + STEERING_MIN_SERVO) / 2 by experimentation
//const float STEERING_LEFT_SERVO = 1864; // half to max
//const float STEERING_LEFT_TURN_RADIUS_INCH = 146;
//const float STEERING_LEFT_TURN_RADIUS = STEERING_LEFT_TURN_RADIUS_INCH * INCH_TO_M; // m
const float STEERING_LEFT_SERVO = 2120; // max
const float STEERING_LEFT_SERVO_RANGE = STEERING_LEFT_SERVO - STEERING_CENTER_SERVO;
const float STEERING_LEFT_TURN_RADIUS_INCH = 77;
const float STEERING_LEFT_TURN_RADIUS = STEERING_LEFT_TURN_RADIUS_INCH * INCH_TO_M; // m
//const float STEERING_RIGHT_SERVO = 1379; // picked half way to max
//const float STEERING_RIGHT_TURN_RADIUS_INCH = -116;
//const float STEERING_RIGHT_TURN_RADIUS = STEERING_RIGHT_TURN_RADIUS_INCH * INCH_TO_M; // m
const float STEERING_RIGHT_SERVO = 1150; // picked half way to max
const float STEERING_RIGHT_SERVO_RANGE = STEERING_CENTER_SERVO - STEERING_RIGHT_SERVO;
const float STEERING_RIGHT_TURN_RADIUS_INCH = -66;
const float STEERING_RIGHT_TURN_RADIUS = STEERING_RIGHT_TURN_RADIUS_INCH * INCH_TO_M; // m

const float MIN_STEERING_THRESHOLD = 0.08726; // 5 degrees
const float MAX_STEERING_THRESHOLD = 0.52359; // 30 degrees
const float STEERING_THRESHOLD_RANGE = MAX_STEERING_THRESHOLD - MIN_STEERING_THRESHOLD;

//// Pick our 30 and 10 degree turn radiuses based on what we can hit with both left and right to standardize
//const float TURN_RADIUS_FOR_30_DEGREES = 1.0; // m
//const float TURN_RADIUS_FOR_10_DEGREES = 4.0; // m
//
//// *** RIGHT
//// Measured servo to circumference values
//const float RIGHT_CIRC0_INCH = 66;
//const float RIGHT_SERVO0 = 1150;
//const float RIGHT_CIRC1_INCH = 116;
//const float RIGHT_SERVO1 = 1379;
//
//// Convert to delta's from center and turn radius in meters
//const float RIGHT_RADIUS0_M = RIGHT_CIRC0_INCH * INCH_TO_M / 2.0;
//const float RIGHT_RADIUS1_M = RIGHT_CIRC1_INCH * INCH_TO_M / 2.0;
//const float RIGHT_SERVO0_DELTA = CENTER_STEERING_SERVO - RIGHT_SERVO0;
//const float RIGHT_SERVO1_DELTA = CENTER_STEERING_SERVO - RIGHT_SERVO1;
//
//// Calculate M and B for RIGHT servo delta from turn radius; servoDelta = M * turnRadius + B
////  used to derive the rest of the needed M/B values
//const float RIGHT_M_SERVO_DELTA_FROM_TURN_RADIUS = (RIGHT_SERVO1_DELTA - RIGHT_SERVO0_DELTA) / (RIGHT_RADIUS1_M - RIGHT_RADIUS0_M);
//const float RIGHT_B_SERVO_DELTA_FROM_TURN_RADIUS = RIGHT_SERVO1_DELTA - (RIGHT_M_SERVO_DELTA_FROM_TURN_RADIUS * RIGHT_RADIUS1_M);
//
//// Calculate M and B for RIGHT servo delta from steering value; 0 steering -> 0 servo delta, so no B value
//const float RIGHT_M_SERVO_DELTA_FROM_STEERING = TURN_RADIUS_FOR_30_DEGREES / 30.0;
//
//// Calculate M and B for RIGHT turn radius from steering value
////  Use reduction of units to combine existing slopes to get to this one
//const float RIGHT_M_TURN_RADIUS_FROM_STEERING = RIGHT_M_SERVO_DELTA_FROM_STEERING / RIGHT_M_SERVO_DELTA_FROM_TURN_RADIUS;
//// Compute B since we know steering 30.0 -> turn radius 1.0
//const float RIGHT_B_TURN_RADIUS_FROM_STEERING = 1.0 - (RIGHT_M_TURN_RADIUS_FROM_STEERING * 30.0);
//
//// *** LEFT
//// Measured servo to circumference values
//const float LEFT_CIRC0_IN = 77;
//const float LEFT_SERVO0 = 2120;
//const float LEFT_CIRC1_IN = 146;
//const float LEFT_SERVO1 = 1864;
//
//// Convert to delta's from center and turn radius in meters
//const float LEFT_RADIUS0_M = LEFT_CIRC0_IN * IN_TO_M / 2.0;
//const float LEFT_RADIUS1_M = LEFT_CIRC1_IN * IN_TO_M / 2.0;
//const float LEFT_SERVO0_DELTA = CENTER_STEERING_SERVO - LEFT_SERVO0;
//const float LEFT_SERVO1_DELTA = CENTER_STEERING_SERVO - LEFT_SERVO1;
//
//// Calculate M and B for LEFT servo delta from turn radius; servoDelta = M * turnRadius + B
////  used to derive the rest of the needed M/B values
//const float LEFT_M_SERVO_DELTA_FROM_TURN_RADIUS = (LEFT_SERVO1_DELTA - LEFT_SERVO0_DELTA) / (LEFT_RADIUS1_M - LEFT_RADIUS0_M);
//const float LEFT_B_SERVO_DELTA_FROM_TURN_RADIUS = LEFT_SERVO1_DELTA - (LEFT_M_SERVO_DELTA_FROM_TURN_RADIUS * LEFT_RADIUS1_M);
//
//// Calculate M for LEFT servo delta from steering value; 0 steering -> 0 servo delta, so no B value
//const float LEFT_M_SERVO_DELTA_FROM_STEERING = TURN_RADIUS_FOR_30_DEGREES / 30.0;
//
//// Calculate M and B for LEFT turn radius from steering value
////  Use reduction of units to combine existing slopes to get to this one
//const float LEFT_M_TURN_RADIUS_FROM_STEERING = LEFT_M_SERVO_DELTA_FROM_STEERING / LEFT_M_SERVO_DELTA_FROM_TURN_RADIUS;
//// Compute B since we know steering 30.0 -> turn radius 1.0
//const float LEFT_B_TURN_RADIUS_FROM_STEERING = 1.0 - (LEFT_M_TURN_RADIUS_FROM_STEERING * 30.0);

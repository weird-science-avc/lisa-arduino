// TODO: Consider changing to #define to avoid taking dataspace

const float MIN_STEERING_SERVO = 1150; // all right, 66" turn radius
// 1379 - 116" turn radius right
const float MAX_STEERING_SERVO = 2120; // all left, 77"
// 1864 - 146" turn radius left
const float CENTER_STEERING_SERVO = 1608; //(MAX_STEERING_SERVO + MIN_STEERING_SERVO) / 2;

const float IN_TO_M = 0.0254;

// Pick our 30 degree turn radius based on what we can hit with both left and right to standardize
const float TURN_RADIUS_FOR_30_DEGREES = 1.0; // m

// *** RIGHT
// Measured servo to circumference values
const float RIGHT_CIRC0_IN = 66;
const float RIGHT_SERVO0 = 1150;
const float RIGHT_CIRC1_IN = 116;
const float RIGHT_SERVO1 = 1379;

// Convert to delta's from center and turn radius in meters
const float RIGHT_RADIUS0_M = RIGHT_CIRC0_IN * IN_TO_M / 2.0;
const float RIGHT_RADIUS1_M = RIGHT_CIRC1_IN * IN_TO_M / 2.0;
const float RIGHT_SERVO0_DELTA = CENTER_STEERING_SERVO - RIGHT_SERVO0;
const float RIGHT_SERVO1_DELTA = CENTER_STEERING_SERVO - RIGHT_SERVO1;

// Calculate M and B for RIGHT servo delta from turn radius; servoDelta = M * turnRadius + B
//  used to derive the rest of the needed M/B values
const float RIGHT_M_SERVO_DELTA_FROM_TURN_RADIUS = (RIGHT_SERVO1_DELTA - RIGHT_SERVO0_DELTA) / (RIGHT_RADIUS1_M - RIGHT_RADIUS0_M);
const float RIGHT_B_SERVO_DELTA_FROM_TURN_RADIUS = RIGHT_SERVO1_DELTA - (RIGHT_M_SERVO_DELTA_FROM_TURN_RADIUS * RIGHT_RADIUS1_M);

// Calculate M for RIGHT servo delta from steering value; 0 steering -> 0 servo delta, so no B value
const float RIGHT_M_SERVO_DELTA_FROM_STEERING = TURN_RADIUS_FOR_30_DEGREES / 30.0;

// Calculate M and B for RIGHT turn radius from steering value
//  Use reduction of units to combine existing slopes to get to this one
const float RIGHT_M_TURN_RADIUS_FROM_STEERING = RIGHT_M_SERVO_DELTA_FROM_STEERING / RIGHT_M_SERVO_DELTA_FROM_TURN_RADIUS;
// Compute B since we know steering 30.0 -> turn radius 1.0
const float RIGHT_B_TURN_RADIUS_FROM_STEERING = 1.0 - (RIGHT_M_TURN_RADIUS_FROM_STEERING * 30.0);

// *** LEFT
// Measured servo to circumference values
const float LEFT_CIRC0_IN = 77;
const float LEFT_SERVO0 = 2120;
const float LEFT_CIRC1_IN = 146;
const float LEFT_SERVO1 = 1864;

// Convert to delta's from center and turn radius in meters
const float LEFT_RADIUS0_M = LEFT_CIRC0_IN * IN_TO_M / 2.0;
const float LEFT_RADIUS1_M = LEFT_CIRC1_IN * IN_TO_M / 2.0;
const float LEFT_SERVO0_DELTA = CENTER_STEERING_SERVO - LEFT_SERVO0;
const float LEFT_SERVO1_DELTA = CENTER_STEERING_SERVO - LEFT_SERVO1;

// Calculate M and B for LEFT servo delta from turn radius; servoDelta = M * turnRadius + B
//  used to derive the rest of the needed M/B values
const float LEFT_M_SERVO_DELTA_FROM_TURN_RADIUS = (LEFT_SERVO1_DELTA - LEFT_SERVO0_DELTA) / (LEFT_RADIUS1_M - LEFT_RADIUS0_M);
const float LEFT_B_SERVO_DELTA_FROM_TURN_RADIUS = LEFT_SERVO1_DELTA - (LEFT_M_SERVO_DELTA_FROM_TURN_RADIUS * LEFT_RADIUS1_M);

// Calculate M for LEFT servo delta from steering value; 0 steering -> 0 servo delta, so no B value
const float LEFT_M_SERVO_DELTA_FROM_STEERING = TURN_RADIUS_FOR_30_DEGREES / 30.0;

// Calculate M and B for LEFT turn radius from steering value
//  Use reduction of units to combine existing slopes to get to this one
const float LEFT_M_TURN_RADIUS_FROM_STEERING = LEFT_M_SERVO_DELTA_FROM_STEERING / LEFT_M_SERVO_DELTA_FROM_TURN_RADIUS;
// Compute B since we know steering 30.0 -> turn radius 1.0
const float LEFT_B_TURN_RADIUS_FROM_STEERING = 1.0 - (LEFT_M_TURN_RADIUS_FROM_STEERING * 30.0);

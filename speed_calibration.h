// TODO: Consider changing to #define to avoid taking dataspace

const int SPEED_MIN_SERVO = 660; // slowest non-zero
const int SPEED_MAX_SERVO = 1080; // fastest non-zero

// CALIBRATION
const float SPEED_M_SERVO_FROM_VELOCITY = 95.964;
const float SPEED_B_SERVO_FROM_VELOCITY = 659.19;

const int SPEED_STOPPED_SERVO = 0;
// NOTE: We can directly set velocity and servo instead of curve evaluation if we like
const float SPEED_LOW_VELOCITY = 0.5;
const int SPEED_LOW_SERVO = max(min(int(SPEED_M_SERVO_FROM_VELOCITY * SPEED_LOW_VELOCITY + SPEED_B_SERVO_FROM_VELOCITY), SPEED_MAX_SERVO), SPEED_MIN_SERVO);
// NOTE: If you bump up the top speed you should also make it do updates in the main loop faster or allow more IMU angle variation, otherwise we turn to fast to track
const float SPEED_HIGH_VELOCITY = 2.0;
const int SPEED_HIGH_SERVO = max(min(int(SPEED_M_SERVO_FROM_VELOCITY * SPEED_HIGH_VELOCITY + SPEED_B_SERVO_FROM_VELOCITY), SPEED_MAX_SERVO), SPEED_MIN_SERVO);

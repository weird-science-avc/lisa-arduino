#include <Servo.h>
#include "navigator.h"

// PIN ASSIGNMENTS
const int SPEED_SERVO_PIN = 9;
const int STEERING_SERVO_PIN = 10;
const int BUTTON_PIN = 12;
const int LED_PIN = 13;

// Servo controls for speed and steering
Servo speedServo, steeringServo;
Navigator navigator(speedServo, steeringServo);

// This is called once
void setup() {
  // Start serial at 9600 baud
  Serial.begin(9600);

  // Setup pins correctly
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Attach servos to pins
  speedServo.attach(SPEED_SERVO_PIN);
  steeringServo.attach(STEERING_SERVO_PIN);

  // Initialize to stopped on speed servo
  speedServo.writeMicroseconds(0);
}

// This is called on a loop
void loop() {

  if (digitalRead(BUTTON_PIN) == HIGH) {
    // Make sure our light is off, turn on to warn, turn off to work
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
    delay(3000);
    digitalWrite(LED_PIN, LOW);

    navigator.NavigateTo(Position{0.0, 0.0, 0.0}, Location{10.0, 0.0});

    // Let them know we finished our task
    digitalWrite(LED_PIN, HIGH);
  }
}

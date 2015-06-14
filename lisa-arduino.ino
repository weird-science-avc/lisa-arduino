#include <Servo.h>
#include "navigator.h"

// PIN ASSIGNMENTS
const int SPEED_SERVO_PIN = 9;
const int STEERING_SERVO_PIN = 10;
const int BUTTON_PIN = 12;
const int LED_PIN = 13;

// Create navigator
Navigator navigator;

// This is called once
void setup() {
  // Start serial at 9600 baud
  Serial.begin(9600);

  // Setup pins correctly
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Attach servos to pins
  navigator.attachSpeedServo(SPEED_SERVO_PIN);
  navigator.attachSteeringServo(STEERING_SERVO_PIN);
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

    // Reset steering first
    navigator.resetSteering();

    // Navigate a pattern
    Position position = Position{0.0, 0.0, 0.0};
    position = navigator.NavigateTo(position, Location{4.0, 0.0});
    position = navigator.NavigateTo(position, Location{6.0, 2.0});
    position = navigator.NavigateTo(position, Location{6.0, 4.0});
    position = navigator.NavigateTo(position, Location{4.0, 6.0});
    position = navigator.NavigateTo(position, Location{2.0, 6.0});
    position = navigator.NavigateTo(position, Location{0, 4.0});
    position = navigator.NavigateTo(position, Location{0, 0});

    // Let them know we finished our task
    digitalWrite(LED_PIN, HIGH);
  }
}

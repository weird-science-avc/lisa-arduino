/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>

const int min_steering = 1150; // all left (going backwards = right)
const int max_steering = 2120; // all right  (going backwards = left)
const int straight_steering = 1608; //1550   (max_steering + min_steering) / 2; 1635 
int motor_speed = 880; // Should be 880 
int distance_to_travel = 30; // in meters
int buttonPin = 12;
int ledPin = 13;
int hallPin = 7;
int hallValue = 0;
Servo firstESC, secondESC; //Create as much as Servoobject you want. You can control 2 or more Servos at the same time

void setup() {
  
  firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
  secondESC.attach(10);
  Serial.begin(9600);    // start serial at 9600 baud
  pinMode(buttonPin, INPUT); 
  pinMode(hallPin, INPUT);
  pinMode(ledPin, OUTPUT); 
  firstESC.writeMicroseconds(0);
  secondESC.writeMicroseconds(straight_steering);
}

void loop() {

  hallValue = digitalRead(hallPin);
  Serial.println(hallValue);
  
  if(digitalRead(buttonPin) == HIGH) {
    Serial.println("Button was pressed!");
    //light LED!!
    digitalWrite(ledPin, HIGH);
    delay(3000);
    test_run();
    motor_speed = motor_speed + 40;
    digitalWrite(ledPin, LOW);
  }
}

void test_run(){
    secondESC.writeMicroseconds(straight_steering);
    firstESC.writeMicroseconds(motor_speed);
    delay((int) get_time_needed((float) motor_speed, (float) distance_to_travel)); // go 30 meters at speed determined above
    firstESC.writeMicroseconds(0);
}

float get_time_needed(float motor_power, float distance_meters) {
  float meters_per_second = (motor_power - 659.19)/95.964;
  return distance_meters/meters_per_second;
}

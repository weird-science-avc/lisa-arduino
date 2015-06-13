/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>

const int min_steering = 1150; // all left
const int max_steering = 2120; // all right
const int straight_steering = (max_steering + min_steering) / 2;
int value = 690; // set values you need to zero
int buttonPin = 12;
int ledPin = 13;
int hallPin = 7;
int hallValue = 0;
Servo firstESC, secondESC; //Create as much as Servoobject you want. You can control 2 or more Servos at the same time

void setup() {
  
  firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
  secondESC.attach(10);    // attached to pin 9 I just do this with 1 Servo
  Serial.begin(9600);    // start serial at 9600 baud
  pinMode(buttonPin, INPUT); 
  pinMode(hallPin, INPUT);
  pinMode(ledPin, OUTPUT); 
  firstESC.writeMicroseconds(0);
  secondESC.writeMicroseconds(straight_steering);
}

void loop() {
  // First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
//  Serial.println(value);
//  if(Serial.available())
//    code = Serial.parseInt();    // Parse an Integer from Serial
//    if(code == 12345)
//      test_run();
  
  
  
  hallValue = digitalRead(hallPin);
  Serial.println(hallValue);
//  delay(30);
  
  
  
//  if(digitalRead(buttonPin) == HIGH) {
//    Serial.println("Button was pressed!");
//    //light LED!!
//    digitalWrite(ledPin, HIGH);
//    delay(3000);
//    test_run();
//    digitalWrite(ledPin, LOW);
//  }
}

void test_run(){
  int halfway_right = ((max_steering - straight_steering) / 2) + straight_steering;
  Serial.println("setting steering");
  Serial.println(halfway_right);
  secondESC.writeMicroseconds(halfway_right);
  //start up
  for (int i = 0; i < 4; i++) {
    Serial.println(value);
    firstESC.writeMicroseconds(value);
    delay(1000);
    value = value + 10;
  }
  
  int halfway_left = ((min_steering - straight_steering) / 2) + straight_steering;
  Serial.println("setting steering");
  Serial.println(halfway_left);
  secondESC.writeMicroseconds(halfway_left); // halfway left
  //slow down
  for (int i = 0; i < 3; i++) {
    Serial.println(value);
    firstESC.writeMicroseconds(value);
    delay(1000);
    value = value - 10;
  }
  Serial.println("Stop");
  firstESC.writeMicroseconds(400);
  secondESC.writeMicroseconds((max_steering + min_steering) / 2);
}

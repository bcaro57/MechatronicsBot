#include <Arduino.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include "pindefs.h"
#include "functions.h"

void setup() {
  // turn on the serial monitor
  Serial.begin(9600);

  // initialize the LEDs that we use as outputs
  pinMode(ledPinRed, OUTPUT);    
  pinMode(ledPinGreen, OUTPUT); 
  pinMode(ledPinBlue, OUTPUT);
  pinMode(ledPinWhite, OUTPUT);
  // initialize the IR sensors
  pinMode(middleFloorSensor, INPUT);
  pinMode(frontIRSensor, INPUT);
  pinMode(LeftIRSensor, INPUT);
  pinMode(RightIRSensor, INPUT);
  // initialize the button
  pinMode(buttonPin, INPUT);  
  // initialize the ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Setting up the servo driver for the two drive wheels and the ladder
  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);  
  delay(10);

  // stop the motors on startup
  servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
  servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);

  // stay in a loop until the on board button is pressed
  while (buttonPressed == false){
    buttonState();
  }
  // complete a homing sequence and flash all lights when done 
  homingSequence();
  digitalWrite(ledPinRed, HIGH); digitalWrite(ledPinGreen, HIGH); digitalWrite(ledPinBlue, HIGH); digitalWrite(ledPinWhite, HIGH);
  delay(250);
  digitalWrite(ledPinRed, LOW); digitalWrite(ledPinGreen, LOW); digitalWrite(ledPinBlue, LOW); digitalWrite(ledPinWhite, LOW);
  delay(250);
  digitalWrite(ledPinRed, HIGH); digitalWrite(ledPinGreen, HIGH); digitalWrite(ledPinBlue, HIGH); digitalWrite(ledPinWhite, HIGH);
  delay(250);
  digitalWrite(ledPinRed, LOW); digitalWrite(ledPinGreen, LOW); digitalWrite(ledPinBlue, LOW); digitalWrite(ledPinWhite, LOW);
  delay(250);
}

/*
this loop switches the algorithm on or off based on if the button is pressed. 

the algorithm will drive up and down each column (which we have denoted as the y-axis), and when it reaches a border, 
it switches to the next row (which we have denoted as the x-axis). when a box is found, it runs a predescribed path
that will check each side of the box for an IR sensor. if that is found, it runs through a reorientation procedure
and hits the box to turn off the signal.
*/
void loop() {
  currentTime = millis();
  buttonState();

  if (buttonPressed == true) {
    smartSteering();
    if (countLines() == -1){
      backUp();
      if (orientation == 'F'){
        goToPosition(currentX + 1, currentY);
        turnRight();
      }
      else if (orientation == 'B'){
        goToPosition(currentX + 1, currentY);
        turnLeft();
      }
    }
    boxInFront();
  }
  else {
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
  }
}
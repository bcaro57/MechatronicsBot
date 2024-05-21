#include <Arduino.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include "pindefs.h"
#include "functions.h"

void setup() {
  // Pin initialization
  pinMode(ledPinRed, OUTPUT);    // initialize the red LED pin as an output
  pinMode(ledPinGreen, OUTPUT);  // initialize the green LED pin as an output
  pinMode(ledPinBlue, OUTPUT);
  pinMode(ledPinWhite, OUTPUT);
  pinMode(middleFloorSensor, INPUT);  // initialize the IR sensor pin as an input
  pinMode(frontIRSensor, INPUT);  // initialize the IR sensor pin as an input
  pinMode(LeftIRSensor, INPUT);
  pinMode(RightIRSensor, INPUT);
  pinMode(buttonPin, INPUT);      // initialize the button pin as an input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //myServo.attach(ladderServoPin);

  // Radio setup

  Serial.begin(9600);
  if (!radio.begin()) {
    //Serial.println(("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
 // radio.openWritingPipe(addresses[0]); // 00001
 radio.openReadingPipe(0, address); // 00002 //radio.openReadingPipe(1, addresses[1]); // 00002
 radio.setPALevel(RF24_PA_HIGH);

  //myServo.write(0);

  // Servo setup
  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);  
  delay(10);

  servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
  servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
  //Serial.println("Ready!");
  while (buttonPressed == false){
    buttonState();
  }
  homingSequenceSimple();
  digitalWrite(ledPinRed, HIGH); digitalWrite(ledPinGreen, HIGH); digitalWrite(ledPinBlue, HIGH); digitalWrite(ledPinWhite, HIGH);
  delay(250);
  digitalWrite(ledPinRed, LOW); digitalWrite(ledPinGreen, LOW); digitalWrite(ledPinBlue, LOW); digitalWrite(ledPinWhite, LOW);
  delay(250);
  digitalWrite(ledPinRed, HIGH); digitalWrite(ledPinGreen, HIGH); digitalWrite(ledPinBlue, HIGH); digitalWrite(ledPinWhite, HIGH);
  delay(250);
  digitalWrite(ledPinRed, LOW); digitalWrite(ledPinGreen, LOW); digitalWrite(ledPinBlue, LOW); digitalWrite(ledPinWhite, LOW);
  delay(250);
}

void loop() {

  currentTime = millis();
  buttonState();
  //transcieveData();
  // calibration sequence - only used prior to actual testing

  // servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
  // for (int i=1410;i<1510;i++){
  //   servo.writeMicroseconds(leftServoPin, i);
  //   Serial.println(i);
  //   delay(1000);
  // }


  //if (buttonPressed == true) {
   digitalWrite(ledPinWhite, LOW);
    smart_steering();
    if (countLines() == -1){
      back_up();
      if (Orientation == 'F'){
        Go_to_Position(Current_X + 1, Current_Y, true);
        turn_right();
      }
      else if (Orientation == 'B'){
        Go_to_Position(Current_X + 1, Current_Y, true);
        turn_left();
      }
    }
    box_in_front();
  //}
  //else{
    //int speed = 75;
    //servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + map(data.leftJoystick, 0, 1023, -speed, speed));
    //servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - map(data.rightJoystick, 0, 1023, -speed, speed));
    //myServo.write(map(data.servoJoystick, 0, 1023, 90, 180));
  //}
}
#include <Arduino.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include "pindefs.h"
#include "functions.h"

Servo myServo;

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

  myServo.attach(A1);

  // Radio setup

  Serial.begin(9600);
  if (!radio.begin()) {
    Serial.println(("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_HIGH);

  // Servo setup
  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);  
  delay(10);

  servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
  servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
  Serial.println("Ready!");
  while (buttonPressed == false){
    buttonState();
  }
  //homingSequence();
  turn_left();
  delay(500);
  turn_right();
  delay(500);
  Move_forward_once();
  delay(1000);
  back_up();
  //Go_to_Position(3, 0);
  //delay(500);
  //Go_to_Position(3, 3);
  
}

void loop() {

  currentTime = millis();
  buttonState();
  // calibration sequence - only used prior to actual testing

  // servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
  // for (int i=1410;i<1510;i++){
  //   servo.writeMicroseconds(leftServoPin, i);
  //   Serial.println(i);
  //   delay(1000);
  // }


  if (buttonPressed == true) {
    transcieveData();
    digitalWrite(ledPinWhite, LOW);
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    myServo.write(90);
  }
  else{
    transcieveData();
    digitalWrite(ledPinWhite, HIGH);
    int speed = 75;
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + map(data.leftJoystick, 0, 1023, -speed, speed));
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - map(data.rightJoystick, 0, 1023, -speed, speed));
    myServo.write(map(data.servoJoystick, 0, 1023, 90, 180));
  }
}
#include <Arduino.h>
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
  pinMode(middleFloorSensor, INPUT);  // initialize the IR sensor pin as an input
  pinMode(frontIRSensor, INPUT);  // initialize the IR sensor pin as an input
  pinMode(LeftIRSensor, INPUT);
  pinMode(RightIRSensor, INPUT);
  pinMode(buttonPin, INPUT);      // initialize the button pin as an input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Radio setup

  Serial.begin(9600);
  if (!radio.begin()) {
    Serial.println(("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  radio.openReadingPipe(0, address); // 00002
  radio.setPayloadSize(sizeof(Data_Package)); 
  radio.setPALevel(RF24_PA_MIN);

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

   
  // moveMotors(data.Lefty,data.Righty);

  if (buttonPressed == true) {
    // Serial.println(countLines());
     //detectBox();
   // Lawn_Mow_Alogrithm();
   //receiveData();
   //moveMotors(data.Lefty,data.Righty);
   smart_steering();
   char set_off=detectFire();
   Put_out_Fire(set_off);
  //box_in_front();
  countLines();
  }
  else{
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);

    Serial.print(Current_X);
    Serial.print("   ");
    Serial.print(Current_Y);
    Serial.print("   ");
    Serial.println(Orientation);

  }
}
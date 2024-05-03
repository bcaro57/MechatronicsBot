#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>

#include "pindefs.h"


// These variables were determined based on some testing of the motors
int left_LowerStopLimit = 1438;
int left_UpperStopLimit = 1487;
int right_LowerStopLimit = 1434;
int right_UpperStopLimit = 1482;

// These give us our "stopped speed" for the left and right servo, which allows us to fine tune the straight driving capabilities
int left_StoppedSpeed = (left_UpperStopLimit + left_LowerStopLimit)/2;
int right_StoppedSpeed = (right_UpperStopLimit + right_LowerStopLimit)/2;

/*
'runSpeed' is how fast we are running our servos. The units are arbitrary, but they add/subtract to the StoppedSpeed 
variables to create forward and backward motion (which is written in microseconds of pwm signal). 'rightSpeed' and 
'leftSpeed' are fed to the motors for how fast they should go
*/
int runSpeed = 120;
int rightSpeed = left_StoppedSpeed;
int leftSpeed = right_StoppedSpeed;

// 'currentTime' keeps the time of our system
unsigned long currentTime = 0;

// Servo initialization
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Radio initialization
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

/* 
This is the package of data from our controller. It includes 'Lefty' and 'Righty', which are values
that control the speed of our two wheels given to us by the controller. It also has 'Single', 'Double', 
and 'border', which are updated by the robot and seen on our controller.

The max size of this struct is 32 bytes - that is the NRF24L01 buffer limit.
*/
struct Data_Package {
  int Lefty = 0;
  int Righty = 0;
  int Single = 0;
  int Double = 0;
  int border = 0;
};

// Create a variable with the above structure
Data_Package data;

// Initializing our functions 
void receiveData();
void moveMotors(int leftInput,int rightInput);
void detectFire();
void countLines();


void setup() {
  // Pin initialization
  pinMode(ledPinRed, OUTPUT);    // initialize the red LED pin as an output
  pinMode(ledPinGreen, OUTPUT);  // initialize the green LED pin as an output
  pinMode(middleFloorSensor, INPUT);  // initialize the IR sensor pin as an input
  pinMode(frontIRSensor, INPUT);  // initialize the IR sensor pin as an input

  // Radio setup
  Serial.begin(9600);
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
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
}

void loop() {
  currentTime = millis();
  // calibration sequence - only used prior to actual testing

  // pwm.writeMicroseconds(9, 1444);
  // for (int i=1420;i<1490;i++){
  //   pwm.writeMicroseconds(10, i);
  //   Serial.println(i);
  //   delay(500);
  // }

  receiveData();
  moveMotors(data.Lefty,data.Righty);
  detectFire();
  countLines();
}

void receiveData(){
  delay(5);
  radio.startListening();
  if (radio.available()) {
    while (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    // Serial.print("Lefty: ");
    // Serial.println(data.Righty);
    }
  }
}

/*
This function is used for us to control the speeds of our two wheels, labeled as left and right.
*/ 
void moveMotors(int leftInput,int rightInput){
  // augment the right wheel speed according to the input
  if (leftInput>700){
    leftSpeed = left_StoppedSpeed + runSpeed;
  }
  else if (leftInput<200){
    leftSpeed = left_StoppedSpeed - runSpeed;
  }
  else{
    leftSpeed = left_StoppedSpeed;
  }
  
  // augment the right wheel speed according to the input
  if (rightInput>700){
    rightSpeed = right_StoppedSpeed - runSpeed;
  }
  else if (rightInput<200){
    rightSpeed = right_StoppedSpeed + runSpeed;
  }
  else{
    rightSpeed = right_StoppedSpeed;
  }

  // actuate the servo motors
  servo.writeMicroseconds(leftServoPin, leftSpeed);
  servo.writeMicroseconds(rightServoPin, rightSpeed);
}

unsigned long fireTimer = 0;
int upPosition = 1000;
int downPosition = 1900;
/*
This function is used for us to detect the IR signal from the 'fire', and respond accordingly by moving our 'ladder'.
*/ 
void detectFire(){
  int fireState = digitalRead(frontIRSensor);
  if(fireState == 0){
    fireTimer = millis();
  }

  if( (currentTime - fireTimer) < 1000){
    servo.writeMicroseconds(ladderServoPin, downPosition);
  }
  else{
    servo.writeMicroseconds(ladderServoPin, upPosition);
  }
  // Serial.println((currentTime - fireTimer));
}

unsigned long lineTimer = 0;
unsigned long ledBlinkingTimer = 0;
bool currentlyCounting = false;
bool hasSeenLine = false;
int lineCount = 0;
/*
This function is used for us to count the lines and determine whether they are a double or a single line.
*/ 
void countLines(){
  if (digitalRead(middleFloorSensor) == LOW && !hasSeenLine) {
    currentlyCounting = true;
    lineTimer = millis();
    hasSeenLine = true;
  }
  if (digitalRead(middleFloorSensor) == HIGH && hasSeenLine){
    lineCount += 1;
    hasSeenLine = false;
    delay(50);
  }

  if (currentTime - lineTimer > 750){
    currentlyCounting = false;
  }

  // // after leaving the counting state, we write our LED's according to the count, and allow them to shine for half a second. after that, they are all turned off (when the count returns to 0).
  if (lineCount == 1 && !currentlyCounting) {
    digitalWrite(ledPinRed, HIGH);
    ledBlinkingTimer = millis();
    Serial.println("we hit for red");
    lineCount = 0;
  }
  else if (lineCount >= 2 && !currentlyCounting) {
    digitalWrite(ledPinGreen, HIGH);
    ledBlinkingTimer = millis();
    Serial.println("we hit for green");
    lineCount = 0;
  }
  else if (!currentlyCounting){
    lineCount = 0;
  }

  if (currentTime - ledBlinkingTimer > 500 && lineCount == 0){
    digitalWrite(ledPinRed, LOW);
    digitalWrite(ledPinGreen, LOW);
  }
 }
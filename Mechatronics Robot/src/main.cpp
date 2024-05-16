#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>

#include "pindefs.h"
//Ultra sonic Stuff
long duration;
double distance;

// These variables were determined based on some testing of the motors
int left_LowerStopLimit = 1434;
int left_UpperStopLimit = 1496;
int right_LowerStopLimit = 1428;
int right_UpperStopLimit = 1487;

// These give us our "stopped speed" for the left and right servo, which allows us to fine tune the straight driving capabilities
int left_StoppedSpeed = (left_UpperStopLimit + left_LowerStopLimit)/2;
int right_StoppedSpeed = (right_UpperStopLimit + right_LowerStopLimit)/2;

/*
'runSpeed' is how fast we are running our servos. The units are arbitrary, but they add/subtract to the StoppedSpeed 
variables to create forward and backward motion (which is written in microseconds of pwm signal). 'rightSpeed' and 
'leftSpeed' are fed to the motors for how fast they should go
*/
int runSpeed = 80;
int rightSpeed = left_StoppedSpeed;
int leftSpeed = right_StoppedSpeed;

// 'currentTime' keeps the time of our system
long currentTime = 0;

long lineTimer = 0;
long ledBlinkingTimer = 0;
bool currentlyCounting = false;
bool hasSeenLine = false;
bool outOfBounds = false;
bool printBoundaryError = false;
int lineCount = 0;

long fireTimer = 0;
int upPosition = 1000;
int downPosition = 1900;

bool buttonPressed = false;

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
int countLines();
void homingSequence();
void buttonState();
double detectBox();
void turn_left();
void turn_right();
void smart_steering();

void setup() {
  // Pin initialization
  pinMode(ledPinRed, OUTPUT);    // initialize the red LED pin as an output
  pinMode(ledPinGreen, OUTPUT);  // initialize the green LED pin as an output
  pinMode(ledPinBlue, OUTPUT);
  pinMode(middleFloorSensor, INPUT);  // initialize the IR sensor pin as an input
  pinMode(frontIRSensor, INPUT);  // initialize the IR sensor pin as an input
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

  // receiveData();
  // moveMotors(data.Lefty,data.Righty);

  if (buttonPressed == true) {
    // Serial.println(countLines());
     //detectBox();
  }
  smart_steering();
  
  if(countLines()==-1){
    turn_right();
  }

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

/*
This function is used for us to count the lines and determine whether they are a double or a single line.

the return statment works as follows -- 
  0: no lines have been counted
  1: one line has been counted
  2+: two lines have been counted
  -1: we are out of bounds
*/ 
int countLines(){
  if (digitalRead(middleFloorSensor) == LOW && !hasSeenLine) {
    currentlyCounting = true;
    lineTimer = millis();
    hasSeenLine = true;
  }
  if (digitalRead(middleFloorSensor) == HIGH && hasSeenLine){
    if (!outOfBounds){
      lineCount += 1;
      hasSeenLine = false;
      delay(50);
    }
    else{
      digitalWrite(ledPinBlue, LOW);
      outOfBounds = false;
      hasSeenLine = false;
    }
  }

  if (currentTime - lineTimer > 400){
    if(lineCount == 0 && currentlyCounting){
      outOfBounds = true;
      printBoundaryError = true;
    }
    currentlyCounting = false;
  }
  
  if (outOfBounds){
    digitalWrite(ledPinBlue, HIGH); // for some reason this LED is dim with a digitalWrite, but this makes it much brighter
    lineCount = -1;
    if(printBoundaryError){
      Serial.println("we are out of bounds");
      printBoundaryError = false;
    }
    return lineCount;
  }
  // after leaving the counting state, we write our LED's according to the count, and allow them to shine for half a second. after that, they are all turned off (when the count returns to 0).
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
  return lineCount;
}

/* 
This function is used to read the button state and alter the buttonPressed flag depending upon
if the button has been pressed previously or not.
*/
void buttonState() {

  // Sets buttonPressed to true if the button is pressed and previously was false
  if (digitalRead(buttonPin) == HIGH && buttonPressed == false) {
    Serial.println("Button Pressed - Begining Control Loop");
    delay(500);
    Serial.print("Initial State: ");
    Serial.print(buttonPressed);
    buttonPressed = true;
    Serial.print(", Final State: ");
    Serial.println(buttonPressed);
  } // Sets buttonPressed to false if the button is pressed and previously was true
  else if (digitalRead(buttonPin) == HIGH && buttonPressed == true) {
    Serial.println("Button Pressed - Pausing Control Loop");
    delay(500);
    Serial.print("Initial State: ");
    Serial.print(buttonPressed);
    buttonPressed = false;
    Serial.print(", Final State: ");
    Serial.println(buttonPressed);
  }

}

#define SYSTEM_START 0
#define DETECT_BOTTOM_EDGE 1
#define FIRST_BACK_UP 2
#define FIRST_TURN_LEFT 3
#define DETECT_LEFT_EDGE 4
#define SECOND_BACK_UP 5
#define SECOND_TURN_LEFT 6
#define ORIENT_FINAL 7

bool calibrated = false;
int slowSpeed = 50;
int calibrationState = SYSTEM_START;
long leftTurnTime = 0;
long reverseTime = 0;
long turnLength = 1700;
long backupLength = 1000;
/*
This function currently homes the robot to the bottom left corner. It has to be oriented facing the back out of bounds line to start.
*/
void homingSequence(){
  Serial.println("starting homing sequence");
  while (!calibrated){
    currentTime = millis();
    switch(calibrationState){
      case SYSTEM_START:
        // initialize our motors to be stopped
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
        delay(2000);
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - slowSpeed);
        calibrationState = DETECT_BOTTOM_EDGE;
        break;
      case DETECT_BOTTOM_EDGE:
        // drive and check for the bottom edge
        
        if (countLines() == -1){
          servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
          servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
          delay(2000);
          Serial.println("now backing up");
          reverseTime = millis();
          calibrationState = FIRST_BACK_UP;
        }
        // could make it more robust by adding logic for passing a single line or a double line
        else if (countLines() == 1){
          // turn 180
          Serial.print("");
        }
        else if (countLines() == 2){
          // turn 90
          Serial.print("");
        }
        break;
      case FIRST_BACK_UP:
      // back up
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed - slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        if(currentTime - reverseTime > backupLength){
          servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
          servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
          delay(1000);
          Serial.println("now turning");
          leftTurnTime = millis();
          calibrationState = FIRST_TURN_LEFT;
        }
        break;
      case FIRST_TURN_LEFT:
        // turn left
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        if (currentTime - leftTurnTime > turnLength){
          servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
          servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
          delay(1000);
          Serial.println("now looking for the left edge");
          calibrationState = DETECT_LEFT_EDGE;
        }
        break;
      case DETECT_LEFT_EDGE:
        // drive until the left edge is detected
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - slowSpeed);
        if (countLines() == -1){
          servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
          servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
          delay(1000);
          Serial.println("now backing up");
          reverseTime = millis();
          calibrationState = SECOND_BACK_UP;
        }
        break;
      case SECOND_BACK_UP:
      // back up
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed - slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        if(currentTime - reverseTime > backupLength){
          servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
          servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
          delay(1000);
          Serial.println("now turning");
          leftTurnTime = millis();
          calibrationState = SECOND_TURN_LEFT;
        }
        break;
      case SECOND_TURN_LEFT:
        // turn left
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        if (currentTime - leftTurnTime > turnLength){
          servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
          servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
          delay(500);
          Serial.println("finished orienting");
          calibrationState = ORIENT_FINAL;
        }
        break;
      case ORIENT_FINAL:
        // orient the robot to face forwar
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
        for (int i = 0; i < 3; i++){
          digitalWrite(ledPinRed, HIGH);
          digitalWrite(ledPinGreen, HIGH);
          digitalWrite(ledPinBlue, HIGH);
          delay(300);
          digitalWrite(ledPinRed, LOW);
          digitalWrite(ledPinGreen, LOW);
          digitalWrite(ledPinBlue, LOW);
          delay(300);
        } 
        Serial.println("finished homing sequence :)");
        calibrated = true;
        break;
    }
  }
}


double detectBox(){
  duration=0;
  int compare=0;
  for (int i=0;i<10;i++){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds

      compare = pulseIn(echoPin, HIGH);
      if(compare>duration){
        duration=compare;
      }
    
  // Calculating the distance
    distance = duration * 0.034 / 2*0.393701;
  }
    Serial.print("Distance: ");
    Serial.println(distance);

  if (distance > 20.0){
      digitalWrite(ledPinRed,HIGH);
      digitalWrite(ledPinGreen,HIGH);
      digitalWrite(ledPinBlue, HIGH);
    }
  else if (distance <= 20.0 && distance > 10.0){
      digitalWrite(ledPinRed,HIGH);
      digitalWrite(ledPinGreen,HIGH);
      digitalWrite(ledPinBlue,LOW);
    }
  else{
      digitalWrite(ledPinRed,HIGH);
      digitalWrite(ledPinGreen,LOW);
      digitalWrite(ledPinBlue,LOW);
    }
    return distance;
}

void turn_right(){
  currentTime = millis();
  leftTurnTime = millis();
      while(currentTime -leftTurnTime < turnLength){
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        currentTime = millis();
}
          servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
          servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
          delay(500);
}

void turn_left(){
  currentTime = millis();
  leftTurnTime = millis();
      while(currentTime -leftTurnTime < turnLength){
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed - slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - slowSpeed);
        currentTime = millis();
}
          servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
          servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
          delay(500);
}
void smart_steering(){
  if(digitalRead(rightFloorSensor)==0){
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - runSpeed);
    Serial.println("right trip");
    
  }
  else if(digitalRead(leftFloorSensor)==0){
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + runSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    Serial.println("LEFT trip");
  }
  else{
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + runSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - runSpeed);
    Serial.println("NO trip");
  }

}
void back_up(){
    currentTime = millis();
    long back_up_time  = millis();
    while(currentTime -back_up_time < 1000){
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed - slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        currentTime = millis();
}
}
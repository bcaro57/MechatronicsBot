#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include "pindefs.h"
#include "variables.h"

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

void turnLeft();
void turnRight();
void backUp();
void forward();
void setOrientation(char desiredOrientation);
int goToPosition(int desiredX, int desiredY);
void updateOrientation(char currentOrientation, char rotation);
void updatePosition(int lines);
int countLines();
void buttonState();
void homingSequence();
double detectBox();
void smartSteering();
int putOutFire();
void boxInFront();

/*
this function helps us turn left
*/
void turnLeft() {
    currentTime = millis();
    leftTurnTime = millis();
    while (currentTime - leftTurnTime < turnLength) {
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed - slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - slowSpeed);
        currentTime = millis();
    }
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(200);
    updateOrientation(orientation,'L');
}

/*
this function helps us turn right
*/
void turnRight() {
    currentTime = millis();
    leftTurnTime = millis();
    while (currentTime - leftTurnTime < turnLength) {
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        currentTime = millis();
    }
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(200);
    updateOrientation(orientation,'R');
}

/*
this function drives us backward
*/
void backUp() {
    currentTime = millis();
    backUpTime  = millis();
    while (currentTime - backUpTime < backupLength) {
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed - slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        currentTime = millis();
    }
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(500);
}

/*
this function drives us forward
*/
void forward() {
    currentTime = millis();
    backUpTime = millis();
    while (currentTime - backUpTime < forwardLength) {
      smartSteering();
        currentTime = millis();
    }
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(500);
}
/*
this function sets out orientation to be correct
*/
void setOrientation(char desiredOrientation){
  if (orientation == 'F') {
      if (desiredOrientation == 'L'){
        turnLeft();
      }
      else if (desiredOrientation == 'R'){
        turnRight();
      }
      else if (desiredOrientation == 'B'){
      turnRight();
      turnRight();
      }
}
  else if (orientation=='B') {
      if(desiredOrientation=='R'){
        turnLeft();
      }
      else if(desiredOrientation=='L'){
        turnRight();
      }
      else if(desiredOrientation=='F'){
      turnRight();
      turnRight();
      }
}
  else if (orientation=='L') {
      if(desiredOrientation=='B'){
        turnLeft();
      }
      else if(desiredOrientation=='F'){
        turnRight();
      }
      else if(desiredOrientation=='R'){
      turnRight();
      turnRight();
      }
}
  else if (orientation=='R') {
      if(desiredOrientation=='F'){
        turnLeft();
      }
      else if(desiredOrientation=='B'){
        turnRight();
      }
      else if (desiredOrientation=='L'){
      turnRight();
      turnRight();
      }
}
}

/*
this function allows us to go to any x,y position, moving first in x and then in y
*/
int goToPosition(int desiredX, int desiredY) {
    int errorX = desiredX - currentX;
    int errorY = desiredY - currentY;
    if (errorX < 0) {
      setOrientation('L');
      while (currentX != desiredX || lines == -1){
        currentTime = millis();
        smartSteering();
        lines = countLines();
        if (putOutFire() == 1){
          return 1;
        }
      }
    forward();
    } 
    else if (errorX>0) {
      setOrientation('R');
      while (currentX != desiredX || lines == -1){
        currentTime = millis();
        smartSteering();
        lines = countLines();
        if(putOutFire() == 1){
          return 1;
        }
      }
    forward();
    }
    
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    if(errorY < 0) {
        setOrientation('B');
        while(currentY != desiredY || lines == -1){
          currentTime = millis();
          smartSteering();
          lines = countLines();
          if(putOutFire() == 1){
            return 1;
          }
        }
    forward();
    }
    else if (errorY>0) {
        setOrientation('F');
        while(currentY != desiredY || lines == -1){
          currentTime = millis();
          smartSteering();
          lines = countLines();
          if(putOutFire() == 1){
            return 1;
          }
        }
    forward();
    }
    
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    return 0;
}

/*
this function updates our orientation depending on how we rotated
*/
void updateOrientation(char currentOrientation, char rotation) {
    if (currentOrientation == 'F') {
        if (rotation == 'L') {
            orientation = 'L';
        } else if (rotation == 'R') {
            orientation = 'R';
        }
    } 
    else if (currentOrientation == 'B') {
        if (rotation == 'L') {
            orientation = 'R';
        }
        else if (rotation == 'R') {
            orientation = 'L';
        }
    }
    else if (currentOrientation == 'L') {
        if (rotation == 'L') {
            orientation = 'B';
        }
        else if (rotation == 'R') {
            orientation = 'F';
        }
    }
    else if (currentOrientation == 'R') {
        if (rotation == 'L') {
            orientation = 'F';
        }
        else if (rotation == 'R') {
            orientation = 'B';
        }
    }
}

/*
This function updates our x,y position based on the line number from the countLines equation
*/
void updatePosition(int lines) {
    if (lines == 1 && orientation == 'L') {
      currentX = currentX - 1;
    }
    else if (lines == 1 && orientation == 'R') {
      currentX = currentX + 1;
    }
    else if (lines == 2 && orientation == 'F') {
      currentY = currentY + 1;
    }
    else if (lines == 2 && orientation == 'B') {
      currentY = currentY - 1;
    }
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
    // check if the middle floor sensor sees a line, and it has not previously seen one, and starts counting
    if (digitalRead(middleFloorSensor) == LOW && !hasSeenLine) {
        currentlyCounting = true;
        lineTimer = millis();
        hasSeenLine = true;
    }
    // when the sensor turns off, it adds one to the count of lines if it's wasn't out of bounds. 
    // if it was out of bounds, it resets itself and and turns the indicator led off
    if (digitalRead(middleFloorSensor) == HIGH && hasSeenLine) {
        lineTimer = millis();
        if (!outOfBounds) {
            lineCount += 1;
            hasSeenLine = false;
            delay(50);
        }
        else {
            digitalWrite(ledPinBlue, LOW);
            outOfBounds = false;
            hasSeenLine = false;
        }
    }
    
    if (currentTime - lineTimer > 375) {
        if (lineCount == 0 && currentlyCounting) {
            outOfBounds = true;
            printBoundaryError = true;
        }
        currentlyCounting = false;
    }
    
    if (outOfBounds) {
        digitalWrite(ledPinBlue, HIGH); // for some reason this LED is dim with a digitalWrite, but this makes it much brighter
        lineCount = -1;
        if (printBoundaryError) {
            //Serial.println("we are out of bounds");
            printBoundaryError = false;
        }
        return lineCount;
    }

    // after leaving the counting state, we write our LED's according to the count, and allow them to shine for half a second. after that, they are all turned off (when the count returns to 0).
    if (lineCount == 1 && !currentlyCounting) {
        digitalWrite(ledPinRed, HIGH);
        ledBlinkingTimer = millis();
        //Serial.println("we hit for red");
        updatePosition(1);
        lineCount = 0;
    }
    else if (lineCount >= 2 && !currentlyCounting) {
        digitalWrite(ledPinGreen, HIGH);
        ledBlinkingTimer = millis();
        //Serial.println("we hit for green");
        updatePosition(2);
        lineCount = 0;
    }
    else if (!currentlyCounting){
        lineCount = 0;
    }
  
    if (currentTime - ledBlinkingTimer > 500 && lineCount == 0) {
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
    delay(500);
    buttonPressed = true;
  } // Sets buttonPressed to false if the button is pressed and previously was true
  else if (digitalRead(buttonPin) == HIGH && buttonPressed == true) {
    delay(500);
    buttonPressed = false;
  }

}

/*
This function currently homes the robot to the bottom left corner. It has to be oriented facing the back out of bounds line to start.
*/
void homingSequence(){
  currentTime = millis();
  turnLeft();
  backUp();
  currentTime = millis();
  lines=countLines();
  while(lines != -1){
    currentTime = millis();
    smartSteering();
    lines = countLines();
  }
  backUp();
  turnRight();
  backUp();
  currentX = 0;
  currentY = 0;
}
/*
this function is used to detect the box
*/
double detectBox() {
   duration = 0;
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(echoPin, HIGH);
        distance = duration * 0.034 / 2*0.393701;
        return distance;
}
/*
this function allows us to steer straight with a bang-bang type approach
*/
void smartSteering() {
    if (digitalRead(middleFloorSensor)==1) {
        if (digitalRead(rightFloorSensor)==0) {
            servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
            servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - runSpeed);
        } 
        else if (digitalRead(leftFloorSensor)==0) {
            servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + runSpeed);
            servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
        }
        else {
            servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + runSpeed);
            servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - runSpeed);
        }
    }
    else {
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + runSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - runSpeed);
    }
}
/*
this function is the algorithm for what happens when a fire is detected
*/
int putOutFire(){
  if (digitalRead(LeftIRSensor) == 0){
    turnLeft();
    double distanceToBox = 100;
    while (distanceToBox > 2) {
      distanceToBox = detectBox();
      smartSteering();
    }
    servo.writeMicroseconds(ladderServoPin, 1100);
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(1000);
    servo.writeMicroseconds(ladderServoPin, 500);
    backUp();
    turnRight();
 
    return 1;
   }
  else if (digitalRead(RightIRSensor) == 0) {
    turnRight();
    double distanceToBox = 100;
    while (distanceToBox > 2) {
      distanceToBox = detectBox();
      smartSteering();
    }
    servo.writeMicroseconds(ladderServoPin, 1100);
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(1000);
    servo.writeMicroseconds(ladderServoPin, 500);
    backUp();
    turnLeft();
    
    return 1;
   }
    else if(digitalRead(frontIRSensor)==0){
    double distanceToBox = 100;
    while(distanceToBox > 2){
      distanceToBox = detectBox();
      smartSteering();
    }
    servo.writeMicroseconds(ladderServoPin, 1100);
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(1000);
    servo.writeMicroseconds(ladderServoPin, 500);
    backUp();
   
    return 1;
   }
   else{
    return 0;
   }
}
/*
This function determines what we do when a box is detected in front of us
*/
void boxInFront(){
  double closeToBox = detectBox();
  if(closeToBox <= 5){
    if(orientation=='F'){
      if(goToPosition(currentX - 1, currentY) == 1){
        goToPosition(currentX - 1, currentY);
      }
      if(goToPosition(currentX, currentY + 2) == 1){
        goToPosition(currentX, currentY + 1);
      }
      if(goToPosition(currentX + 2, currentY) == 1){
          goToPosition(currentX + 1, currentY);
        }
      if (goToPosition(currentX, currentY - 1) == 1){
        digitalWrite(ledPinWhite, HIGH);
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
        delay(2000);
      }
      goToPosition(currentX, currentY + 1);
      goToPosition(currentX - 1, currentY);
      turnRight();
    }
    else if(orientation == 'B'){
      if(goToPosition(currentX + 1, currentY) == 1){
        goToPosition(currentX + 1, currentY);
      }
      if(goToPosition(currentX, currentY - 2) == 1){
        goToPosition(currentX, currentY - 1);
      }
      if(goToPosition(currentX - 2, currentY) == 1){
          goToPosition(currentX - 1, currentY);
        }
      if (goToPosition(currentX, currentY + 1) == 1){
        digitalWrite(ledPinWhite, HIGH);
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
        delay(2000);
      }
      goToPosition(currentX, currentY - 1);
      goToPosition(currentX + 1, currentY);
      turnRight();
    }
  }
}
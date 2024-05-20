#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include "pindefs.h"
#include "variables.h"

Servo myServo;

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Radio initialization
RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

/* 
This is the package of data from our controller. It includes 'Lefty' and 'Righty', which are values
that control the speed of our two wheels given to us by the controller. It also has 'Single', 'Double', 
and 'border', which are updated by the robot and seen on our controller.

The max size of this struct is 32 bytes - that is the NRF24L01 buffer limit.
*/

struct Data_Package {
  int leftJoystick = 512;
  int rightJoystick = 512;
  int servoJoystick = 512;

  int xPosition = 0;
  int yPosition = 0;
  char orientation = 'F';
};

// Create a variable with the above structure
Data_Package data;

// Initializing our functions 
void turn_left();
void turn_right();
void back_up();
void forward();
void moveMotors(int leftInput,int rightInput);
int countLines();
void transcieveData();
char detectFire();
void homingSequence();
void buttonState();
double detectBox();
double detectBox_Loop();
void smart_steering();
void update_position(int Lines);
void update_orientation(char current_O, char rotate);
void Lawn_Mow_Alogrithm();
int Go_to_Position(int Desired_X, int Desired_Y, bool backUpNeeded);
int Put_out_Fire();
void Move_forward_once();
void box_in_front();
void set_Orientation(char desired_O);
void turn_left() {
    currentTime = millis();
    leftTurnTime = millis();
    while(currentTime -leftTurnTime < turnLength) {
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed - slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - slowSpeed);
        currentTime = millis();
    }
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(200);
    update_orientation(Orientation,'L');
}

void turn_right() {
    currentTime = millis();
    leftTurnTime = millis();
    while(currentTime -leftTurnTime < turnLength) {
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        currentTime = millis();
    }
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(200);
    update_orientation(Orientation,'R');
}

void back_up() {
    currentTime = millis();
    back_up_time  = millis();
    while(currentTime -back_up_time < backupLength) {
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed - slowSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed + slowSpeed);
        currentTime = millis();
    }
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(500);
}
void forward() {
    currentTime = millis();
    back_up_time  = millis();
    while(currentTime -back_up_time < forwardLength) {
      smart_steering();
        currentTime = millis();
    }
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(500);
}
void set_Orientation(char desired_O){
  if (Orientation=='F') {
      if(desired_O=='L'){
        turn_left();
      }
      else if(desired_O=='R'){
        turn_right();
      }
      else if(desired_O=='B'){
      turn_right();
      turn_right();
      }
}
  else if (Orientation=='B') {
      if(desired_O=='R'){
        turn_left();
      }
      else if(desired_O=='L'){
        turn_right();
      }
      else if(desired_O=='F'){
      turn_right();
      turn_right();
      }
}
  else if (Orientation=='L') {
      if(desired_O=='B'){
        turn_left();
      }
      else if(desired_O=='F'){
        turn_right();
      }
      else if(desired_O=='R'){
      turn_right();
      turn_right();
      }
}
  else if (Orientation=='R') {
      if(desired_O=='F'){
        turn_left();
      }
      else if(desired_O=='B'){
        turn_right();
      }
      else if (desired_O=='L'){
      turn_right();
      turn_right();
      }
}
}

int Go_to_Position(int Desired_X, int Desired_Y, bool backUpNeeded) {
    int error_X=Desired_X-Current_X;
    int error_Y=Desired_Y-Current_Y;
    if (backUpNeeded){
      back_up();
    }
    if (error_X<0) {
      set_Orientation('L');
      while(Current_X!=Desired_X ||Lines==-1){
      currentTime = millis();
      smart_steering();
      Lines=countLines();
      if(Put_out_Fire()==1){
        return 1;
      }
    }
    forward();
    } else if (error_X>0) {
        set_Orientation('R');
        while(Current_X!=Desired_X||Lines==-1){
        currentTime = millis();
        smart_steering();
        Lines=countLines();
        if(Put_out_Fire()==1){
          return 1;
        }
    }
    forward();
    }
    
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    if(error_Y<0) {
        set_Orientation('B');
        while(Current_Y!= Desired_Y|| Lines==-1){
        currentTime = millis();
        smart_steering();
        Lines=countLines();
        if(Put_out_Fire()==1){
          return 1;
        }
    }
    forward();
    }
    else if(error_Y>0) {
        set_Orientation('F');
        Serial.print("orientation");
        Serial.println(Orientation);
        while(Current_Y!= Desired_Y|| Lines==-1){
        currentTime = millis();
        smart_steering();
        Lines=countLines();
        if(Put_out_Fire()==1){
          return 1;
        }
    }
    forward();
    }
    
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    return 0;
}

void update_orientation(char current_O, char rotate) {
    if (current_O=='F') {
        if (rotate=='L') {
            Orientation='L';
        } else if (rotate=='R') {
            Orientation='R';
        }
    } else if (current_O=='B') {
        if (rotate=='L') {
            Orientation='R';
        }
        else if (rotate=='R') {
            Orientation='L';
        }
    }
    else if (current_O=='L') {
        if (rotate=='L') {
            Orientation='B';
        }
        else if (rotate=='R') {
            Orientation='F';
        }
    }
    else if (current_O=='R') {
        if (rotate=='L') {
            Orientation='F';
        }
        else if (rotate=='R') {
            Orientation='B';
        }
    }
}

void update_position(int Lines) {
    if (Lines==1 && Orientation=='L') {
      Current_X=Current_X-1;
    }
    else if (Lines==1 && Orientation=='R') {
      Current_X=Current_X+1;
    }
    else if (Lines==2 && Orientation=='F') {
      Current_Y=Current_Y+1;
    }
    else if (Lines==2 && Orientation=='B') {
      Current_Y=Current_Y-1;
    }
}

void transcieveData() {
  delay(2);
  radio.startListening();
  if(radio.available()) {
    while (radio.available()) {
      radio.read(&data, sizeof(Data_Package));
    }
  }
  else {
  }


  delay(2);
  radio.stopListening();
  countLines();
  data.xPosition = Current_X;
  data.yPosition = Current_Y;
  data.orientation = Orientation;
  radio.write(&data, sizeof(Data_Package));

}

/*
This function is used for us to control the speeds of our two wheels, labeled as left and right.
*/ 
void moveMotors(int leftInput,int rightInput) {
    // augment the right wheel speed according to the input
    if (leftInput>700) {
        leftSpeed = left_StoppedSpeed + runSpeed;
    }
    else if (leftInput<200) {
        leftSpeed = left_StoppedSpeed - runSpeed;
    }
    else {
        leftSpeed = left_StoppedSpeed;
    }

    // augment the right wheel speed according to the input
    if (rightInput>700) {
        rightSpeed = right_StoppedSpeed - runSpeed;
    }
    else if (rightInput<200) {
    rightSpeed = right_StoppedSpeed + runSpeed;
    }
    else {
        rightSpeed = right_StoppedSpeed;
    }

  // actuate the servo motors
  servo.writeMicroseconds(leftServoPin, leftSpeed);
  servo.writeMicroseconds(rightServoPin, rightSpeed);
}


/*
This function is used for us to detect the IR signal from the 'fire', and respond accordingly by moving our 'ladder'.
*/ 
/*
char detectFire_prev() {
    int fireStateForward = digitalRead(frontIRSensor);
    int fireStateLeft = digitalRead(frontIRSensor);
    int fireStateRight = digitalRead(frontIRSensor);
    if (fireStateForward == 0) {
        fireTimerForward = millis();
    }
    if (fireStateLeft == 0) {
        fireTimerLeft = millis();
    }
    if (fireStateRight == 0) {
        fireTimerRight = millis();
    }

    if( (currentTime - fireTimerForward) < 1000){
        return('f');
    }    if( (currentTime - fireTimerLeft) < 1000){
         return('L');
    }     if( (currentTime - fireTimerRight) < 1000){
         return('R');
    } else {
        servo.writeMicroseconds(ladderServoPin, upPosition);
    }
  // Serial.println((currentTime - fireTimer));
}
*/
char detectFire() {
    int fireStateForward = digitalRead(frontIRSensor);
    int fireStateLeft = digitalRead(LeftIRSensor);
    int fireStateRight = digitalRead(RightIRSensor);
    if (fireStateForward == 0) {
      return 'F';
    }
    if (fireStateLeft == 0) {
        return 'L';
    }
    if (fireStateRight == 0) {
        return 'R';
    }
    return 'O';
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
        update_position(1);
        lineCount = 0;
    }
    else if (lineCount >= 2 && !currentlyCounting) {
        digitalWrite(ledPinGreen, HIGH);
        ledBlinkingTimer = millis();
        Serial.println("we hit for green");
        update_position(2);
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

/*
This function currently homes the robot to the bottom left corner. It has to be oriented facing the back out of bounds line to start.
*/

void homingSequence() {
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

double detectBox() {
    duration=0;
    int compare=0;
    for (int i=0;i<10;i++) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds

        compare = pulseIn(echoPin, HIGH);
        if (compare>duration) {
            duration=compare;
        }
        
        // Calculating the distance
        distance = duration * 0.034 / 2*0.393701;
    }
    Serial.print("Distance: ");
    Serial.println(distance);
    
    if (distance > 20.0) {
        digitalWrite(ledPinRed,HIGH);
        digitalWrite(ledPinGreen,HIGH);
        digitalWrite(ledPinBlue, HIGH);
    } else if (distance <= 20.0 && distance > 10.0) {
      digitalWrite(ledPinRed,HIGH);
      digitalWrite(ledPinGreen,HIGH);
      digitalWrite(ledPinBlue,LOW);
    } else {
      digitalWrite(ledPinRed,HIGH);
      digitalWrite(ledPinGreen,LOW);
      digitalWrite(ledPinBlue,LOW);
    }
    return distance;

}
double detectBox_Loop() {
   duration=0;
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(echoPin, HIGH);
        distance = duration * 0.034 / 2*0.393701;
        return(distance);
}

void smart_steering() {
    if (digitalRead(middleFloorSensor)==1) {
        if (digitalRead(rightFloorSensor)==0) {
            servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
            servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - runSpeed);
            // Serial.println("right trip");  
        } 
        else if (digitalRead(leftFloorSensor)==0) {
            servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + runSpeed);
            servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
            // Serial.println("LEFT trip");
        }
        else {
            servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + runSpeed);
            servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - runSpeed);
            // Serial.println("NO trip");
        }
    }
    else {
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed + runSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed - runSpeed);
    }
}

void Lawn_Mow_Alogrithm() {
    smart_steering();
    Lines=countLines();
    //update_position(Lines);
    if (Lines==-1) {
      back_up();
      turn_right();
      back_up();
    }
}

int Put_out_Fire(){
  if(digitalRead(LeftIRSensor)==0){//( set_off=='L'){
    turn_left();
    double distance2box=100;
    while(distance2box>2){
      distance2box=detectBox_Loop();
      smart_steering();
      Serial.println(distance2box);
    }
    servo.writeMicroseconds(ladderServoPin, downPosition);
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(1000);
    back_up();
    turn_right();
    servo.writeMicroseconds(ladderServoPin, upPosition);
    return 1;
   }
  else if(digitalRead(RightIRSensor)==0){//else if( set_off=='R'){
    turn_right();
    double distance2box=100;
    while(distance2box>2){
      distance2box=detectBox_Loop();
      smart_steering();
      Serial.println(distance2box);
    }
    servo.writeMicroseconds(ladderServoPin, downPosition);
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(1000);
    back_up();
    turn_left();
    servo.writeMicroseconds(ladderServoPin, upPosition);
    return 1;
   }
    else if(digitalRead(frontIRSensor)==0){//else if(set_off=='F'){
    double distance2box=100;
    while(distance2box>2){
      distance2box=detectBox_Loop();
      smart_steering();
      Serial.println(distance2box);
    }
    myServo.write(120);
    servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
    servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
    delay(1000);
    back_up();
    myServo.write(0);
    return 1;
   }
   else{
    return 0;
   }
}

int box_end_y;
void box_in_front(){
  double Close2Box=detectBox_Loop();
  if(Close2Box<=5){
    int box_start_x=Current_X;
    int box_start_y=Current_Y;
    if(Orientation=='F'){
      box_end_y = box_start_y + 2;

      if(Go_to_Position((Current_X-1), Current_Y, true) == 1){
        Go_to_Position((Current_X-1), Current_Y, true);
      }
      if(Go_to_Position((Current_X), Current_Y+2, true) == 1){
        Go_to_Position((Current_X), Current_Y+1, true);
      }
      if(Go_to_Position((Current_X+2), Current_Y, true) == 1){
          Go_to_Position((Current_X+1), Current_Y, true);
        }
      if (Go_to_Position(Current_X, Current_Y-1, true) == 1){
        digitalWrite(ledPinWhite, HIGH);
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
        delay(2000);
      }
      Go_to_Position(Current_X, Current_Y+1, true);
      Go_to_Position(Current_X-1, Current_Y, true);
      turn_right();
    }
    else if(Orientation=='B'){
      box_end_y = box_start_y - 2;

      if(Go_to_Position((Current_X+1), Current_Y, true) == 1){
        Go_to_Position((Current_X+1), Current_Y, true);
      }
      if(Go_to_Position((Current_X), Current_Y-2, true) == 1){
        Go_to_Position((Current_X), Current_Y-1, true);
      }
      if(Go_to_Position((Current_X-2), Current_Y, true) == 1){
          Go_to_Position((Current_X-1), Current_Y, true);
        }
      if (Go_to_Position(Current_X, Current_Y+1, true) == 1){
        digitalWrite(ledPinWhite, HIGH);
        servo.writeMicroseconds(leftServoPin, left_StoppedSpeed);
        servo.writeMicroseconds(rightServoPin, right_StoppedSpeed);
        delay(2000);
      }
      Go_to_Position(Current_X, Current_Y-1, true);
      Go_to_Position(Current_X+1, Current_Y, true);
      turn_right();
    }
  }
}

/*
Description: Used to check if the robot is on a border based on current position
returns:
L=Left Border
R=Right Border
T=Top Border
B=Bottom Border
C=not on borders
*/
char on_border(int curr_x, int curr_y){
  if(curr_x==0 && curr_y>=0 && curr_y<=7){
    return'L';
  }
  else if(curr_x==7 && curr_y>=0 && curr_y<=7){
    return'R';
  }
  else if(curr_y==0 && curr_x>=0 && curr_x<=7){
    return'B';
  }
  else if(curr_y==7 && curr_x>=0 && curr_x<=7){
    return'T';
  }
  else{
    return 'C';
  }
}

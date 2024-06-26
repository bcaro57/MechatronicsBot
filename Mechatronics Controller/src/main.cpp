#include <Arduino.h>
#include <RF24.h>

/*
* Arduino Wireless Communication Tutorial
*     Example 2 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/


#define RADIO_LED 41
#define LEFT_JOYSTICK_PIN A0
#define RIGHT_JOYSTICK_PIN A1
#define SERVO_JOYSTICK_PIN A2


RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  int leftJoystick = 512;
  int rightJoystick = 512;
  int servoJoystick = 512;

  int xPosition = 0;
  int yPosition = 0;
  char orientation = 'F';
};

Data_Package data;

int xPrev = 0;
int yPrev = 0;
char orientationPrev = 'F';

void setup() {
  pinMode(RADIO_LED, OUTPUT);
  pinMode(LEFT_JOYSTICK_PIN, INPUT);
  pinMode(RIGHT_JOYSTICK_PIN, INPUT);
  pinMode(SERVO_JOYSTICK_PIN, INPUT);

  Serial.begin(9600);
  Serial.println("Ready to go!"); Serial.println(); Serial.println();
  
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_HIGH);
}

void loop() {
  /*
  Radio Communication - happens at the beginning of every loop
  */
  delay(2);
  radio.stopListening();
  data.leftJoystick = analogRead(LEFT_JOYSTICK_PIN);
  data.rightJoystick = analogRead(RIGHT_JOYSTICK_PIN);
  data.servoJoystick = analogRead(SERVO_JOYSTICK_PIN);

  // Serial.print("    Right Stick: ");
  // Serial.print(data.leftJoystick);
  // Serial.print("  | Left Stick: ");
  // Serial.print(data.rightJoystick);
  // Serial.print("  | Servo: ");
  // Serial.println(data.servoJoystick);

  radio.write(&data, sizeof(Data_Package));

  delay(2);
  radio.startListening();
  if(radio.available()) {
    digitalWrite(RADIO_LED, HIGH);
  }
  else {
    digitalWrite(RADIO_LED, LOW);
  }
  while (!radio.available());
  radio.read(&data, sizeof(Data_Package));

  if ( (data.xPosition != xPrev) || (data.yPosition != yPrev) || (data.orientation != orientationPrev) ){
    xPrev = data.xPosition;
    yPrev = data.yPosition;
    orientationPrev = data.orientation;

    Serial.print("    x-position: ");
    Serial.print(data.xPosition);
    Serial.print("  | y-position: ");
    Serial.print(data.yPosition);
    Serial.print("  | Orientation: ");
    if(data.orientation == 'F') {
      Serial.println("Forward");
    }
    else if(data.orientation == 'L') {
      Serial.println("Left");
    }
    else if(data.orientation == 'B') {
      Serial.println("Backward");
    }
    else if(data.orientation == 'R') {
      Serial.println("Right");
    }
  }
}
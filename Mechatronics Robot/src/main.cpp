#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>

const int ledPinRed = 11;      // the number of the red LED pin
const int ledPinGreen = 12;    // the number of the green LED pin
const int irSensorPin0 = 36;  // the number of the IR sensor pin
const int right_sensor = 38;
const int left_sensor = 34;
int max_speed = 1720;
int min_speed = 1280;
int L_Lower_limit_stop=1438;
int L_Upper_limit_stop=1487;
int R_Lower_limit_stop=1434;
int R_Upper_limit_stop=1482;
int L_stopped_speed = (L_Upper_limit_stop+L_Lower_limit_stop)/2;
int R_stopped_speed = (R_Upper_limit_stop+R_Lower_limit_stop)/2;
int run_speed = 75; // if 20 or below it will be stopped, and should exceeding 220 has no effec
long timer = 0;
long t = 0;
long t_4_fire = 0;
long led_blink_time = 0;
int count = 0;
bool counting = false;
bool has_seen = false;
int Fire_detector=6;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  int Lefty=90;
  int Righty = 1024;
  int Single = 0;
  int Double = 0;
  int border=0;
};

Data_Package data; //Create a variable with the above structure


void Receive_data();
void Move_Motors(int Left_speed,int Right_speed);
void Detect_Fire();
void counting_lines();


void setup() {
  pinMode(ledPinRed, OUTPUT);    // initialize the red LED pin as an output
  pinMode(ledPinGreen, OUTPUT);  // initialize the green LED pin as an output
  pinMode(irSensorPin0, INPUT);  // initialize the IR sensor pin as an input
  pinMode(Fire_detector, INPUT);  // initialize the IR sensor pin as an input
Serial.begin(9600);
    if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radio.openReadingPipe(0, address); // 00002
  radio.setPayloadSize(sizeof(Data_Package)); 
  radio.setPALevel(RF24_PA_MIN);

  //Added Stuff
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  
  delay(10);

}

void loop() {
  // pwm.writeMicroseconds(9, 1444);
  // for (int i=1420;i<1490;i++){
  //   pwm.writeMicroseconds(10, i);
  //   Serial.println(i);
  //   delay(500);
  // }
//
  Receive_data();
  Move_Motors(data.Lefty,data.Righty);
  counting_lines();
  Detect_Fire();
}

void Receive_data(){
  delay(5);
  radio.startListening();
  if (radio.available()) {
    while (radio.available()) {
  radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
  // Serial.print("Lefty: ");
  // Serial.println(data.Righty);
    }}
}
void Move_Motors(int Left_speed,int Right_speed){
  if (Left_speed>700){
    pwm.writeMicroseconds(9, L_stopped_speed+100);
  }
  else if (Left_speed<200){
    pwm.writeMicroseconds(9, L_stopped_speed-100);
  }
  else{
    pwm.writeMicroseconds(9, L_stopped_speed);
  }
  
    if (Right_speed>700){
    pwm.writeMicroseconds(10, R_stopped_speed-100);
  }
   else if (Right_speed<200){
    pwm.writeMicroseconds(10,R_stopped_speed+100);
   }
  else{
    pwm.writeMicroseconds(10, R_stopped_speed);
  }
}
void Detect_Fire(){
  int Fire_State=digitalRead(Fire_detector);
  if(Fire_State==0){
    t_4_fire  = millis();
  }
  if((millis()-t_4_fire)<1000){
    pwm.writeMicroseconds(11, 1900);
  }
  else{
    pwm.writeMicroseconds(11, 1000);
  }
  // Serial.println((millis()-t_4_fire));
}

void counting_lines(){
 t = millis();
  if (digitalRead(irSensorPin0) == LOW && !has_seen) {
    counting = true;
    timer = millis();
    has_seen = true;
  }
  if (digitalRead(irSensorPin0) == HIGH && has_seen){
    count += 1;
    has_seen = false;
    delay(50);
  }


  if (t - timer > 750){
    counting = false;
  }
  //Serial.println(count);


  // // after leaving the counting state, we write our LED's according to the count, and allow them to shine for half a second. after that, they are all turned off (when the count returns to 0).
  if (count == 1 && !counting) {
    digitalWrite(ledPinRed, HIGH);
    led_blink_time = millis();
    Serial.println("we hit for red");
    count = 0;
  }
  else if (count >= 2 && !counting) {
    digitalWrite(ledPinGreen, HIGH);
    led_blink_time = millis();
    Serial.println("we hit for green");
    count = 0;
  }
  else if (!counting){
    count = 0;
  }

  if (t - led_blink_time > 500 && count == 0){
    digitalWrite(ledPinRed, LOW);
    digitalWrite(ledPinGreen, LOW);
  }

 }
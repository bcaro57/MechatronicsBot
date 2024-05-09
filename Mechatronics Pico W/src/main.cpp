#include <Arduino.h>

int external_led = 15;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(external_led, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  // Basic LED Blink - To test configuration
  // For some reason the on board LED does not blink (thought maybe it was the pico w
  // because the LED deals with a few other things), but I got it to blink using python initially
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(external_led, HIGH);
  Serial.println("1");
  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(external_led, LOW);
  Serial.println("0");
  delay(500);

  // if (Serial.available() > 0) {
  //   char incomingByte = Serial.read();

  //   Serial.print("Received: ");
  //   Serial.println(incomingByte);
  // }

}
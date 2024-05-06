#include <Arduino.h>

int led = PIN_LED;

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(led, HIGH);
  Serial.println("LED ON");
  delay(500);

  digitalWrite(led, LOW);
  Serial.println("LED OFF");
  delay(500);
}


#include <Arduino.h>

/*
  Arduino Wireless Communication Tutorial
      Example 1 - Transmitter Code

  by Dejan Nedelkovski, www.HowToMechatronics.com

  Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  int Lefty=698;
  int Righty = 0;
  int Single = 0;
  int Double = 0;
  int border=0;
};

Data_Package data; // Create a variable with the above structure

void setup() {
  Serial.begin(9600);
  /*
    if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  */
 radio.openWritingPipe(address); // 00002
  radio.setPayloadSize(sizeof(Data_Package)); 
  radio.setPALevel(RF24_PA_MIN); 
  radio.stopListening();
}

void loop() {
  // Send the whole data from the structure to the receiver

   //delay(5);
  radio.stopListening();
  //radio.startListening();
  data.Lefty = analogRead(A0);
  data.Righty = analogRead(A1);
  radio.write(&data, sizeof(Data_Package));
  Serial.println(data.Lefty);
  delay(5);
   //
  /*
  radio.startListening();
  while (!radio.available());
  radio.read(&data, sizeof(Data_Package));
  radio.startListening();
  */
  /*
  Serial.print("Lefty: ");
  Serial.print(data.Lefty);
  Serial.print(" Righty: ");
  Serial.print(data.Righty);
  Serial.print(" Single: ");
  Serial.print(data.Single);
  Serial.print(" Double: ");
  Serial.print(data.Double);
  Serial.print(" border: ");
  Serial.println(data.border); 
*/
}

/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
/*
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  const char text[] = "Hello World";
  radio.write(&text, sizeof(text));
  delay(1000);
}
*/

#include <Arduino.h>

#include <WiFi.h>

const int external_LED = 15;

const char* ssid = "bens laptop";
const char* password = "demetrius";

// ------------------------ Initial WiFi Setup ----------------------- //
/*
void setup() {

  // Start the Serial Monitor
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // For some reason this doesn't work - Not sure why yet
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  //   Serial.print(WiFi.status());
  //   delay(500);
  // }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(external_LED, OUTPUT);
}

void loop() {
  // Connection established
  Serial.println("");
  Serial.print("Pico W is connected to WiFi network ");
  Serial.println(WiFi.SSID());
 
  // Print IP Address
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());

  // Basic LED Blink 
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(external_LED, HIGH);
  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(external_LED, LOW);

  delay(500);
}
*/
 
// --------------------------- LED Control -------------------------------- //
// Set web server port number to 80
WiFiServer server(80);
 
String header;                          // Variable to store the HTTP request
String picoLEDState = "off";            // Variable to store onboard LED state
 
unsigned long currentTime = millis();   // Current time
unsigned long previousTime = 0;         // Previous time

const long timeoutTime = 2000;          // Define timeout time in milliseconds (example: 2000ms = 2s)
 
void setup() {
 
  // Start Serial Monitor
  Serial.begin(9600);
 
  pinMode(LED_BUILTIN, OUTPUT);         // Initialize the LED as an output
  pinMode(external_LED, OUTPUT);        // Initialize the external LED as an output

  digitalWrite(LED_BUILTIN, LOW);       // Set LED off
 
  WiFi.begin(ssid, password);           // Connect to Wi-Fi network with SSID and password
 
  // Display progress on Serial monitor
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  // Print local IP address and start web server
  Serial.println("");
  Serial.print("WiFi connected at IP Address ");
  Serial.println(WiFi.localIP());
 
  // Start Server
  server.begin();
}
 
 // ------------------------ LED Control --------------------------- //
void loop() {
 
  WiFiClient client = server.available();   // Listen for incoming clients
 
  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
 
            // Switch the LED on and off
            if (header.indexOf("GET /led/on") >= 0) {
              Serial.println("LED on");
              picoLEDState = "on";
              digitalWrite(LED_BUILTIN, HIGH);
              digitalWrite(external_LED, HIGH);
            } else if (header.indexOf("GET /led/off") >= 0) {
              Serial.println("LED off");
              picoLEDState = "off";
              digitalWrite(LED_BUILTIN, LOW);
              digitalWrite(external_LED, LOW);
            }
 
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
 
            // CSS to style the on/off buttons
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #F23A3A;}</style></head>");
 
            // Web Page Heading
            client.println("<body><h1>Pico W LED Control</h1>");
 
            // Display current state, and ON/OFF buttons for Onboard LED
            client.println("<p>Onboard LED is " + picoLEDState + "</p>");
            
            // Set buttons
            if (picoLEDState == "off") {
              
              //picoLEDState is off, display the ON button
              client.println("<p><a href=\"/led/on\"><button class=\"button\">ON</button></a></p>");
            } else {
 
              //picoLEDState is on, display the OFF button
              client.println("<p><a href=\"/led/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
 
            client.println("</body></html>");
 
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
#include <stdio.h>

// set pin numbers
const int buttonPin = 18;  // Button pin
const int buttonPin2 = 19; // Button pin
const int buzzerPin = 32; // Buzzer pin

// variable for storing the button status
int buttonState = 0;
int buttonState2 = 0;

void setup() {
  Serial.begin(115200);
  // initialize the button pin as an input
  pinMode(buttonPin, INPUT);
  // initialize the button pin as an input
  pinMode(buttonPin2, INPUT);
}

void loop() {
  // read the state of the button value
  buttonState = digitalRead(buttonPin);
  buttonState2 = digitalRead(buttonPin2);

  // Check for a low-priority message from Python
  if (Serial.available() > 0) {
    char incoming = Serial.read();  // Read the incoming byte

    if (incoming == '1') {  // Check if a low-priority warning exists
      for (int i = 0; i < 200; i++) { // Loop 200 times and play a short tone each time
        digitalWrite(buzzerPin, HIGH); // Set to HIGH to make the buzzer sound
        delay(3); // Wait for 3 milliseconds
        digitalWrite(buzzerPin, LOW); // LOW to turn off the buzzer
        delay(3); // Wait for 3 milliseconds
      }
    }
  }

  // If button 1 is pressed, send "LOW_PRIORITY"
  if (buttonState == HIGH) {
    // Send request for low priority message to program
    Serial.println("LOW_PRIORITY");
    delay(500);
  } 
  // If button 2 is pressed, send "LAST_MESSAGE"
  else if (buttonState2 == HIGH) {
    // Send request for last message to program
    Serial.println("LAST_MESSAGE");
    delay(500);
  }
}

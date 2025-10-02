// Use this example with the Adafruit Keypad products.
// You'll need to know the Product ID for your keypad.
// Here's a summary:
//   * PID3844 4x4 Matrix Keypad
//   * PID3845 3x4 Matrix Keypad
//   * PID1824 3x4 Phone-style Matrix Keypad
//   * PID1332 Membrane 1x4 Keypad
//   * PID419  Membrane 3x4 Matrix Keypad

#include "Adafruit_Keypad.h"

// define your specific keypad here via PID
#define KEYPAD_PID3844
// define your pins here
// can ignore ones that don't apply
#define R1    2
#define R2    3
#define R3    4
#define R4    5
#define C1    8
#define C2    9
#define C3    10
#define C4    11
#define red   12
#define green 7
#define blue 6
#define greenLED 13
#define bitter e.bit

// leave this import after the above configuration
#include "keypad_config.h"

String pass = "73738#"; // Correct input expected.
String progress = ""; // Keeps track of what keys have been inputted.

//initialize an instance of class NewKeypad
Adafruit_Keypad customKeypad = Adafruit_Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Serial.begin(9600);
  customKeypad.begin();
  pinMode(12, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(13, OUTPUT);
  randomSeed(analogRead(0));
}

void colorSetter(uint8_t red_state, uint8_t green_state, uint8_t blue_state) 
{
  digitalWrite(red, red_state);
  digitalWrite(green, green_state);
  digitalWrite(blue, blue_state);
}

void loop() {
  // Main code that runs repeatedly:
  customKeypad.tick();

  while(customKeypad.available()){
    keypadEvent e = customKeypad.read(); // Reads most recent keypad event.

       
    if (progress.length() >= 6) // When progress equals length of correct code...
    {
      if (progress == pass) // Checking if progress equals the correct code.
      {
        Serial.println("SUCCESS!");
        int rand = random(1, 5); // Generates a random number between 1 and 4.
        switch (rand) {
          case 1: // Red
            colorSetter(HIGH, LOW, LOW);
            break;
          case 2: // Green
            colorSetter(LOW, HIGH, LOW);
            break;
          case 3:
            colorSetter(LOW, LOW, HIGH);
            break;
          case 4: //Purple
            colorSetter(HIGH, LOW, HIGH);
            break;
        }
        digitalWrite(greenLED, HIGH);
      }
      else
      {
        Serial.println("FAIL!");
      }
      progress = ""; // Reset progress for next attempt.
      delay(1000); // LED stays on for 1 second before turning back off for standby.
      colorSetter(LOW, LOW, LOW); // off
      digitalWrite(greenLED, LOW);
    }

    if (bitter.EVENT == KEY_JUST_PRESSED) // If most recent event is a button pressed...
    {
      progress += (char)bitter.KEY; // Cast the key pressed to a char and add to progress.
      Serial.println(progress); // Print current progress for debugging.
    }
  }

  delay(10);
}
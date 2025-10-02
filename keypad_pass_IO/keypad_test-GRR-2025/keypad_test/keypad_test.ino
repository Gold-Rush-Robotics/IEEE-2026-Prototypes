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

// leave this import after the above configuration
#include "keypad_config.h"

String pass = "73738#"; // Correct input expected.
String progress = ""; // Keeps track of what keys have been inputted.

//initialize an instance of class NewKeypad
Adafruit_Keypad customKeypad = Adafruit_Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  //progress = ""; 
  Serial.begin(9600);
  customKeypad.begin();
  pinMode(12, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
}

void colorSetter(uint8_t color) // Sets LED color to color.
{
  digitalWrite(green, LOW);
  digitalWrite(red, LOW);
  digitalWrite(blue, LOW);
  digitalWrite(color, HIGH);
}

void loop() {
  // Main code that runs repeatedly:
  customKeypad.tick();

  while(customKeypad.available()){
    keypadEvent e = customKeypad.read(); // Reads most recent keypad event.

    colorSetter(blue);
       
    if (progress.length() >= 6) // When progress equals length of correct code...
    {
      if (progress == pass) // Checking if progress equals the correct code.
      {
        Serial.println("SUCCESS!");
        colorSetter(green);
      }
      else
      {
        Serial.println("FAIL!");
        colorSetter(red);
      }
      progress = ""; // Reset progress for next attempt.
      delay(1000); // LED stays red/green for 1 second before switching back to blue for standby.
      colorSetter(blue);
    }

    if (e.bit.EVENT == KEY_JUST_PRESSED) // If most recent event is a button pressed...
    {
      progress += (char)e.bit.KEY; // Cast the key pressed to a char and add to progress.
      Serial.println(progress); // Print current progress for debugging.
    }

  }

  delay(10);
}
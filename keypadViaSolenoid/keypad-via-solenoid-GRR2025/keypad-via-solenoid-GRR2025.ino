/*
About: Made for the 2026 IEEE Competition keypad task. 
To accomplish this task, solenoids are used to enter the code 73738# (RESET#)
Version: October 7th, 2025
*/

// Defined Pins:

int solenoid7 = 3; // PINS CURRENTLY UNDEFINED!
int solenoid3 = 2;
int solenoid8 = 4;
int solenoidPound = 5;

void setup() {
  // Pins set as output:

  pinMode(solenoid7, OUTPUT);
  pinMode(solenoid3, OUTPUT);
  pinMode(solenoid8, OUTPUT);
  pinMode(solenoidPound, OUTPUT);

  // Solenoid sequence runs:
  // Code: 73738# (RESET#)

  digitalWrite(solenoid7, HIGH); // Solenoid presses 7.
  delay(1000);
  digitalWrite(solenoid7, LOW);
  delay(1000);

  digitalWrite(solenoid3, HIGH); // Solenoid presses 3.
  delay(1000);
  digitalWrite(solenoid3, LOW);
  delay(1000);

  digitalWrite(solenoid7, HIGH); // Solenoid presses 7.
  delay(1000);
  digitalWrite(solenoid7, LOW);
  delay(1000);

  digitalWrite(solenoid3, HIGH); // Solenoid presses 3.
  delay(1000);
  digitalWrite(solenoid3, LOW);
  delay(1000);

  digitalWrite(solenoid8, HIGH); // Solenoid presses 8.
  delay(1000);
  digitalWrite(solenoid8, LOW);
  delay(1000);

  digitalWrite(solenoidPound, HIGH); // Solenoid presses #.
  delay(1000);
  digitalWrite(solenoidPound, LOW);
  delay(1000);
}

void loop() {
  // Code only needs to run once.
}
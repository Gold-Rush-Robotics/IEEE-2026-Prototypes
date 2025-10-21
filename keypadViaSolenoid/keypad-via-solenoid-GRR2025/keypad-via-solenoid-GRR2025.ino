/*
About: Made for the 2026 IEEE Competition keypad task. 
To accomplish this task, solenoids are used to enter the code 73738# (RESET#)
Version: October 7th, 2025
*/

// Defined Pins:

int solenoid3 = 2;
int solenoid7 = 3;
int solenoid8 = 4;
int solenoidPound = 5;
int delayAfterHigh= 100;
int delayAfterLow = 2000;

void setup() {
  // Pins set as output:

  pinMode(solenoid7, OUTPUT);
  pinMode(solenoid3, OUTPUT);
  pinMode(solenoid8, OUTPUT);
  pinMode(solenoidPound, OUTPUT);

  // Solenoid sequence runs:
  // Code: 73738# (RESET#)

  
}

void loop() {
  // Code only needs to run once.
  digitalWrite(solenoid7, HIGH); // Solenoid presses 7.
  delay(delayAfterHigh);
  digitalWrite(solenoid7, LOW);
  delay(delayAfterLow);

  digitalWrite(solenoid3, HIGH); // Solenoid presses 3.
  delay(delayAfterHigh);
  digitalWrite(solenoid3, LOW);
  delay(delayAfterLow);

  digitalWrite(solenoid7, HIGH); // Solenoid presses 7.
  delay(delayAfterHigh);
  digitalWrite(solenoid7, LOW);
  delay(delayAfterLow);

  digitalWrite(solenoid3, HIGH); // Solenoid presses 3.
  delay(delayAfterHigh);
  digitalWrite(solenoid3, LOW);
  delay(delayAfterLow);

  digitalWrite(solenoid8, HIGH); // Solenoid presses 8.
  delay(delayAfterHigh);
  digitalWrite(solenoid8, LOW);
  delay(delayAfterLow);

  digitalWrite(solenoidPound, HIGH); // Solenoid presses #.
  delay(delayAfterHigh);
  digitalWrite(solenoidPound, LOW);
  delay(delayAfterLow);

  delay(5000);
}
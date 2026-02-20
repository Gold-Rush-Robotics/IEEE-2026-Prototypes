/*
About: Made for the 2026 IEEE Competition keypad task. 
To accomplish this task, solenoids are used to enter the code 73738# (RESET#)
Version: October 7th, 2025
*/

// Defined Pins:

int solenoid3 = 3;
int solenoid7 = 2;
int solenoid8 = 4;
int solenoidPound = 5;
int delayAfterHigh= 100;
int delayAfterLow = 2000;
int soleArr[6] = {solenoid7, solenoid3, solenoid7, solenoid3, solenoid8, solenoidPound};

void setup() {
  // Pins set as output:

  pinMode(solenoid7, OUTPUT);
  pinMode(solenoid3, OUTPUT);
  pinMode(solenoid8, OUTPUT);
  pinMode(solenoidPound, OUTPUT);

  // Solenoid sequence runs:
  // Code: 73738# (RESET#)

  
}

void funcInputter(int num)
{
  digitalWrite(num, HIGH);
  delay(delayAfterHigh);
  digitalWrite(num, LOW);
  delay(delayAfterLow);
}

void loop() {
  // Prototype Optimization 2
  for (int i = 0; i<6; i++)
  {
    funcInputter(soleArr[i]);
  }

  delay(5000);
}
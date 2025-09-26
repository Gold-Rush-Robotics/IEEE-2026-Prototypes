/*
Author: Benjamin LaCroix
Version: September 25th, 2025
*/

int solenoid7 = ;
int solenoid3 = ;
int solenoid8 = ;
int solenoidPound = ;
boolean runOnce = true;

void setup() {
  pinMode(solenoid7, OUTPUT); //Sets the pin as an output
}

void loop() {
  if (runOnce) {
    // Code to run once:
    digitalWrite(solenoid7, HIGH);
    delay(1000);
    digitalWrite(solenoid7, LOW);
    delay(1000);

    digitalWrite(solenoid3, HIGH);
    delay(1000);
    digitalWrite(solenoid3, LOW);
    delay(1000);

    digitalWrite(solenoid7, HIGH);
    delay(1000);
    digitalWrite(solenoid7, LOW);
    delay(1000);

    digitalWrite(solenoid3, HIGH);
    delay(1000);
    digitalWrite(solenoid3, LOW);
    delay(1000);

    digitalWrite(solenoid8, HIGH);
    delay(1000);
    digitalWrite(solenoid8, LOW);
    delay(1000);

    digitalWrite(solenoidPound, HIGH);
    delay(1000);
    digitalWrite(solenoidPound, LOW);
    delay(1000);

    // Stop code after single loop
    runOnce = false;
  }
  


}

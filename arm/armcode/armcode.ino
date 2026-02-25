#include <SCServo.h>

SMS_STS st;

// Changes position of servo with ID
void writePosition(int id, int position, int speed = 2500, int acceleration = 50) {
  st.WritePosEx(id, position, speed, acceleration);
}

// Reads position of servo with ID
int getPosition(int id) {
  int pos = st.ReadPos(id);
  return pos;
}

// Prints position of servo with ID
void printPosition(int id) {
  int pos = getPosition(id);
  Serial.print("Current Position: ");
  Serial.println(pos);
}

// Serial Stuff. IDK
void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1);
  st.pSerial = &Serial1;

  delay(1000);
}

void loop()
{
  writePosition(1, 4095);
  delay(3000);
  printPosition(1);
  delay(1000);

  writePosition(2, 4095);
  delay(3000);
  printPosition(2);
  delay(1000);

  writePosition(1, 0);
  delay(3000);
  printPosition(1);
  delay(1000);

  writePosition(2, 0);
  delay(3000);
  printPosition(2);
  delay(1000);

}

// V=60*0.732=43.92rpm A=50*8.7deg/s^2，P0=0, P1=4095
// delay((4095-0)*1000/(60*50) + (60*50)*10/(50) + 50); //[(P1-P0)/(V*50)]*1000+[(V*50)/(A*100)]*1000 + 50
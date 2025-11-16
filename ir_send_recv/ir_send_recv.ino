#define IR_SEND_PIN A0
#define IR_RECV_PIN A1
#define MESSAGE_SEND_DELAY_MS 500

#include <IRremote.h>

IRsend irsend;
IRrecv irrecv(IR_RECV_PIN);

uint32_t lastSendTime = millis();

void setup() {
  // false disables LED feedback.
  irsend.begin(false);
  irrecv.enableIRIn();
  Serial.begin(9600);
}

void loop() {
  // Check/handle any received commands
  if (irrecv.decode()) {
    // This is the actual data received from the IR transmitter
    uint16_t command = irrecv.decodedIRData.command;
    Serial.print(command, HEX);
    irrecv.resume();  // unblock for next input
  }

  // Send IR command if it has been longer than delay
  uint32_t now = millis();
  if (now - lastSendTime >= MESSAGE_SEND_DELAY_MS) {
    lastSendTime = now;  // Reset the timer
    irsend.sendNEC(0xB847, 0xAB, 0);
  }
}
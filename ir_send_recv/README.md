# IR_Send_Recv

This program is an abstraction for sending/receiving IR data on a Teensy.

## Configuration (`#define`s)

| Variable | Default | Description |
| :--- | :--- | :--- |
| `IR_SEND_PIN` | `A0` | Digital pin used for **transmitting** IR signals (i.e. the pin that the IR transmitter data pin is connected to). |
| `IR_RECV_PIN` | `A1` | Digital pin used for **receiving** IR signals (i.e. the pin that the IR receiver data pin is connected to). |
| `MESSAGE_SEND_DELAY_MS` | `500` | Delay (in milliseconds) between sending IR commands. |

## Running

Open the code in Arduino-IDE. You will need to have Teensyduino set up to be able to run code on a Teensy ([installation instructions](https://www.pjrc.com/teensy/td_download.html)). If correctly set up, you should be able to select the Teensy board and flash it to the Teensy to be able to run the code.

> [!NOTE]
> If the teensy is powered over USB, you can use the Teensy's VIN pin (pin 0) as VCC (5V) for the IR transmitter/receiver.

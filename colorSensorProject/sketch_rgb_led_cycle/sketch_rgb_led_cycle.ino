// Pin assignments
int redPin   = 2;
int greenPin = 3;
int bluePin  = 4;

int cycleLedPin = 5;

// Tracks which color we are on
int colorIndex = 0;

// Debounce
unsigned long lastPress = 0;
const unsigned long debounceTime = 200; // ms


// Sets the actual LED color (0–255 each)
void setColor(int r, int g, int b) {
  analogWrite(redPin, r);
  analogWrite(greenPin, g);
  analogWrite(bluePin, b);
}


// What each button press should show
void cycleColor(int index) {
  switch (index) {

    case 0: // Off
      setColor(0, 0, 0);
      break;
      
    case 1: // Red
      setColor(255, 0, 0);
      break;

    case 2: // Green
      setColor(0, 255, 0);
      break;

    case 3: // Blue
      setColor(0, 0, 255);
      break;

    case 4: // Purple (R + B)
      setColor(255, 0, 255);
      break;
  }
}


void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(cycleLedPin, INPUT_PULLUP);

  // Start with all off
  setColor(0, 0, 0);
}


void loop() {
  // Check for button press (active LOW)
  if (digitalRead(cycleLedPin) == LOW) {
    if (millis() - lastPress > debounceTime) {
      lastPress = millis();

      // Move to next color
      colorIndex = (colorIndex + 1) % 5;

      cycleColor(colorIndex);
    }
  }
}
#include <Wire.h>
#include <math.h> // to use the sqrt function
#include "Adafruit_TCS34725.h"


struct Color {
  float r;
  float g;
  float b;
};

// LED color vectors
const Color red = {234.0, 13.0, 24.0};
const Color green = {23.0, 150.0, 77};
const Color blue = {10.0, 50.0, 200.0}; 
const Color purple = {110.0, 35.0, 119.0};

// LED control pin, change variables if needed for wiring
const int LED_PIN = 33;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);


void setup() {
  // Turn off onboard LED - Code only dims LED, its powered through a transistor i believe, Sylvester.. elp.. 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

// pin of doom and gloom
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
    // also supposed to turn off (dim) onboard LED
    tcs.setInterrupt(true);
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}


void debug(float r, float g, float b, float redDistance, float greenDistance, float blueDistance, float purpleDistance) {
  // Serial Prints
  Serial.print("\nR: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.println(" ");

  Serial.print("Distance from red: ");
  Serial.println(redDistance);

  Serial.print("Distance from green: ");
  Serial.println(greenDistance);

  Serial.print("Distance from blue: ");
  Serial.println(blueDistance);

  Serial.print("Distance from purple: ");
  Serial.println(purpleDistance);
}


float calculateEvilDistance(Color current, Color match) {
  // k-nearest neighbors >:)
  float diffR, diffG, diffB;
  diffR = current.r - match.r;
  diffG = current.g - match.g;
  diffB = current.b - match.b;
  
  long sumOfSquares = (long)diffR * diffR + (long)diffG * diffG + (long)diffB * diffB;

  
  return sqrt(sumOfSquares);
}


void loop() {
  // read from sens find dist (can add delay)
  float r, g, b;

  tcs.getRGB(&r, &g, &b);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);

  Color currentColor = {r, g, b};

  float redDistance = calculateEvilDistance(currentColor, red);
  float greenDistance = calculateEvilDistance(currentColor, green);
  float blueDistance = calculateEvilDistance(currentColor, blue);
  float purpleDistance = calculateEvilDistance(currentColor, purple);

  // uncomment for serial prints to check raw values
  debug(r, g, b, redDistance, greenDistance, blueDistance, purpleDistance);

  delay(1000);
}

/* if (distance >= 20 && distance <= 80) {
    Serial.print("\nColor red");
  } else {
    Serial.print("\nColor not found.");
  } */

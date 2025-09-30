#include <Wire.h>
#include <math.h> // to use the sqrt function
#include "Adafruit_TCS34725.h"

struct Color {
  float r;
  float g;
  float b;
};

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

void setup() {
// pin of doom and gloom
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}

float calculateEvilDistance(Color current, Color match) {
  float diffR, diffG, diffB;
  diffR = current.r - match.r;
  diffG = current.g - match.g;
  diffB = current.b - match.b;
  
  long sumOfSquares = (long)diffR * diffR + (long)diffG * diffG + (long)diffB * diffB;

  
  return sqrt(sumOfSquares);

}


void loop() {
  // read from sens find dist (can add delay)
  Color blue= {10.0, 50.0, 200.0}; 
  Color purple= {110.0, 35.0, 119.0};
  Color green= {23.0, 150.0, 77};
  Color red= {234.0, 13.0, 24.0};


  float r, g, b;


  tcs.getRGB(&r, &g, &b);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);

  Color currentColor = {r, g, b};

  float blueDistance = calculateEvilDistance(currentColor, blue);
  float purpleDistance = calculateEvilDistance(currentColor, purple);
  float greenDistance = calculateEvilDistance(currentColor, green);
  float redDistance = calculateEvilDistance(currentColor, red);


  Serial.print("\nR: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.println(" ");

  Serial.print("Distance from blue: ");
  Serial.println(blueDistance);

  Serial.print("Distance from purple: ");
  Serial.println(purpleDistance);

  Serial.print("Distance from green: ");
  Serial.println(greenDistance);

  Serial.print("Distance from red: ");
  Serial.println(redDistance);

  delay(1000);
}

/* if (distance >= 20 && distance <= 80) {
    Serial.print("\nColor red");
  } else {
    Serial.print("\nColor not found.");
  } */

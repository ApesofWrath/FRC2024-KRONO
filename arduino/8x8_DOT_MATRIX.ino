/* Basic example code for MAX7219 LED dot matrix display with Arduino.
More info: https://www.makerguides.com */

// Include the required Arduino libraries:
#include "MD_Parola.h"
#include "MD_MAX72xx.h"
#include "SPI.h"

// Define hardware type, size, and output pins:
#define HARDWARE_TYPE MD_MAX72XX::GENERIC_HW
#define MAX_DEVICES 1
#define CS_PIN 4

// Create a new instance of the MD_Parola class with hardware SPI connection:
MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

String message = "0A";
int n = 0;

void setup() {
  // Intialize the object:
  myDisplay.begin();
  // Set the intensity (brightness) of the display (0-15):
  myDisplay.setIntensity(1);
  // Clear the display:
  myDisplay.displayClear();
}

void loop() {
  if (myDisplay.displayAnimate()) {
    myDisplay.print(message.charAt(n));
    n = n==message.length()?0:n+1;
    delay(1000);
  }
}
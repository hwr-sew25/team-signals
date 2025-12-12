#include <Adafruit_NeoPixel.h>
#include "patterns.h"
#include "state_defs.h"

#define LED_PIN 6
#define NUM_LEDS 15

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

SignalState currentState = IDLE;

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "GREETING") greeting();
    else if (cmd == "IDLE") idle();
    else if (cmd == "BUSY") busy();
    else if (cmd == "ERROR_MINOR") errorMinor();
    else if (cmd == "ERROR") errorCritical();
    else if (cmd == "LOWBATTERY") lowBattery();
    else if (cmd == "START_MOVE") startMove();
    else if (cmd == "STOP_MOVE") stopMove();
    else if (cmd == "REVERSE") reverse();
    else if (cmd == "SPEAKING") speaking();
  }
}

// --- LED FUNCTIONS --------------------------------------------------

void fillColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

// --- LED PATTERNS -----------------------------------------------------

void greeting() {
  fillColor(0, 0, 255); 
}

void idle() {
  fillColor(20, 20, 20);
}

void busy() {
  fillColor(0, 0, 180);
}

void errorMinor() {
  fillColor(255, 180, 0);
}

void errorMajor() {
  fillColor(255, 0, 0);
}

void lowBattery() {
  fillColor(200, 100, 0);
}

void startMove() {
  fillColor(255, 255, 255);
}

void stopMove() {
  fillColor(255, 0, 0);
}

void reverse() {
  fillColor(255, 255, 255);
}

void speaking() {
  fillColor(180, 0, 255);
}



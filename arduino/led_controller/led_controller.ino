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

// STATE MACHINE LOOP
void loop() {

  // 1. Listen for state input (later from Raspberry Pi)
  if (Serial.available()) {
    int s = Serial.parseInt();
    if (s >= 0 && s <= 10) {
      currentState = (SignalState)s;
    }
  }

  // 2. Execute LED pattern based on state
  switch (currentState) {
    case GREETING:
      patternGreeting(strip);
      break;
    case IDLE:
      patternIdle(strip);
      break;
    case BUSY:
      patternBusy(strip);
      break;
    case ERROR_MINOR:
      patternErrorMinor(strip);
      break;
    case ERROR_MAJOR:
      patternErrorMajor(strip);
      break;
    case LOW_BATTERY:
      patternLowBattery(strip);
      break;
    case MOVE:
      patternMove(strip);
      break;
    case START_MOVE:
      patternStartMove(strip);
      break;
    case STOP_MOVE:
      patternStopMove(strip);
      break;
    case REVERSE:
      patternReverse(strip);
      break;
    case SPEAKING:
      patternSpeaking(strip);
      break;
  }
}



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

  // --- SERIAL INPUT ---
  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.equals("GREETING")) {
      currentState = GREETING;
    }
    else if (cmd.equals("IDLE")) {
      currentState = IDLE;
    }
    else if (cmd.equals("BUSY")) {
      currentState = BUSY;
    }
    else if (cmd.equals("ERROR_MINOR")) {
      currentState = ERROR_MINOR;
    }
    else if (cmd.equals("ERROR_MAJOR")) {
      currentState = ERROR_MAJOR;
    }
    else if (cmd.equals("LOW_BATTERY")) {
      currentState = LOW_BATTERY;
    }
    else if (cmd.equals("MOVE")) {
      currentState = MOVE;
    }
    else if (cmd.equals("START_MOVE")) {
      currentState = START_MOVE;
    }
    else if (cmd.equals("STOP_MOVE")) {
      currentState = STOP_MOVE;
    }
    else if (cmd.equals("REVERSE")) {
      currentState = REVERSE;
    }
    else if (cmd.equals("SPEAKING")) {
      currentState = SPEAKING;
    }
  }

  // ---- LED STATE MACHINE ----
  switch(currentState) {
    case GREETING:      patternGreeting(strip); break;
    case IDLE:          patternIdle(strip); break;
    case BUSY:          patternBusy(strip); break;
    case ERROR_MINOR:   patternErrorMinor(strip); break;
    case ERROR_MAJOR:   patternErrorMajor(strip); break;
    case LOW_BATTERY:   patternLowBattery(strip); break;
    case MOVE:          patternMove(strip); break;
    case START_MOVE:    patternStartMove(strip); break;
    case STOP_MOVE:     patternStopMove(strip); break;
    case REVERSE:       patternReverse(strip); break;
    case SPEAKING:      patternSpeaking(strip); break;
  }
}


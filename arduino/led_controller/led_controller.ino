#include <Adafruit_NeoPixel.h>
#include "patterns.h"
#include "state_defs.h"

#define LED_PIN 6
#define NUM_LEDS 60

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

SignalState currentState = IDLE;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  strip.begin();
  strip.show();
  Serial.println("READY");
}

void loop() {
  static String cmd = "";

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      cmd.trim();
      cmd.toUpperCase();

      Serial.print("RECEIVED: ");
      Serial.println(cmd);

      // hier deine if-else state switches
      if (cmd == "GREETING") currentState = GREETING;
      else if (cmd == "IDLE") currentState = IDLE;
      else if (cmd == "BUSY") currentState = BUSY;
      else if (cmd == "ERROR_MINOR") currentState = ERROR_MINOR;
      else if (cmd == "ERROR_MAJOR") currentState = ERROR_MAJOR;
      else if (cmd == "LOW_BATTERY") currentState = LOW_BATTERY;
      else if (cmd == "MOVE") currentState = MOVE;
      else if (cmd == "START_MOVE") currentState = START_MOVE;
      else if (cmd == "STOP_MOVE") currentState = STOP_MOVE;
      else if (cmd == "REVERSE") currentState = REVERSE;
      else if (cmd == "SPEAKING") currentState = SPEAKING;
      else Serial.println("-> UNKNOWN COMMAND");

      cmd = ""; // reset buffer
  } else {
    cmd += c;
  }
  }

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



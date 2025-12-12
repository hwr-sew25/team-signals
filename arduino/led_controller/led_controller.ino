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
}

void loop() {

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    Serial.print("RECEIVED: ");
    Serial.println(cmd);

    if (cmd.equals("GREETING")) {
      Serial.println("-> SET STATE GREETING");
      currentState = GREETING;
    }
    else if (cmd.equals("IDLE")) {
      Serial.println("-> SET STATE IDLE");
      currentState = IDLE;
    }
    else if (cmd.equals("BUSY")) {
      Serial.println("-> SET STATE BUSY");
      currentState = BUSY;
    }
    else if (cmd.equals("ERROR_MINOR")) {
      Serial.println("-> SET STATE ERROR_MINOR");
      currentState = ERROR_MINOR;
    }
    else if (cmd.equals("ERROR_MAJOR")) {
      Serial.println("-> SET STATE ERROR_MAJOR");
      currentState = ERROR_MAJOR;
    }
    else if (cmd.equals("LOW_BATTERY")) {
      Serial.println("-> SET STATE LOW_BATTERY");
      currentState = LOW_BATTERY;
    }
    else if (cmd.equals("MOVE")) {
      Serial.println("-> SET STATE MOVE");
      currentState = MOVE;
    }
    else if (cmd.equals("START_MOVE")) {
      Serial.println("-> SET STATE START_MOVE");
      currentState = START_MOVE;
    }
    else if (cmd.equals("STOP_MOVE")) {
      Serial.println("-> SET STATE STOP_MOVE");
      currentState = STOP_MOVE;
    }
    else if (cmd.equals("REVERSE")) {
      Serial.println("-> SET STATE REVERSE");
      currentState = REVERSE;
    }
    else if (cmd.equals("SPEAKING")) {
      Serial.println("-> SET STATE SPEAKING");
      currentState = SPEAKING;
    }
    else {
      Serial.println("-> UNKNOWN COMMAND");
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



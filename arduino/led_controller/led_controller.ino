#include <Adafruit_NeoPixel.h>
#include "patterns.h"
#include "state_defs.h"

#define LED_PIN 6
#define NUM_LEDS 64

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Default-State
SignalState currentState = IDLE;

// Serial command buffer
static String cmd = "";

// Optional: Debug an/aus
#define DEBUG_SERIAL 1

static void setStateFromCommand(const String &command) {
  // command ist bereits TRIM + UPPERCASE
  if      (command == "GREETING")          currentState = GREETING;
  else if (command == "IDLE")              currentState = IDLE;
  else if (command == "BUSY")              currentState = BUSY;
  else if (command == "STOP_BUSY")         currentState = STOP_BUSY;
  else if (command == "ERROR_MINOR_STUCK") currentState = ERROR_MINOR_STUCK;
  else if (command == "ERROR_MINOR_NAV")   currentState = ERROR_MINOR_NAV;
  else if (command == "ROOM_NOT_FOUND")    currentState = ROOM_NOT_FOUND;
  else if (command == "ERROR_MAJOR")       currentState = ERROR_MAJOR;
  else if (command == "LOW_BATTERY")       currentState = LOW_BATTERY;
  else if (command == "MOVE_LEFT")         currentState = MOVE_LEFT;
  else if (command == "MOVE_FORWARD")      currentState = MOVE_FORWARD;
  else if (command == "MOVE_RIGHT")        currentState = MOVE_RIGHT;
  else if (command == "MOVE_BACKWARD")     currentState = MOVE_BACKWARD;
  else if (command == "START_MOVE")        currentState = START_MOVE;
  else if (command == "STOP_MOVE")         currentState = STOP_MOVE;
  else if (command == "GOAL_REACHED")      currentState = GOAL_REACHED;
  else if (command == "REVERSE")           currentState = REVERSE;
  else if (command == "SPEAKING")          currentState = SPEAKING;
  else if (command == "WAITING")           currentState = WAITING;
  else {
#if DEBUG_SERIAL
    Serial.println("-> UNKNOWN COMMAND");
#endif
  }

#if DEBUG_SERIAL
  Serial.print("-> STATE NOW: ");
  Serial.println(command);
#endif
}



void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  strip.begin();
  strip.setBrightness(127); 
  strip.show(); // LEDs aus

#if DEBUG_SERIAL
  Serial.println("READY");
#endif
}


void loop() {
  // ---------- 1) SERIAL INPUT ----------
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // Line endings ignorieren
    if (c == '\r' || c == '\n') continue;

    // Terminator ';' -> Command fertig
    if (c == ';') {
      cmd.trim();
      cmd.toUpperCase();

#if DEBUG_SERIAL
      Serial.print("RECEIVED: '");
      Serial.print(cmd);
      Serial.print("' LEN=");
      Serial.println(cmd.length());
#endif

      if (cmd.length() > 0) {
        setStateFromCommand(cmd);
      } else {
#if DEBUG_SERIAL
        Serial.println("-> EMPTY COMMAND");
#endif
      }

      cmd = "";
      break; // sofort raus, damit wir unten rendern
    }

    // Control bytes raus
    if ((uint8_t)c < 0x20) continue;

    // Buffer limit
    if (cmd.length() < 64) cmd += c;
    else cmd = "";
  }

  // ---------- 2) OUTPUT: nur sinnvoll rendern ----------
  static SignalState lastState = IDLE;

  // Wenn State sich geändert hat: statische Patterns nur 1x setzen
  if (currentState != lastState) {
    lastState = currentState;

    switch (currentState) {
      // Statische Zustände: 1x setzen (kein Dauer-show())
      case IDLE:              patternIdle(strip); break;
      case BUSY:              patternBusy(strip); break;
      case STOP_BUSY:         patternStopBusy(strip); break;
      case ERROR_MINOR_NAV:   patternErrorMinorNav(strip); break;
      case ERROR_MAJOR:       patternErrorMajor(strip); break;
      case MOVE_LEFT:         patternMoveLeft(strip); break;
      case MOVE_FORWARD:      patternMoveForward(strip); break;
      case MOVE_RIGHT:        patternMoveRight(strip); break;
      case MOVE_BACKWARD:     patternMoveBackward(strip); break;
      case STOP_MOVE:         patternStopMove(strip); break;
      case SPEAKING:          patternSpeaking(strip); break;

      // Animierte: initial auch einmal ok
      case GREETING:          patternGreeting(strip); break;
      case LOW_BATTERY:       patternLowBattery(strip); break;
      case START_MOVE:        patternStartMove(strip); break;
      case ROOM_NOT_FOUND:    patternRoomNotFound(strip); break;
      case GOAL_REACHED:      patternGoalReached(strip); break;
      case ERROR_MINOR_STUCK: patternErrorMinorStuck(strip); break;
      case REVERSE:           patternMoveBackward(strip); break;  // REVERSE = MOVE_BACKWARD
      case WAITING:           patternWaiting(strip); break;
    }
  }

  // Animierte Zustände: weiterlaufen lassen (haben millis()-Gating)
  switch (currentState) {
    case GREETING:          patternGreeting(strip); break;
    case LOW_BATTERY:       patternLowBattery(strip); break;
    case START_MOVE:        patternStartMove(strip); break;
    case ROOM_NOT_FOUND:    patternRoomNotFound(strip); break;
    case GOAL_REACHED:      patternGoalReached(strip); break;
    case ERROR_MINOR_STUCK: patternErrorMinorStuck(strip); break;
    case WAITING:           patternWaiting(strip); break;
    default: break; // statische States nicht dauernd neu "show()"en
  }
}




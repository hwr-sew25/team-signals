#include <Adafruit_NeoPixel.h>
#include "patterns.h"
#include "state_defs.h"

// ============================================================
// Hardware-Konfiguration
// ============================================================

#define LED_PIN 6
#define NUM_LEDS 64
#define GLOBAL_BRIGHTNESS 127  // 50% Helligkeit

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================
// State-Variablen
// ============================================================

// Default-State
SignalState currentState = IDLE;

// Flag das anzeigt, dass der State gerade gewechselt hat
// Patterns können dies nutzen um ihre internen Variablen zurückzusetzen
bool stateJustChanged = false;

// Serial command buffer
static String cmd = "";

// ============================================================
// Debug-Konfiguration
// ============================================================

#define DEBUG_SERIAL 1

// ============================================================
// Command Processing
// ============================================================

static void setStateFromCommand(const String &command) {
  // command ist bereits TRIM + UPPERCASE
  bool validCommand = true;
  
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
  else if (command == "SPEAKING")          currentState = SPEAKING;
  else if (command == "WAITING")           currentState = WAITING;
  else {
    validCommand = false;
#if DEBUG_SERIAL
    Serial.println("ERR:UNKNOWN");
#endif
  }

  if (validCommand) {
    // WICHTIG: "OK" Bestätigung senden
    Serial.print("OK:");
    Serial.println(command);
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  strip.begin();
  strip.setBrightness(GLOBAL_BRIGHTNESS); 
  strip.show(); // LEDs aus

#if DEBUG_SERIAL
  Serial.println("READY");
#endif
}

// ============================================================
// Main Loop
// ============================================================

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
      Serial.print("RCV:");
      Serial.println(cmd);
#endif

      if (cmd.length() > 0) {
        setStateFromCommand(cmd);
      } else {
#if DEBUG_SERIAL
        Serial.println("ERR:EMPTY");
#endif
      }

      cmd = "";
      break; // sofort raus, damit wir unten rendern
    }

    // Control bytes raus
    if ((uint8_t)c < 0x20) continue;

    // Buffer limit (verhindert Buffer Overflow)
    if (cmd.length() < 64) {
      cmd += c;
    } else {
      // Buffer voll - verwerfen und Fehler melden
      cmd = "";
#if DEBUG_SERIAL
      Serial.println("ERR:OVERFLOW");
#endif
    }
  }

  // ---------- 2) OUTPUT: nur sinnvoll rendern ----------
  static SignalState lastState = IDLE;

  // Wenn State sich geändert hat: statische Patterns nur 1x setzen
  if (currentState != lastState) {
    lastState = currentState;
    stateJustChanged = true;  // Flag setzen für Pattern-Reset

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

      // Animierte: initial auch einmal aufrufen
      case GREETING:          patternGreeting(strip); break;
      case LOW_BATTERY:       patternLowBattery(strip); break;
      case START_MOVE:        patternStartMove(strip); break;
      case ROOM_NOT_FOUND:    patternRoomNotFound(strip); break;
      case GOAL_REACHED:      patternGoalReached(strip); break;
      case ERROR_MINOR_STUCK: patternErrorMinorStuck(strip); break;
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
  
  // Reset-Flag nach erstem Pattern-Aufruf zurücksetzen
  stateJustChanged = false;
}

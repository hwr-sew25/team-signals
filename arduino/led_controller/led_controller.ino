#include <Adafruit_NeoPixel.h>
#include "patterns.h"
#include "state_defs.h"

// ============================================================
// Hardware-Konfiguration
// ============================================================

#define LED_PIN 6
#define NUM_LEDS 64

// LED-Segment Definitionen (4 Segmente à 16 LEDs)
#define SEGMENT_SIZE 16
#define SEG_LEFT     0   // LEDs 0-15
#define SEG_FORWARD  1   // LEDs 16-31
#define SEG_RIGHT    2   // LEDs 32-47
#define SEG_BACKWARD 3   // LEDs 48-63

// Aktuelle Bewegungsrichtung (für MOVE State)
static uint8_t currentMoveDirection = SEG_FORWARD;
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
#define DEBUG_PRIORITY 1  // Extra Debug-Ausgaben für Prioritäten

// ============================================================
// Prioritäts-basiertes State Management
// ============================================================

// Versucht einen neuen State zu setzen, respektiert Prioritäten
// Returns: true wenn State gewechselt wurde, false wenn blockiert
static bool trySetState(SignalState newState, bool forceOverride) {
    // Gleicher State? Kein Wechsel nötig
    if (newState == currentState) {
        return false;
    }
    
    // Force-Override: Immer erlauben (für FORCE_ Prefix oder spezielle States)
    if (forceOverride) {
        currentState = newState;
#if DEBUG_PRIORITY
        Serial.print("FORCE:");
        Serial.println(newState);
#endif
        return true;
    }
    
    // Spezial-States die IMMER überschreiben dürfen
    if (isForceAllowedState(newState)) {
        currentState = newState;
#if DEBUG_PRIORITY
        Serial.print("SAFETY:");
        Serial.println(newState);
#endif
        return true;
    }
    
    // Normale Prioritätsprüfung
    if (canOverrideState(currentState, newState)) {
        currentState = newState;
#if DEBUG_PRIORITY
        Serial.print("PRIO_OK:");
        Serial.println(newState);
#endif
        return true;
    }
    
    // State-Wechsel blockiert
#if DEBUG_PRIORITY
    Serial.print("PRIO_BLOCKED:");
    Serial.print(newState);
    Serial.print("<");
    Serial.println(currentState);
#endif
    return false;
}

// ============================================================
// Command Processing
// ============================================================

// Parst einen String-Command zu einem SignalState
// Returns: STATE_INVALID wenn nicht erkannt
static SignalState parseStateCommand(const String &command) {
    if      (command == "GREETING")          return GREETING;
    else if (command == "IDLE")              return IDLE;
    else if (command == "BUSY")              return BUSY;
    else if (command == "STOP_BUSY")         return STOP_BUSY;
    else if (command == "ERROR_MINOR_STUCK") return ERROR_MINOR_STUCK;
    else if (command == "ERROR_MINOR_NAV")   return ERROR_MINOR_NAV;
    else if (command == "ROOM_NOT_FOUND")    return ROOM_NOT_FOUND;
    else if (command == "ERROR_MAJOR")       return ERROR_MAJOR;
    else if (command == "LOW_BATTERY")       return LOW_BATTERY;
    else if (command == "MOVE_LEFT")         return MOVE_LEFT;
    else if (command == "MOVE_FORWARD")      return MOVE_FORWARD;
    else if (command == "MOVE_RIGHT")        return MOVE_RIGHT;
    else if (command == "MOVE_BACKWARD")     return MOVE_BACKWARD;
    else if (command == "START_MOVE")        return START_MOVE;
    else if (command == "STOP_MOVE")         return STOP_MOVE;
    else if (command == "GOAL_REACHED")      return GOAL_REACHED;
    else if (command == "SPEAKING")          return SPEAKING;
    else if (command == "WAITING")           return WAITING;
    else                                     return STATE_INVALID;
}

static void setStateFromCommand(const String &command) {
    // command ist bereits TRIM + UPPERCASE
    bool forceOverride = false;
    String actualCommand = command;
    
    // ========================================
    // Spezielle Kommandos
    // ========================================
    
    // RESET: Erzwingt IDLE, ignoriert alle Prioritäten
    if (command == "RESET") {
        currentState = IDLE;
        Serial.println("OK:RESET");
        return;
    }
    
    // STATUS: Gibt aktuellen State zurück
    if (command == "STATUS") {
        Serial.print("STATE:");
        Serial.print(currentState);
        Serial.print(":PRIO:");
        Serial.println(getStatePriority(currentState));
        return;
    }
    
    // FORCE_ Prefix: Überschreibt Prioritäten
    // Beispiel: FORCE_IDLE, FORCE_GREETING
    if (command.startsWith("FORCE_")) {
        forceOverride = true;
        actualCommand = command.substring(6);  // Entferne "FORCE_"
#if DEBUG_SERIAL
        Serial.print("FORCE_CMD:");
        Serial.println(actualCommand);
#endif
    }
    
    // ========================================
    // State-Command parsen und setzen
    // ========================================
    
    SignalState newState = parseStateCommand(actualCommand);
    
    if (newState == STATE_INVALID) {
#if DEBUG_SERIAL
        Serial.print("ERR:UNKNOWN:");
        Serial.println(actualCommand);
#endif
        return;
    }
    
    // Versuche State zu setzen (mit Prioritätsprüfung)
    bool stateChanged = trySetState(newState, forceOverride);
    
    if (stateChanged) {
        // Erfolg - Bestätigung senden
        Serial.print("OK:");
        Serial.println(actualCommand);
    } else {
        // State-Wechsel wurde blockiert
        Serial.print("BLOCKED:");
        Serial.print(actualCommand);
        Serial.print(":BY:");
        Serial.println(currentState);
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
    Serial.println("PRIORITY_ENABLED");
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
            
            // Fallback für unbekannte States
            default:                patternIdle(strip); break;
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

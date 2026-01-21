#ifndef PATTERNS_H
#define PATTERNS_H

#include <Adafruit_NeoPixel.h>

// ============================================================
// LED-Konfiguration
// ============================================================

// Segment-Konstanten für 64 LEDs (4 Segmente à 16 LEDs)
#define PATTERN_SEGMENT_SIZE 16
#define PATTERN_SEG_BACKWARD 0   // LEDs 0-15  (Rückwärts)
#define PATTERN_SEG_RIGHT    1   // LEDs 16-31 (Rechts)
#define PATTERN_SEG_FORWARD  2   // LEDs 32-47 (Vorwärts)
#define PATTERN_SEG_LEFT     3   // LEDs 48-63 (Links)

// ============================================================
// Helligkeits-Konstanten (0-255)
// ============================================================

#define BRIGHTNESS_FULL      255   // Volle Helligkeit
#define BRIGHTNESS_HIGH      200   // Hohe Helligkeit
#define BRIGHTNESS_MEDIUM    127   // Mittlere Helligkeit (50%)
#define BRIGHTNESS_LOW       60    // Niedrige Helligkeit
#define BRIGHTNESS_DIM       10    // Sehr dunkel (Hintergrund)
#define BRIGHTNESS_OFF       0     // Aus

// ============================================================
// Farb-Konstanten (R, G, B Werte)
// ============================================================

// Grundfarben
#define COLOR_WHITE_R        255
#define COLOR_WHITE_G        255
#define COLOR_WHITE_B        255

#define COLOR_RED_R          255
#define COLOR_RED_G          0
#define COLOR_RED_B          0

#define COLOR_GREEN_R        0
#define COLOR_GREEN_G        200
#define COLOR_GREEN_B        0

#define COLOR_BLUE_R         0
#define COLOR_BLUE_G         0
#define COLOR_BLUE_B         120

#define COLOR_YELLOW_R       255
#define COLOR_YELLOW_G       150
#define COLOR_YELLOW_B       0

#define COLOR_ORANGE_R       255
#define COLOR_ORANGE_G       85
#define COLOR_ORANGE_B       0

#define COLOR_PURPLE_R       120
#define COLOR_PURPLE_G       0
#define COLOR_PURPLE_B       120

#define COLOR_CYAN_R         0
#define COLOR_CYAN_G         180
#define COLOR_CYAN_B         255

// ============================================================
// State-spezifische Helligkeiten
// ============================================================

#define IDLE_BRIGHTNESS      127   // IDLE State - dezent
#define BUSY_BRIGHTNESS      120   // BUSY State - blau
#define MOVE_BRIGHT          255   // Bewegungsrichtung - hell
#define MOVE_DIM             10    // Bewegung Hintergrund - dunkel
#define GOAL_BRIGHT          200   // Ziel erreicht - grün hell
#define GOAL_DIM             80    // Ziel erreicht - grün dezent
#define ERROR_BRIGHTNESS     255   // Fehler - volle Helligkeit
#define WARNING_BRIGHTNESS   200   // Warnung - Orange
#define START_MOVE_BRIGHT    150   // Start Bewegung - weiß
#define START_MOVE_DIM       60    // Start Bewegung fertig - dezent
#define STOP_MOVE_BRIGHTNESS 200   // Stop Bewegung - rot

// ============================================================
// Animation Timing (Millisekunden)
// ============================================================

#define TIMING_GREETING_PULSE    20    // Begrüßung Puls-Geschwindigkeit
#define TIMING_LOW_BATTERY_PULSE 30    // Batterie Puls-Geschwindigkeit
#define TIMING_BLINK_FAST        80    // Schnelles Blinken
#define TIMING_BLINK_MEDIUM      200   // Mittleres Blinken
#define TIMING_BLINK_SLOW        400   // Langsames Blinken
#define TIMING_GOAL_BLINK        250   // Ziel erreicht Blinken
#define TIMING_REVERSE_BLINK     180   // Rückwärts Blinken
#define TIMING_WAITING_WAVE      30    // Warte-Animation Geschwindigkeit

// ============================================================
// Pattern-Funktionen
// ============================================================

void patternGreeting(Adafruit_NeoPixel &s);
void patternIdle(Adafruit_NeoPixel &s);
void patternBusy(Adafruit_NeoPixel &s);
void patternStopBusy(Adafruit_NeoPixel &s);
void patternErrorMinorStuck(Adafruit_NeoPixel &s);
void patternErrorMinorNav(Adafruit_NeoPixel &s);
void patternRoomNotFound(Adafruit_NeoPixel &s);
void patternErrorMajor(Adafruit_NeoPixel &s);
void patternLowBattery(Adafruit_NeoPixel &s);
void patternMoveLeft(Adafruit_NeoPixel &s);
void patternMoveForward(Adafruit_NeoPixel &s);
void patternMoveRight(Adafruit_NeoPixel &s);
void patternMoveBackward(Adafruit_NeoPixel &s);
void patternStartMove(Adafruit_NeoPixel &s);
void patternStopMove(Adafruit_NeoPixel &s);
void patternGoalReached(Adafruit_NeoPixel &s);
void patternReverse(Adafruit_NeoPixel &s);
void patternSpeaking(Adafruit_NeoPixel &s);
void patternWaiting(Adafruit_NeoPixel &s);

#endif

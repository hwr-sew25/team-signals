#include <Arduino.h>
#include "patterns.h"

// Externes Flag aus led_controller.ino - zeigt an, dass State gewechselt hat
extern bool stateJustChanged;

// ============================================================
// Helper-Funktionen
// ============================================================

// Helper: setzt alle Pixel auf eine Farbe und zeigt an
static void fillAll(Adafruit_NeoPixel &s, uint32_t color) {
  for (uint16_t i = 0; i < s.numPixels(); i++) {
    s.setPixelColor(i, color);
  }
  s.show();
}

// Helper: skaliert eine RGB-Farbe mit brightness (0..255)
static uint32_t scaleColor(Adafruit_NeoPixel &s, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  uint16_t rr = (uint16_t)r * brightness / 255;
  uint16_t gg = (uint16_t)g * brightness / 255;
  uint16_t bb = (uint16_t)b * brightness / 255;
  return s.Color((uint8_t)rr, (uint8_t)gg, (uint8_t)bb);
}

// Helper: Setzt einen LED-Bereich (Start bis End, inklusive) auf eine Farbe
static void fillRange(Adafruit_NeoPixel &s, uint16_t startIdx, uint16_t endIdx, uint32_t color) {
  for (uint16_t i = startIdx; i <= endIdx && i < s.numPixels(); i++) {
    s.setPixelColor(i, color);
  }
}

// ============================================================
// GREETING - Pulsierendes Blau
// ============================================================

void patternGreeting(Adafruit_NeoPixel &s) {
  static int b = 0;
  static int dir = 1;
  static unsigned long last = 0;

  // Bei State-Wechsel: Interne Variablen zurücksetzen
  if (stateJustChanged) {
    b = 0;
    dir = 1;
    last = 0;
  }

  if (millis() - last < TIMING_GREETING_PULSE) return;
  last = millis();

  b += dir * 2;
  if (b >= 150) dir = -1;
  if (b <= 10)  dir = 1;

  uint32_t c = s.Color(0, 0, (uint8_t)b);
  fillAll(s, c);
}

// ============================================================
// IDLE - Dezentes Weiß
// ============================================================

void patternIdle(Adafruit_NeoPixel &s) {
  uint32_t c = s.Color(IDLE_BRIGHTNESS, IDLE_BRIGHTNESS, IDLE_BRIGHTNESS);
  fillAll(s, c);
}

// ============================================================
// ERROR_MINOR_STUCK - Gelb blinkend
// ============================================================

void patternErrorMinorStuck(Adafruit_NeoPixel &s) {
  static bool on = false;
  static unsigned long last = 0;

  // Bei State-Wechsel: Interne Variablen zurücksetzen
  if (stateJustChanged) {
    on = false;
    last = 0;
  }

  if (millis() - last < TIMING_BLINK_SLOW) return;
  last = millis();

  on = !on;
  uint32_t c = on ? s.Color(COLOR_YELLOW_R, COLOR_YELLOW_G, COLOR_YELLOW_B) 
                  : s.Color(BRIGHTNESS_OFF, BRIGHTNESS_OFF, BRIGHTNESS_OFF);
  fillAll(s, c);
}

// ============================================================
// ERROR_MINOR_NAV - Gelb durchgehend
// ============================================================

void patternErrorMinorNav(Adafruit_NeoPixel &s) {
  uint32_t c = s.Color(COLOR_YELLOW_R, COLOR_YELLOW_G, COLOR_YELLOW_B);
  fillAll(s, c);
}

// ============================================================
// ROOM_NOT_FOUND - Rot blinken, dann Orange
// ============================================================

void patternRoomNotFound(Adafruit_NeoPixel &s) {
  static bool on = false;
  static unsigned long last = 0;
  static int blinkCount = 0;
  static bool blinkFinished = false;

  // Bei State-Wechsel: Interne Variablen zurücksetzen
  if (stateJustChanged) {
    on = false;
    last = 0;
    blinkCount = 0;
    blinkFinished = false;
  }

  // Nach dem Blinken: Orange durchgehend anzeigen
  if (blinkFinished) {
    fillAll(s, s.Color(WARNING_BRIGHTNESS, ROOM_NOT_FOUND_ORANGE_G, 0));  // Dezentes Orange
    return;
  }

  if (millis() - last < TIMING_BLINK_MEDIUM) return;
  last = millis();

  // 2x kurz rot blinken (4 Zustandswechsel)
  if (blinkCount < 4) {
    on = !on;
    blinkCount++;
    uint32_t c = on ? s.Color(ERROR_BRIGHTNESS, 0, 0) 
                    : s.Color(BRIGHTNESS_OFF, BRIGHTNESS_OFF, BRIGHTNESS_OFF);
    fillAll(s, c);
  } else {
    blinkFinished = true;
    blinkCount = 0;
    fillAll(s, s.Color(WARNING_BRIGHTNESS, ROOM_NOT_FOUND_ORANGE_G, 0));
  }
}

// ============================================================
// ERROR_MAJOR - Rot konstant
// ============================================================

void patternErrorMajor(Adafruit_NeoPixel &s) {
  uint32_t c = s.Color(ERROR_BRIGHTNESS, COLOR_RED_G, COLOR_RED_B);
  fillAll(s, c);
}

// ============================================================
// LOW_BATTERY - Sanfter Orange-Puls
// ============================================================

void patternLowBattery(Adafruit_NeoPixel &s) {
  static int b = 40;
  static int dir = 1;
  static unsigned long last = 0;

  // Bei State-Wechsel: Interne Variablen zurücksetzen
  if (stateJustChanged) {
    b = 40;
    dir = 1;
    last = 0;
  }

  if (millis() - last < TIMING_LOW_BATTERY_PULSE) return;
  last = millis();

  b += 3 * dir;
  if (b >= 180) dir = -1;
  if (b <= 40)  dir = 1;

  uint32_t c = scaleColor(s, COLOR_ORANGE_R, COLOR_ORANGE_G, COLOR_ORANGE_B, (uint8_t)b);
  fillAll(s, c);
}

// ============================================================
// MOVE-Patterns - Richtungsanzeige
// Angepasst an physische LED-Strip Befestigung:
// - Backward: LEDs 0-13
// - Right:    LEDs 14-28
// - Forward:  LEDs 29-43
// - Left:     LEDs 44-63
// ============================================================

void patternMoveLeft(Adafruit_NeoPixel &s) {
  // Links: LEDs 44-63 leuchten hell, Rest dunkel
  uint32_t dimColor = s.Color(MOVE_DIM, MOVE_DIM, MOVE_DIM);
  for (uint16_t i = 0; i < s.numPixels(); i++) {
    s.setPixelColor(i, dimColor);
  }
  
  uint32_t brightColor = s.Color(MOVE_BRIGHT, MOVE_BRIGHT, MOVE_BRIGHT);
  fillRange(s, SEG_LEFT_START, SEG_LEFT_END, brightColor);
  
  s.show();
}

void patternMoveForward(Adafruit_NeoPixel &s) {
  // Vorwärts: LEDs 29-43 leuchten hell, Rest dunkel
  uint32_t dimColor = s.Color(MOVE_DIM, MOVE_DIM, MOVE_DIM);
  for (uint16_t i = 0; i < s.numPixels(); i++) {
    s.setPixelColor(i, dimColor);
  }
  
  uint32_t brightColor = s.Color(MOVE_BRIGHT, MOVE_BRIGHT, MOVE_BRIGHT);
  fillRange(s, SEG_FORWARD_START, SEG_FORWARD_END, brightColor);
  
  s.show();
}

void patternMoveRight(Adafruit_NeoPixel &s) {
  // Rechts: LEDs 14-28 leuchten hell, Rest dunkel
  uint32_t dimColor = s.Color(MOVE_DIM, MOVE_DIM, MOVE_DIM);
  for (uint16_t i = 0; i < s.numPixels(); i++) {
    s.setPixelColor(i, dimColor);
  }
  
  uint32_t brightColor = s.Color(MOVE_BRIGHT, MOVE_BRIGHT, MOVE_BRIGHT);
  fillRange(s, SEG_RIGHT_START, SEG_RIGHT_END, brightColor);
  
  s.show();
}

void patternMoveBackward(Adafruit_NeoPixel &s) {
  // Rückwärts: LEDs 0-13 leuchten hell, Rest dunkel
  uint32_t dimColor = s.Color(MOVE_DIM, MOVE_DIM, MOVE_DIM);
  for (uint16_t i = 0; i < s.numPixels(); i++) {
    s.setPixelColor(i, dimColor);
  }
  
  uint32_t brightColor = s.Color(MOVE_BRIGHT, MOVE_BRIGHT, MOVE_BRIGHT);
  fillRange(s, SEG_BACKWARD_START, SEG_BACKWARD_END, brightColor);
  
  s.show();
}

// ============================================================
// START_MOVE - Kurzes weißes Blinken
// ============================================================

void patternStartMove(Adafruit_NeoPixel &s) {
  static bool on = false;
  static unsigned long last = 0;
  static int blinkCount = 0;
  static bool blinkFinished = false;

  // Bei State-Wechsel: Interne Variablen zurücksetzen
  if (stateJustChanged) {
    on = false;
    last = 0;
    blinkCount = 0;
    blinkFinished = false;
  }

  // Nach dem Blinken: Dezentes Weiß (bereit zur Bewegung)
  if (blinkFinished) {
    fillAll(s, s.Color(START_MOVE_DIM, START_MOVE_DIM, START_MOVE_DIM));
    return;
  }

  if (millis() - last < TIMING_BLINK_FAST) return;
  last = millis();

  // 4x blinken (2 komplette An-Aus-Zyklen)
  if (blinkCount < 4) {
    on = !on;
    blinkCount++;
    uint32_t c = on ? s.Color(START_MOVE_BRIGHT, START_MOVE_BRIGHT, START_MOVE_BRIGHT) 
                    : s.Color(BRIGHTNESS_OFF, BRIGHTNESS_OFF, BRIGHTNESS_OFF);
    fillAll(s, c);
  } else {
    blinkFinished = true;
    fillAll(s, s.Color(START_MOVE_DIM, START_MOVE_DIM, START_MOVE_DIM));
  }
}

// ============================================================
// STOP_MOVE - Rot konstant (Bremsen)
// ============================================================

void patternStopMove(Adafruit_NeoPixel &s) {
  uint32_t c = s.Color(STOP_MOVE_BRIGHTNESS, COLOR_RED_G, COLOR_RED_B);
  fillAll(s, c);
}

// ============================================================
// GOAL_REACHED - Grün 2x blinken, dann dezentes Grün
// ============================================================

void patternGoalReached(Adafruit_NeoPixel &s) {
  static bool on = false;
  static unsigned long last = 0;
  static int blinkCount = 0;
  static bool blinkFinished = false;

  // Bei State-Wechsel: Interne Variablen zurücksetzen
  if (stateJustChanged) {
    on = false;
    last = 0;
    blinkCount = 0;
    blinkFinished = false;
  }

  // Nach dem Blinken: Sanftes Grün anzeigen
  if (blinkFinished) {
    fillAll(s, s.Color(COLOR_GREEN_R, GOAL_DIM, COLOR_GREEN_B));
    return;
  }

  if (millis() - last < TIMING_GOAL_BLINK) return;
  last = millis();

  // 2x grün blinken (4 Zustandswechsel)
  if (blinkCount < 4) {
    on = !on;
    blinkCount++;
    uint32_t c = on ? s.Color(COLOR_GREEN_R, GOAL_BRIGHT, COLOR_GREEN_B) 
                    : s.Color(BRIGHTNESS_OFF, BRIGHTNESS_OFF, BRIGHTNESS_OFF);
    fillAll(s, c);
  } else {
    blinkFinished = true;
    fillAll(s, s.Color(COLOR_GREEN_R, GOAL_DIM, COLOR_GREEN_B));
  }
}

// ============================================================
// SPEAKING - Lila konstant
// ============================================================

void patternSpeaking(Adafruit_NeoPixel &s) {
  uint32_t c = s.Color(COLOR_PURPLE_R, COLOR_PURPLE_G, COLOR_PURPLE_B);
  fillAll(s, c);
}

// ============================================================
// WAITING - Lichtwellen-Animation
// ============================================================

void patternWaiting(Adafruit_NeoPixel &s) {
  static uint16_t wavePos = 0;
  static unsigned long last = 0;
  
  // Bei State-Wechsel: Interne Variablen zurücksetzen
  if (stateJustChanged) {
    wavePos = 0;
    last = 0;
  }
  
  if (millis() - last < TIMING_WAITING_WAVE) return;
  last = millis();
  
  uint16_t numLeds = s.numPixels();
  
  // Alle LEDs durchgehen
  for (uint16_t i = 0; i < numLeds; i++) {
    // Berechne Distanz zur Wellenposition (zirkular)
    int16_t dist = (int16_t)i - (int16_t)wavePos;
    
    // Zirkulare Distanz (kürzester Weg im Ring)
    if (dist < 0) dist += numLeds;
    if (dist > (int16_t)(numLeds / 2)) dist = numLeds - dist;
    
    // Wellenlänge: ca. 8 LEDs hell, dann Fade-out
    uint8_t brightness = 0;
    
    if (dist == 0) {
      brightness = BRIGHTNESS_FULL;
    } else if (dist == 1) {
      brightness = BRIGHTNESS_HIGH;
    } else if (dist == 2) {
      brightness = 150;
    } else if (dist == 3) {
      brightness = 100;
    } else if (dist == 4) {
      brightness = BRIGHTNESS_LOW;
    } else if (dist == 5) {
      brightness = 30;
    } else if (dist == 6) {
      brightness = 15;
    } else {
      brightness = 5;  // Hintergrund (sehr dunkel)
    }
    
    // Cyan-Blau Farbe für die Welle
    uint32_t c = scaleColor(s, COLOR_CYAN_R, COLOR_CYAN_G, COLOR_CYAN_B, brightness);
    s.setPixelColor(i, c);
  }
  
  s.show();
  
  // Welle weiterbewegen
  wavePos++;
  if (wavePos >= numLeds) wavePos = 0;
}

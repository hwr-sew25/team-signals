#include <Arduino.h>
#include "patterns.h"

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

/* ------------------- PATTERNS ------------------- */

void patternGreeting(Adafruit_NeoPixel &s) {
  // Pulsierendes Blau
  static int b = 0;
  static int dir = 1;
  static unsigned long last = 0;

  if (millis() - last < 20) return;
  last = millis();

  b += dir * 2;
  if (b >= 150) dir = -1;
  if (b <= 10)  dir = 1;

  uint32_t c = s.Color(0, 0, (uint8_t)b);
  fillAll(s, c);
}

void patternIdle(Adafruit_NeoPixel &s) {
  // Idle bei 50% der globalen Helligkeit (= 25% absolut)
  uint32_t c = s.Color(127, 127, 127);
  fillAll(s, c);
}

void patternBusy(Adafruit_NeoPixel &s) {
  // Blau konstant
  uint32_t c = s.Color(0, 0, 120);
  fillAll(s, c);
}

void patternStopBusy(Adafruit_NeoPixel &s) {
  // Zurück zu Idle-Pattern (50% der globalen = 25% absolut)
  uint32_t c = s.Color(127, 127, 127);
  fillAll(s, c);
}

void patternErrorMinorStuck(Adafruit_NeoPixel &s) {
  // Gelb durchgehend für "Stuck" Fehler
  uint32_t c = s.Color(255, 150, 0);
  fillAll(s, c);
}

void patternErrorMinorNav(Adafruit_NeoPixel &s) {
  // Gelb durchgehend für Navigation Error
  uint32_t c = s.Color(255, 150, 0);
  fillAll(s, c);
}

void patternRoomNotFound(Adafruit_NeoPixel &s) {
  // Rot kurz blinkend für "Raum existiert nicht"
  static bool on = false;
  static unsigned long last = 0;
  static int blinkCount = 0;

  if (millis() - last < 200) return;
  last = millis();

  // Nur 1x kurz rot blinken, dann aus
  if (blinkCount < 2) {
    on = !on;
    blinkCount++;
    uint32_t c = on ? s.Color(255, 0, 0) : s.Color(0, 0, 0);
    fillAll(s, c);
  } else {
    // Nach dem Blinken: Reset für nächsten Aufruf
    blinkCount = 0;
    fillAll(s, s.Color(0, 0, 0));
  }
}

void patternErrorMajor(Adafruit_NeoPixel &s) {
  // Rot konstant (später evtl. schneller Blink)
  uint32_t c = s.Color(255, 0, 0);
  fillAll(s, c);
}

void patternLowBattery(Adafruit_NeoPixel &s) {
  // Sanfter Orange-Puls
  static int b = 40;
  static int dir = 1;
  static unsigned long last = 0;

  if (millis() - last < 30) return;
  last = millis();

  b += 3 * dir;
  if (b >= 180) dir = -1;
  if (b <= 40)  dir = 1;

  uint32_t c = scaleColor(s, 255, 85, 0, (uint8_t)b);
  fillAll(s, c);
}

// Helper: Setzt ein bestimmtes Segment auf eine Farbe
static void fillSegment(Adafruit_NeoPixel &s, uint8_t segment, uint32_t color) {
  uint16_t startIdx = segment * PATTERN_SEGMENT_SIZE;
  uint16_t endIdx = startIdx + PATTERN_SEGMENT_SIZE;
  
  for (uint16_t i = startIdx; i < endIdx && i < s.numPixels(); i++) {
    s.setPixelColor(i, color);
  }
}

void patternMoveLeft(Adafruit_NeoPixel &s) {
  // Links: Segment 0 (LEDs 0-15) leuchtet hell, Rest dunkel
  // Konstantes Weiß ohne Pulsieren
  
  // Alle LEDs erst dunkel machen
  uint32_t dimColor = s.Color(10, 10, 10);
  for (uint16_t i = 0; i < s.numPixels(); i++) {
    s.setPixelColor(i, dimColor);
  }
  
  // Segment 0 (Links) hell machen
  uint32_t brightColor = s.Color(255, 255, 255);  // Helles Weiß
  fillSegment(s, PATTERN_SEG_LEFT, brightColor);
  
  s.show();
}

void patternMoveForward(Adafruit_NeoPixel &s) {
  // Vorwärts: Segment 1 (LEDs 16-31) leuchtet hell, Rest dunkel
  // Konstantes Weiß ohne Pulsieren
  
  // Alle LEDs erst dunkel machen
  uint32_t dimColor = s.Color(10, 10, 10);
  for (uint16_t i = 0; i < s.numPixels(); i++) {
    s.setPixelColor(i, dimColor);
  }
  
  // Segment 1 (Vorne) hell machen
  uint32_t brightColor = s.Color(255, 255, 255);  // Helles Weiß
  fillSegment(s, PATTERN_SEG_FORWARD, brightColor);
  
  s.show();
}

void patternMoveRight(Adafruit_NeoPixel &s) {
  // Rechts: Segment 2 (LEDs 32-47) leuchtet hell, Rest dunkel
  // Konstantes Weiß ohne Pulsieren
  
  // Alle LEDs erst dunkel machen
  uint32_t dimColor = s.Color(10, 10, 10);
  for (uint16_t i = 0; i < s.numPixels(); i++) {
    s.setPixelColor(i, dimColor);
  }
  
  // Segment 2 (Rechts) hell machen
  uint32_t brightColor = s.Color(255, 255, 255);  // Helles Weiß
  fillSegment(s, PATTERN_SEG_RIGHT, brightColor);
  
  s.show();
}

void patternMoveBackward(Adafruit_NeoPixel &s) {
  // Rückwärts: Segment 3 (LEDs 48-63) leuchtet hell, Rest dunkel
  // Konstantes Weiß ohne Pulsieren
  
  // Alle LEDs erst dunkel machen
  uint32_t dimColor = s.Color(10, 10, 10);
  for (uint16_t i = 0; i < s.numPixels(); i++) {
    s.setPixelColor(i, dimColor);
  }
  
  // Segment 3 (Hinten) hell machen
  uint32_t brightColor = s.Color(255, 255, 255);  // Helles Weiß
  fillSegment(s, PATTERN_SEG_BACKWARD, brightColor);
  
  s.show();
}

void patternStartMove(Adafruit_NeoPixel &s) {
  // Kurzes Blinken (weiß) ohne delay
  static bool on = false;
  static unsigned long last = 0;

  if (millis() - last < 120) return;
  last = millis();

  on = !on;
  uint32_t c = on ? s.Color(150, 150, 150) : s.Color(0, 0, 0);
  fillAll(s, c);
}

void patternStopMove(Adafruit_NeoPixel &s) {
  // Rot konstant (Bremsen)
  uint32_t c = s.Color(200, 0, 0);
  fillAll(s, c);
}

void patternGoalReached(Adafruit_NeoPixel &s) {
  // Grün aufblinkend für "Ziel erreicht"
  static bool on = false;
  static unsigned long last = 0;
  static int blinkCount = 0;

  if (millis() - last < 150) return;
  last = millis();

  // 2x schnell grün blinken
  if (blinkCount < 4) {
    on = !on;
    blinkCount++;
    uint32_t c = on ? s.Color(0, 200, 0) : s.Color(0, 0, 0);
    fillAll(s, c);
  } else {
    // Nach dem Blinken: zurück zu sanftem Grün
    blinkCount = 0;
    fillAll(s, s.Color(0, 80, 0));
  }
}

void patternReverse(Adafruit_NeoPixel &s) {
  // Weiß blinkend ohne delay
  static bool on = false;
  static unsigned long last = 0;

  if (millis() - last < 180) return;
  last = millis();

  on = !on;
  uint32_t c = on ? s.Color(120, 120, 120) : s.Color(0, 0, 0);
  fillAll(s, c);
}

void patternSpeaking(Adafruit_NeoPixel &s) {
  // Lila konstant (Ton kommt später)
  uint32_t c = s.Color(120, 0, 120);
  fillAll(s, c);
}

void patternWaiting(Adafruit_NeoPixel &s) {
  // Lichtwellen-Animation wie ein Ladekreis
  // Eine helle "Welle" wandert durch alle 64 LEDs
  
  static uint16_t wavePos = 0;       // Aktuelle Position der Welle (0-63)
  static unsigned long last = 0;
  
  // Geschwindigkeit: 30ms pro Schritt = ca. 2 Sekunden für einen Umlauf
  if (millis() - last < 30) return;
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
      brightness = 255;  // Hellster Punkt
    } else if (dist == 1) {
      brightness = 200;
    } else if (dist == 2) {
      brightness = 150;
    } else if (dist == 3) {
      brightness = 100;
    } else if (dist == 4) {
      brightness = 60;
    } else if (dist == 5) {
      brightness = 30;
    } else if (dist == 6) {
      brightness = 15;
    } else {
      brightness = 5;    // Hintergrund (sehr dunkel)
    }
    
    // Cyan-Blau Farbe für die Welle (wie ein moderner Loading-Spinner)
    uint32_t c = scaleColor(s, 0, 180, 255, brightness);
    s.setPixelColor(i, c);
  }
  
  s.show();
  
  // Welle weiterbewegen
  wavePos++;
  if (wavePos >= numLeds) wavePos = 0;
}


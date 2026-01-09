#ifndef PATTERNS_H
#define PATTERNS_H

#include <Adafruit_NeoPixel.h>

// Segment-Konstanten für 64 LEDs (4 Segmente à 16 LEDs)
#define PATTERN_SEGMENT_SIZE 16
#define PATTERN_SEG_LEFT     0   // LEDs 0-15
#define PATTERN_SEG_FORWARD  1   // LEDs 16-31
#define PATTERN_SEG_RIGHT    2   // LEDs 32-47
#define PATTERN_SEG_BACKWARD 3   // LEDs 48-63

void patternGreeting(Adafruit_NeoPixel &s);
void patternIdle(Adafruit_NeoPixel &s);
void patternBusy(Adafruit_NeoPixel &s);
void patternStopBusy(Adafruit_NeoPixel &s);
void patternErrorMinorStuck(Adafruit_NeoPixel &s);
void patternErrorMinorNav(Adafruit_NeoPixel &s);
void patternRoomNotFound(Adafruit_NeoPixel &s);
void patternErrorMajor(Adafruit_NeoPixel &s);
void patternLowBattery(Adafruit_NeoPixel &s);
void patternMove(Adafruit_NeoPixel &s);
void patternMoveDirection(Adafruit_NeoPixel &s, uint8_t direction);
void patternStartMove(Adafruit_NeoPixel &s);
void patternStopMove(Adafruit_NeoPixel &s);
void patternGoalReached(Adafruit_NeoPixel &s);
void patternReverse(Adafruit_NeoPixel &s);
void patternSpeaking(Adafruit_NeoPixel &s);
void patternWaiting(Adafruit_NeoPixel &s);

#endif



#ifndef PATTERNS_H
#define PATTERNS_H

#include <Adafruit_NeoPixel.h>

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
void patternStartMove(Adafruit_NeoPixel &s);
void patternStopMove(Adafruit_NeoPixel &s);
void patternGoalReached(Adafruit_NeoPixel &s);
void patternReverse(Adafruit_NeoPixel &s);
void patternSpeaking(Adafruit_NeoPixel &s);

#endif



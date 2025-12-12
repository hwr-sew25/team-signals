#ifndef PATTERNS_H
#define PATTERNS_H

#include <Adafruit_NeoPixel.h>

void patternGreeting(Adafruit_NeoPixel &s) {
  static int b = 0;
  static int dir = 1;

  b += dir * 2;
  if (b > 150) dir = -1;
  if (b < 10)  dir = 1;

  for (int i=0; i<s.numPixels(); i++)
    s.setPixelColor(i, s.Color(0, 0, b));
  s.show();
  delay(20);
}

void patternIdle(Adafruit_NeoPixel &s) {
  for (int i=0; i<s.numPixels(); i++)
    s.setPixelColor(i, s.Color(5, 5, 5));
  s.show();
}

void patternBusy(Adafruit_NeoPixel &s) {
  for (int i=0; i<s.numPixels(); i++)
    s.setPixelColor(i, s.Color(0, 0, 120));
  s.show();
}

void patternErrorMinor(Adafruit_NeoPixel &s) {
  for (int i=0; i<s.numPixels(); i++)
    s.setPixelColor(i, s.Color(255, 150, 0));
  s.show();
}

void patternErrorMajor(Adafruit_NeoPixel &s) {
  for (int i=0; i<s.numPixels(); i++)
    s.setPixelColor(i, s.Color(255, 0, 0));
  s.show();
}

void patternLowBattery(Adafruit_NeoPixel &s) {
  static int b=0, dir=1;
  b += 3*dir;
  if(b > 180) dir = -1;
  if(b < 40)  dir = 1;

  for(int i=0; i<s.numPixels(); i++)
    s.setPixelColor(i, s.Color(b, b/3, 0));
  s.show();
  delay(30);
}

void patternMove(Adafruit_NeoPixel &s) {
  for(int i=0;i<s.numPixels();i++)
    s.setPixelColor(i, s.Color(50,50,50));
  s.show();
}

void patternStartMove(Adafruit_NeoPixel &s) {
  static bool on=false;
  on = !on;

  uint32_t c = on ? s.Color(150,150,150) : s.Color(0,0,0);
  for(int i=0;i<s.numPixels();i++)
    s.setPixelColor(i,c);
  s.show();
  delay(120);
}

void patternStopMove(Adafruit_NeoPixel &s) {
  for(int i=0;i<s.numPixels();i++)
    s.setPixelColor(i, s.Color(200,0,0));
  s.show();
}

void patternReverse(Adafruit_NeoPixel &s) {
  static bool on=false;
  on = !on;
  uint32_t c = on ? s.Color(120,120,120) : s.Color(0,0,0);

  for(int i=0;i<s.numPixels();i++)
    s.setPixelColor(i,c);
  s.show();
  delay(180);
}

void patternSpeaking(Adafruit_NeoPixel &s) {
  for(int i=0;i<s.numPixels();i++)
    s.setPixelColor(i, s.Color(120,0,120));
  s.show();
}

#endif


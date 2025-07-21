#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class Neopixel
{
    Adafruit_NeoPixel *_neopixel;

public:
    Neopixel(uint8_t pin);
    void init(uint8_t power);
    void noticedPink();
    void noticedGreen();
    void noticedRed();
    void noticedBlue();
    void noticedWhite();
    void noticedTime(bool permiddionTime);

private:
    uint8_t _power;
};
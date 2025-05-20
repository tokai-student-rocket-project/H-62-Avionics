#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class Neopixel
{
    Adafruit_NeoPixel *_neopixel;

public:
    Neopixel(uint8_t pin);
    void init(uint8_t power);
    void noticedGreen();
    void noticedBlue();
    void noticedPink();
    void noticedTime(bool permiddionTime);
};
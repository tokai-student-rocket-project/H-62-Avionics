#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Lib_Neopixel.hpp>

Neopixel::Neopixel(uint8_t pin)
{
    _neopixel = new Adafruit_NeoPixel(1, pin, NEO_GRB + NEO_KHZ800);
}

void Neopixel::init(uint8_t power)
{
    _neopixel->begin();
    pinMode(power, OUTPUT);
    digitalWrite(power, HIGH);
}

void Neopixel::noticedPink()
{
    _neopixel->clear();
    _neopixel->setPixelColor(0, _neopixel->Color(255, 51, 255));
    _neopixel->show();
}

void Neopixel::noticedGreen()
{
    _neopixel->clear();
    _neopixel->setPixelColor(0, _neopixel->Color(0, 150, 0));
    _neopixel->show();
}

void Neopixel::noticedBlue()
{
    _neopixel->clear();
    _neopixel->setPixelColor(0, _neopixel->Color(0, 0, 150));
    _neopixel->show();
}

void Neopixel::noticedTime(bool permissionTime)
{
    if (permissionTime == 1)
    {
        _neopixel->clear();
        _neopixel->setPixelColor(0, _neopixel->Color(255, 0, 0));
        _neopixel->show();
    }
}

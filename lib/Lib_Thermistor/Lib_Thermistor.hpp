#pragma once


#include <Arduino.h>


class Thermistor {
  uint8_t _pinNumber;

  const float T0 = 25.0 + 273.15;
  const float R0 = 10000.0;
  const float B = 3380.0;

public:
  Thermistor(uint8_t pinNumber);

  float getTemperature_degC();
};

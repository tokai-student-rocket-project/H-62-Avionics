#pragma once

#include <Arduino.h>


class RateMonitor {
public:
  float updateRate();
  float getRate();

  void print();

private:
  uint32_t _tref;
  float _rate;
};

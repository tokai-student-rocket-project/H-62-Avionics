#pragma once

#include <Arduino.h>


class CountDetector {
public:
  CountDetector(uint16_t threshold);

  void update(bool state);
  void reset();
  void onExceeded(void(*callback)());
  bool isExceeded();

private:
  uint16_t _threshold = 0;
  uint16_t _count = 0;

  void (*_onExceeded)();
};

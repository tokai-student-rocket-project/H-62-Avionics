#pragma once


#include <Arduino.h>


class OutputPin {
public:
  OutputPin(uint8_t pinNumber);

  void high();
  void low();
  void set(bool isHigh);
  bool get();
  void toggle();
  uint8_t number();

private:
  uint8_t _pinNumber;
};
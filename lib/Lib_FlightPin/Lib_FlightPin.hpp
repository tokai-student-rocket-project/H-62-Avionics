#pragma once


#include <Arduino.h>


class FlightPin {
public:
  FlightPin(uint8_t pinNumber);

  bool isClosed();
  bool isOpen();

private:
  uint8_t _pinNumber;
};

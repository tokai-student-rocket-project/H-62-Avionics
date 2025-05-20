#pragma once


#include <Arduino.h>
#include "Lib_Var.hpp"


class FlightMode {
public:
  FlightMode();
  FlightMode(Var::FlightMode initialMode);

  void change(Var::FlightMode newMode);
  Var::FlightMode current();
  uint8_t currentNumber();

  bool is(Var::FlightMode mode);
  bool isNot(Var::FlightMode mode);
  bool isFlying();
  bool isBetween(Var::FlightMode start, Var::FlightMode end);

  void print();

private:
  Var::FlightMode _currentMode;
};

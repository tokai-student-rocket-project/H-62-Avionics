#pragma once


#include <Arduino.h>
#include <TaskManager.h>
#include "Lib_OutputPin.hpp"


class Shiranui {
public:
  Shiranui(uint8_t pinNumber, String identify);

  void separate();
  bool isOn();

private:
  OutputPin* _pin;
  String _identify;
};
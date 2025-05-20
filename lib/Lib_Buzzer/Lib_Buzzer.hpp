#pragma once

#include <Arduino.h>
#include <TaskManager.h>
#include "Lib_OutputPin.hpp"

class Buzzer
{
public:
  Buzzer(uint8_t pinNumber, String identify);

  void beep(uint8_t times);
  void beepOnce();
  void beepTwice();
  void beepLongOnce();
  void beepLongThreeTimes();
  void beepAttention();
  void beepWarning();
  void beepMorse(String message);
  void beepEndless();
  void beepLong(uint8_t times);

  bool isOn();

private:
  OutputPin *_pin;
  String _identify;

  void dot();
  void dash();
  void letterPause();
  void wordPause();
};
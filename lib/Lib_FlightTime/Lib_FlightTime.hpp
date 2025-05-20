#pragma once


#include <Arduino.h>


class FlightTime {
public:
  uint16_t SEPARATION_1_PROTECTION_TIME;
  uint16_t SEPARATION_1_FORCE_TIME;
  uint16_t SEPARATION_2_PROTECTION_TIME;
  uint16_t SEPARATION_2_FORCE_TIME;
  uint16_t LANDING_TIME;

  void setZero();
  uint16_t get();
  bool isElapsed(uint16_t time);

  void print();

private:
  uint32_t _referenceTime = 0;
};

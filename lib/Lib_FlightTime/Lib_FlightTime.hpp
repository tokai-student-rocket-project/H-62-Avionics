#pragma once


#include <Arduino.h>


class FlightTime {
public:
  uint32_t SEPARATION_1_PROTECTION_TIME;
  uint32_t SEPARATION_1_FORCE_TIME;
  uint32_t SEPARATION_2_PROTECTION_TIME;
  uint32_t SEPARATION_2_FORCE_TIME;
  uint32_t LANDING_TIME;

  void setZero();
  uint32_t get();
  bool isElapsed(uint32_t time);

  void print();

private:
  uint32_t _referenceTime = 0;
};

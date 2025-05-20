#pragma once

#include <Arduino.h>
#include "Lib_FRAM.hpp"


class Logger {
public:
  Logger(uint32_t csFram0, uint32_t csFram1, uint32_t csFram2, uint32_t csFram3);

  void reset();
  void dump();
  void clear();

  uint32_t write(const uint8_t* data, uint32_t size);

  uint32_t getOffset();
  float getUsage();
  uint8_t framNumber();

private:
  uint32_t _offset = 0;

  FRAM* _fram0;
  FRAM* _fram1;
  FRAM* _fram2;
  FRAM* _fram3;
};
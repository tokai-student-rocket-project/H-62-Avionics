#pragma once


#include <Arduino.h>
#include <LoRa.h>


class Telemeter {
public:
  void initialize(int32_t frequency, int32_t bandwidth);

  void reserveData(const uint8_t* data, uint32_t size);
  void sendReservedData();

private:
  uint8_t reservedData[4096];
  uint32_t reservedSize = 0;
};

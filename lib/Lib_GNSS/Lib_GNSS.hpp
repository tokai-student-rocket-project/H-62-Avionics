#pragma once


#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


class GNSS {
public:
  void begin();

  bool available();
  float getLatitude();
  float getLongitude();
  uint8_t getSatelliteCount();
  bool isFixed();
  uint8_t getFixType();
  float getAltitude();
  float getSpeed();
  float getAccuracy();
  uint32_t getUnixEpoch();

  void print();

private:
  SFE_UBLOX_GNSS _gnss;
};

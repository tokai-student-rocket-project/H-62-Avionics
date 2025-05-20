#pragma once


#include <Arduino.h>
#include <Adafruit_INA219.h>


class PowerMonitor {
public:
  void initialize();

  void getVoltage(float* ground_V, float* battery_V, float* tie_V, float* bus_V);
  void getCurrent(float* ground_mA, float* battery_mA, float* tie_mA, float* bus_mA);
  void getPower(float* ground_mW, float* battery_mW, float* tie_mW, float* bus_mW);

private:
  Adafruit_INA219* _ground;
  Adafruit_INA219* _battery;
  Adafruit_INA219* _tie;
  Adafruit_INA219* _bus;
};

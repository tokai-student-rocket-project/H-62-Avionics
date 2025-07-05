#pragma once

#include <Arduino.h>
#include <Adafruit_INA228.h>

class PowerMonitor
{
public:
  void initialize();

  void getVoltage(float *ground_V, float *battery_V, float *bus_V);
  void getCurrent(float *ground_mA, float *battery_mA, float *bus_mA);
  void getPower(float *ground_mW, float *battery_mW, float *bus_mW);
  void getTemperature(float *ground_C, float *battery_C, float *bus_C);

private:
  Adafruit_INA228 *_ground;
  Adafruit_INA228 *_battery;
  Adafruit_INA228 *_bus;
};

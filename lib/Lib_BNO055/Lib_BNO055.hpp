#pragma once


#include <Arduino.h>
#include <Wire.h>


class BNO055 {
public:
  void begin();

  void getAcceleration(float* x_mps2, float* y_mps2, float* z_mps2);
  void getMagnetometer(float* x_nT, float* y_nT, float* z_nT);
  void getGyroscope(float* x_dps, float* y_dps, float* z_dps);

private:
  void write(uint8_t address, uint8_t data);
  void readVector3D(uint8_t address, float lsb, float* x, float* y, float* z);
};
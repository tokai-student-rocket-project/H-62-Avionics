#include "Lib_BNO055.hpp"

void BNO055::begin()
{
  delay(850);

  // OPR_MODE <- Config mode
  write(0x3D, 0x00);
  delay(30);

  // PAGE <- 1
  write(0x07, 1);

  // ACC_Config <- 16G, 1000Hz, Normal
  write(0x08, 0b00011111);
  delay(10);

  // MAG_Config <- 30Hz, Regular, Normal
  write(0x09, 0b00001111);
  delay(10);

  // GYR_Config_0 <- 2000dps, 523Hz
  write(0x0A, 0b00000000);
  delay(10);

  // GYR_Config_1 <- Normal
  write(0x0B, 0b00000000);
  delay(10);

  // PAGE <- 0
  write(0x07, 0);

  // PWR_MODE <- Normal mode
  write(0x3E, 0x00);
  delay(10);

  // SYS_TRIGGER <- Use external oscillator
  write(0x3F, 0b10000000);
  delay(10);

  // OPR_MODE <- NDOF mode
  write(0x3D, 0b00000111);
  delay(20);
}

void BNO055::getAcceleration(float *x_mps2, float *y_mps2, float *z_mps2)
{
  readVector3D(0x08, 100.0, x_mps2, y_mps2, z_mps2);
}

void BNO055::getMagnetometer(float *x_nT, float *y_nT, float *z_nT)
{
  readVector3D(0x0E, 16.0, x_nT, y_nT, z_nT);
}

void BNO055::getGyroscope(float *x_dps, float *y_dps, float *z_dps)
{
  readVector3D(0x14, 16.0, x_dps, y_dps, z_dps);
}

void BNO055::write(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(0x28);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

void BNO055::readVector3D(uint8_t address, float lsb, float *x, float *y, float *z)
{
  Wire.beginTransmission(0x28);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(0x28, 6);

  int16_t xRaw = ((int16_t)Wire.read()) | (((int16_t)Wire.read()) << 8);
  int16_t yRaw = ((int16_t)Wire.read()) | (((int16_t)Wire.read()) << 8);
  int16_t zRaw = ((int16_t)Wire.read()) | (((int16_t)Wire.read()) << 8);

  // 座標軸を合わせるためにxyzを入れ替えているので注意
  *x = ((float)zRaw) / lsb;
  *y = ((float)yRaw) / lsb;  // 軸の初期設定から90度反時計回りに動かしたので xRaw / Lsb のはず
  *z = ((float)xRaw) / -lsb; // 軸の初期設定から90度反時計回りに動かしたので yRaw / -Lsb のはず
}

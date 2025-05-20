#include "Lib_GNSS.hpp"

void GNSS::begin()
{
  Wire.begin();

  _gnss.begin();
  _gnss.setI2COutput(COM_TYPE_UBX);
  _gnss.setNavigationFrequency(10);
  _gnss.setAutoPVT(true);
}

bool GNSS::available()
{
  if (!_gnss.getPVT() || _gnss.getInvalidLlh())
  {
    return false;
  }

  // 受信しているなら保存しておく
  int32_t latitude = _gnss.getLatitude();
  int32_t longitude = _gnss.getLongitude();

  return latitude != 0 && longitude != 0;
}

float GNSS::getLatitude()
{
  return (float)_gnss.getLatitude() / 10000000.0;
}

float GNSS::getLongitude()
{
  return (float)_gnss.getLongitude() / 10000000.0;
}

uint8_t GNSS::getSatelliteCount()
{
  return _gnss.getSIV();
}

bool GNSS::isFixed()
{
  return _gnss.getGnssFixOk();
}

uint8_t GNSS::getFixType()
{
  return _gnss.getFixType();
}

float GNSS::getAltitude()
{
  return _gnss.getAltitude() / 1000.0;
}

float GNSS::getSpeed()
{
  return _gnss.getGroundSpeed() / 1000.0;
}

float GNSS::getAccuracy()
{
  return _gnss.getHorizontalAccEst() / 1000.0;
}

uint32_t GNSS::getUnixEpoch()
{
  return _gnss.getUnixEpoch();
}

void GNSS::print()
{
  Serial.print(isFixed());
  Serial.print("\t");
  Serial.print(getFixType());
  Serial.print("\t");
  Serial.print(getSatelliteCount());
  Serial.print("\t");
  Serial.print(getLatitude(), 6);
  Serial.print("\t");
  Serial.print(getLongitude(), 6);
  Serial.print("\t");
  Serial.print(getAltitude());
  Serial.print("\t");
  Serial.print(getSpeed());
  Serial.print("\n");
}
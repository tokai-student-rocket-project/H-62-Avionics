#include "Lib_PowerMonitor.hpp"

void PowerMonitor::initialize()
{
  _ground = new Adafruit_INA228();
  _ground->begin(0x40); // 0b100 0000
  _ground->setShunt(0.15, 10.0);
  _ground->setAveragingCount(INA228_COUNT_16);
  _ground->setVoltageConversionTime(INA228_TIME_150_us);
  _ground->setCurrentConversionTime(INA228_TIME_280_us);

  _battery = new Adafruit_INA228();
  _battery->begin(0x41); // 0b100 0001
  _battery->setShunt(0.15, 10.0);
  _battery->setAveragingCount(INA228_COUNT_16);
  _battery->setVoltageConversionTime(INA228_TIME_150_us);
  _battery->setCurrentConversionTime(INA228_TIME_280_us);

  _bus = new Adafruit_INA228();
  _bus->begin(0x45); // 0b100 0101
  _bus->setShunt(0.15, 10.0);
  _bus->setAveragingCount(INA228_COUNT_16);
  _bus->setVoltageConversionTime(INA228_TIME_150_us);
  _bus->setCurrentConversionTime(INA228_TIME_280_us);
}

void PowerMonitor::getVoltage(float *ground_V, float *battery_V, float *bus_V)
{
  *ground_V = _ground->getBusVoltage_V();
  *battery_V = _battery->getBusVoltage_V();
  *bus_V = _bus->getBusVoltage_V();
}

void PowerMonitor::getCurrent(float *ground_mA, float *battery_mA, float *bus_mA)
{
  *ground_mA = _ground->getCurrent_mA();
  *battery_mA = _battery->getCurrent_mA();
  *bus_mA = _bus->getCurrent_mA();
}

void PowerMonitor::getPower(float *ground_W, float *battery_W, float *bus_W)
{
  *ground_W = _ground->getPower_mW() / 1000.0;
  *battery_W = _battery->getPower_mW() / 1000.0;
  *bus_W = _bus->getPower_mW() / 1000.0;
}

void PowerMonitor::getTemperature(float *ground_C, float *battery_C, float *bus_C)
{
  *ground_C = _ground->readDieTemp();
  *battery_C = _battery->readDieTemp();
  *bus_C = _bus->readDieTemp();
}

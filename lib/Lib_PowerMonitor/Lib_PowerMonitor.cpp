#include "Lib_PowerMonitor.hpp"


void PowerMonitor::initialize() {
  _ground = new Adafruit_INA219(0b1000000);
  _ground->begin();
  _ground->setCalibration_32V_2A();

  _battery = new Adafruit_INA219(0b1000001);
  _battery->begin();
  _battery->setCalibration_32V_2A();

  _tie = new Adafruit_INA219(0b1000100);
  _tie->begin();
  _tie->setCalibration_32V_2A();

  _bus = new Adafruit_INA219(0b1000101);
  _bus->begin();
  _bus->setCalibration_32V_2A();
}


void PowerMonitor::getVoltage(float* ground_V, float* battery_V, float* tie_V, float* bus_V) {
  *ground_V = _ground->getBusVoltage_V();
  *battery_V = _battery->getBusVoltage_V();
  *tie_V = _tie->getBusVoltage_V();
  *bus_V = _bus->getBusVoltage_V();
}


void PowerMonitor::getCurrent(float* ground_mA, float* battery_mA, float* tie_mA, float* bus_mA) {
  *ground_mA = _ground->getCurrent_mA();
  *battery_mA = _battery->getCurrent_mA();
  *tie_mA = _tie->getCurrent_mA();
  *bus_mA = _bus->getCurrent_mA();
}


void PowerMonitor::getPower(float* ground_mW, float* battery_mW, float* tie_mW, float* bus_mW) {
  *ground_mW = _ground->getPower_mW();
  *battery_mW = _battery->getPower_mW();
  *tie_mW = _tie->getPower_mW();
  *bus_mW = _bus->getPower_mW();
}

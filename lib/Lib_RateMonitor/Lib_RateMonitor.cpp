#include "Lib_RateMonitor.hpp"


float RateMonitor::updateRate() {
  const uint32_t t = micros();
  const uint32_t dt = t - _tref;
  _tref = t;

  _rate = 1.0 / ((float)dt / 1000.0 / 1000.0);
  return _rate;
}


float RateMonitor::getRate() {
  return _rate;
}


void RateMonitor::print() {
  Serial.println(_rate);
}

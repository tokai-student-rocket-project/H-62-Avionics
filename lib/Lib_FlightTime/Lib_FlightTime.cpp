#include "Lib_FlightTime.hpp"


void FlightTime::setZero() {
  _referenceTime = millis();
}


uint32_t FlightTime::get() {
  return (uint32_t)(millis() - _referenceTime);
}


bool FlightTime::isElapsed(uint32_t time) {
  return get() >= time;
}


void FlightTime::print() {
  Serial.println(get() / 1000.0, 3);
}

#include "Lib_FlightTime.hpp"


void FlightTime::setZero() {
  _referenceTime = millis();
}


uint16_t FlightTime::get() {
  return (uint16_t)(millis() - _referenceTime);
}


bool FlightTime::isElapsed(uint16_t time) {
  return get() >= time;
}


void FlightTime::print() {
  Serial.println(get() / 1000.0, 3);
}

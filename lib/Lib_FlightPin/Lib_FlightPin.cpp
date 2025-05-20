#include "Lib_FlightPin.hpp"


FlightPin::FlightPin(uint8_t pinNumber) {
  _pinNumber = pinNumber;
  pinMode(_pinNumber, INPUT);
}


bool FlightPin::isClosed() {
  return digitalRead(_pinNumber) == HIGH;
}


bool FlightPin::isOpen() {
  return digitalRead(_pinNumber) == LOW;
}

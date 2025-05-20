#include "Lib_OutputPin.hpp"


OutputPin::OutputPin(uint8_t pinNumber) {
  _pinNumber = pinNumber;
  pinMode(pinNumber, OUTPUT);
}


void OutputPin::high() {
  digitalWrite(_pinNumber, HIGH);
}


void OutputPin::low() {
  digitalWrite(_pinNumber, LOW);
}


void OutputPin::set(bool isHigh) {
  digitalWrite(_pinNumber, isHigh ? HIGH : LOW);
}


bool OutputPin::get() {
  return digitalRead(_pinNumber) == HIGH;
}


void OutputPin::toggle() {
  digitalWrite(_pinNumber, !digitalRead(_pinNumber));
}


uint8_t OutputPin::number() {
  return _pinNumber;
}

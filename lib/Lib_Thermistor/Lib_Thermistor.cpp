#include "Lib_Thermistor.hpp"


Thermistor::Thermistor(uint8_t pinNumber) {
  _pinNumber = pinNumber;
}


float Thermistor::getTemperature_degC() {
  float readingValue = (float)analogRead(_pinNumber);
  float resistance = 10000.0 * readingValue / (4095.0 - readingValue);

  float temperatureBar = 1.0 / T0 + 1.0 / B * log(resistance / R0);
  return 1.0 / temperatureBar - 273.15;
}

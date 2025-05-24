#include "Lib_Shiranui.hpp"


Shiranui::Shiranui(uint8_t pinNumber, String identify) {
  _pin = new OutputPin(pinNumber);
  _identify = identify;

  Tasks.add(_identify, [&]() {_pin->low();});
}


void Shiranui::separate() {
  _pin->high();
  Tasks[_identify]->startOnceAfterSec(1);
}


bool Shiranui::isOn() {
  return _pin->get();
}

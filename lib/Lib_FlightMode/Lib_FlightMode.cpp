#include "Lib_FlightMode.hpp"

FlightMode::FlightMode()
{
  FlightMode(Var::FlightMode::STANDBY);
}

FlightMode::FlightMode(Var::FlightMode initialMode)
{
  _currentMode = initialMode;
}

void FlightMode::change(Var::FlightMode newMode)
{
  if (_currentMode == newMode)
    return;

  _currentMode = newMode;
}

Var::FlightMode FlightMode::current()
{
  return _currentMode;
}

uint8_t FlightMode::currentNumber()
{
  return static_cast<uint8_t>(current());
}

bool FlightMode::is(Var::FlightMode mode)
{
  return _currentMode == mode;
}

bool FlightMode::isNot(Var::FlightMode mode)
{
  return _currentMode != mode;
}

bool FlightMode::isFlying()
{
  return _currentMode == Var::FlightMode::POWERED_CLIMB || _currentMode == Var::FlightMode::FREE_CLIMB || _currentMode == Var::FlightMode::FREE_DESCENT || _currentMode == Var::FlightMode::DROGUE_CHUTE_DESCENT || _currentMode == Var::FlightMode::MAIN_CHUTE_DESCENT || _currentMode == Var::FlightMode::LANDED;
}

bool FlightMode::isBetween(Var::FlightMode start, Var::FlightMode end)
{
  return static_cast<uint8_t>(_currentMode) >= static_cast<uint8_t>(start) && static_cast<uint8_t>(_currentMode) <= static_cast<uint8_t>(end);
}

void FlightMode::print()
{
  Serial.print(">flightMode: ");
  Serial.println(static_cast<uint8_t>(_currentMode));
}
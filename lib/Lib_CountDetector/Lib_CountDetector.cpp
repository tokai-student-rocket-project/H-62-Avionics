#include "Lib_CountDetector.hpp"

CountDetector::CountDetector(uint16_t threshold)
{
  _threshold = threshold;
}

void CountDetector::update(bool state)
{
  _count = state ? _count + 1 : 0;

  if (_count > _threshold && _onExceeded)
  {
    _onExceeded();
  }
}

void CountDetector::reset()
{
  _count = 0;
}

void CountDetector::onExceeded(void (*callback)())
{
  _onExceeded = callback;
}

bool CountDetector::isExceeded()
{
  return _count > _threshold;
}

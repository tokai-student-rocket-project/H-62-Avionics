#include "Lib_Buzzer.hpp"

Buzzer::Buzzer(uint8_t pinNumber, String identify)
{
  _pin = new OutputPin(pinNumber);
  _identify = identify;

  Tasks.add(_identify, [&]()
            { _pin->toggle(); });
}

void Buzzer::beep(uint8_t times)
{
  _pin->low();
  Tasks[_identify]->startIntervalMsecForCount(100, times * 2);
}

void Buzzer::beepOnce()
{
  _pin->low();
  Tasks[_identify]->startIntervalMsecForCount(100, 2);
}

void Buzzer::beepLong(uint8_t times)
{
  _pin->low();
  Tasks[_identify]->startIntervalMsecForCount(400, times * 2);
}

void Buzzer::beepTwice()
{
  _pin->low();
  Tasks[_identify]->startIntervalMsecForCount(100, 4);
}

void Buzzer::beepLongOnce()
{
  _pin->low();
  Tasks[_identify]->startIntervalMsecForCount(400, 2);
}

void Buzzer::beepLongThreeTimes()
{
  _pin->low();
  Tasks[_identify]->startIntervalMsecForCount(400, 6);
}

void Buzzer::beepAttention()
{
  _pin->low();
  Tasks[_identify]->startIntervalMsecForCount(20, 10);
}

void Buzzer::beepWarning()
{
  _pin->low();
  Tasks[_identify]->startIntervalMsecForCount(40, 100);
}

void Buzzer::beepMorse(String message)
{
  uint16_t length = message.length();
  char letters[length];
  message.toCharArray(letters, length + 1);

  for (uint16_t index = 0; index < length; index++)
  {
    char letter = letters[index];
    if (letter == 'A')
    {
      dot();
      dash();
      letterPause();
    }
    if (letter == 'B')
    {
      dash();
      dot();
      dot();
      dot();
      letterPause();
    }
    if (letter == 'C')
    {
      dash();
      dot();
      dash();
      dot();
      letterPause();
    }
    if (letter == 'D')
    {
      dash();
      dot();
      dot();
      letterPause();
    }
    if (letter == 'E')
    {
      dot();
      letterPause();
    }
    if (letter == 'F')
    {
      dot();
      dot();
      dash();
      dot();
      letterPause();
    }
    if (letter == 'G')
    {
      dash();
      dash();
      dot();
      letterPause();
    }
    if (letter == 'H')
    {
      dot();
      dot();
      dot();
      dot();
      letterPause();
    }
    if (letter == 'I')
    {
      dot();
      dot();
      letterPause();
    }
    if (letter == 'J')
    {
      dot();
      dash();
      dash();
      dash();
      letterPause();
    }
    if (letter == 'K')
    {
      dash();
      dot();
      dash();
      letterPause();
    }
    if (letter == 'L')
    {
      dot();
      dash();
      dot();
      dot();
      letterPause();
    }
    if (letter == 'M')
    {
      dash();
      dash();
      letterPause();
    }
    if (letter == 'N')
    {
      dash();
      dot();
      letterPause();
    }
    if (letter == 'O')
    {
      dash();
      dash();
      dash();
      letterPause();
    }
    if (letter == 'P')
    {
      dot();
      dash();
      dash();
      dot();
      letterPause();
    }
    if (letter == 'Q')
    {
      dash();
      dash();
      dot();
      dash();
      letterPause();
    }
    if (letter == 'R')
    {
      dot();
      dash();
      dot();
      letterPause();
    }
    if (letter == 'S')
    {
      dot();
      dot();
      dot();
      letterPause();
    }
    if (letter == 'T')
    {
      dash();
      letterPause();
    }
    if (letter == 'U')
    {
      dot();
      dot();
      dash();
      letterPause();
    }
    if (letter == 'V')
    {
      dot();
      dot();
      dot();
      dash();
      letterPause();
    }
    if (letter == 'W')
    {
      dot();
      dash();
      dash();
      letterPause();
    }
    if (letter == 'X')
    {
      dash();
      dot();
      dot();
      dash();
      letterPause();
    }
    if (letter == 'Y')
    {
      dash();
      dot();
      dash();
      dash();
      letterPause();
    }
    if (letter == 'Z')
    {
      dash();
      dash();
      dot();
      dot();
      letterPause();
    }
    if (letter == ' ')
    {
      wordPause();
    }
  }
}

bool Buzzer::isOn()
{
  return _pin->get();
}

void Buzzer::dot()
{
  _pin->high();
  delay(70);
  _pin->low();
  delay(70);
}

void Buzzer::dash()
{
  _pin->high();
  delay(210);
  _pin->low();
  delay(70);
}

void Buzzer::letterPause()
{
  delay(210);
}

void Buzzer::wordPause()
{
  delay(490);
}

void Buzzer::beepEndless()
{
  Tasks[_identify]->startIntervalMsec(500);
}

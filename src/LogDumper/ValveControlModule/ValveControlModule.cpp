#include <MsgPacketizer.h>
#include "Lib_FRAM.hpp"
#include "Lib_NeoPixel.hpp"

FRAM fram0(28);

const uint8_t redled = 17;   // GPIO17
const uint8_t greenled = 16; // GPIO16
const uint8_t blueled = 25;  // GPIO25

Neopixel Status(12);

void dump(FRAM *fram)
{
  uint8_t data[4096];
  uint32_t size = 0;
  uint32_t writeAddress = 0;

  for (uint32_t address = 0; address < FRAM::LENGTH; address++)
  {
    data[writeAddress] = fram->read(address);
    size = writeAddress + 1;

    if (data[writeAddress] == 0x00)
    {
      writeAddress = 0;

      if (data[1] == 0x0A)
      {
        MsgPacketizer::feed(data, size);
      }
    }
    else
    {
      writeAddress++;
    }
  }
}

void printHeader()
{
  Serial.print("millis");
  Serial.print(",");
  Serial.print("motorTemperature");
  Serial.print(",");
  Serial.print("mcuTemperature");
  Serial.print(",");
  Serial.print("current");
  Serial.print(",");
  Serial.print("inputVoltage");
  Serial.print(",");
  Serial.print("currentPosition");
  Serial.print(",");
  Serial.print("currentDesiredPosition");
  Serial.print(",");
  Serial.print("currentVelocity");
  Serial.print("\n");
}

void setup()
{
  Serial.begin(115200);
  SPI.begin();

  MsgPacketizer::subscribe_manual(0x0A,
                                  [&](uint32_t millis,
                                      int16_t motorTemperature,
                                      int16_t mcuTemperature,
                                      int16_t current,
                                      int16_t inputVoltage,
                                      int16_t currentPosition,
                                      int16_t currentDesiredPosition,
                                      int16_t currentVelocity)
                                  {
                                    Serial.print(millis);
                                    Serial.print(",");
                                    Serial.print(motorTemperature);
                                    Serial.print(",");
                                    Serial.print(mcuTemperature);
                                    Serial.print(",");
                                    Serial.print(current);
                                    Serial.print(",");
                                    Serial.print(inputVoltage);
                                    Serial.print(",");
                                    Serial.print(currentPosition);
                                    Serial.print(",");
                                    Serial.print(currentDesiredPosition);
                                    Serial.print(",");
                                    Serial.print(currentVelocity);
                                    Serial.print("\n");
                                  });

  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(blueled, OUTPUT);
  digitalWrite(blueled, HIGH);  // RGB LED OFF
  digitalWrite(redled, HIGH);   // RGB LED OFF
  digitalWrite(greenled, HIGH); // RGB LED OFF
  Status.init(11);
  while (!Serial)
    ;
  delay(5000);
  printHeader();
  Status.noticedBlue();
  digitalWrite(blueled, LOW);

  dump(&fram0);

  digitalWrite(blueled, HIGH);
}

void loop()
{
}
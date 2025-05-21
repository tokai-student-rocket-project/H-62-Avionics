#include <MsgPacketizer.h>
#include "Lib_FRAM.hpp"

const uint32_t FRAM_CS_PIN0 = 28; // A2
const uint32_t FRAM_CS_PIN1 = 29; // A3 

FRAM fram0(FRAM_CS_PIN0);
FRAM fram1(FRAM_CS_PIN1);


const uint8_t redled = 17; //GPIO17
const uint8_t greenled = 16; //GPIO16
const uint8_t blueled = 25; //GPIO25


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
  Serial.print("\n");
}

void setup()
{
  Serial.begin(115200);
  SPI.begin();

  MsgPacketizer::subscribe_manual(0x0A,
                                  [&](uint32_t millis)
                                  {
                                    Serial.print(millis);
                                    Serial.print("\n");
                                  });

  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT); 
  pinMode(blueled, OUTPUT);
  digitalWrite(blueled, LOW);
  digitalWrite(redled, HIGH); // RGB LED OFF
  digitalWrite(greenled, HIGH); // RGB LED OFF
  while (!Serial);
  delay(5000);
  printHeader();

  dump(&fram0);
  dump(&fram1);
  
  digitalWrite(blueled, HIGH);
}

void loop()
{
}
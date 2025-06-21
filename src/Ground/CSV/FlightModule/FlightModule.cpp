#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  LoRa.begin(925.8E6);
  LoRa.setSignalBandwidth(500E3);

  MsgPacketizer::subscribe(LoRa, 0x0A,
                           [](
                               uint32_t millis,
                               char ident,
                               uint8_t flightMode,
                               uint16_t flightTime,
                               uint8_t loggerUsage,
                               bool doLogging,
                               uint8_t framNumber,
                               bool flightPinIsOpen,
                               bool sn3IsOn,
                               bool sn4IsOn,
                               bool isLaunchMode,
                               bool isFalling,
                               uint8_t fixType,
                               uint8_t satelliteCount,
                               float latitude,
                               float longitude,
                               int16_t height,
                               int16_t speed,
                               uint16_t accuracy,
                               int16_t motorTemperature,
                               int16_t mcuTemperature,
                               uint16_t inputVoltage,
                               int16_t currentPosition,
                               int16_t currentDesiredPosition)
                           {
                             Serial.print(LoRa.packetRssi());
                             Serial.print(",");
                             Serial.print(LoRa.packetSnr());
                             Serial.print(",");
                             Serial.print((String)ident);
                             Serial.print(",");
                             Serial.print((float)millis / 1000.0);
                             Serial.print(",");

                             Serial.print(doLogging);
                             Serial.print(",");
                             Serial.print(loggerUsage);
                             Serial.print(",");
                             Serial.print(framNumber);
                             Serial.print(",");

                             Serial.print(flightMode);
                             Serial.print(",");
                             Serial.print((float)flightTime / 1000.0);
                             Serial.print(",");
                             Serial.print(isLaunchMode);
                             Serial.print(",");
                             Serial.print(flightPinIsOpen);
                             Serial.print(",");
                             Serial.print(isFalling);
                             Serial.print(",");
                             Serial.print(sn3IsOn);
                             Serial.print(",");
                             Serial.print(sn4IsOn);
                             Serial.print(",");

                             Serial.print(fixType);
                             Serial.print(",");
                             Serial.print(satelliteCount);
                             Serial.print(",");
                             Serial.print(latitude);
                             Serial.print(",");
                             Serial.print(longitude);
                             Serial.print(",");
                             Serial.print((float)height / 10.0);
                             Serial.print(",");
                             Serial.print((float)speed / 10.0);
                             Serial.print(",");
                             Serial.print((float)accuracy / 10.0);
                             Serial.print(",");

                             Serial.print((float)motorTemperature / 100.0);
                             Serial.print(",");
                             Serial.print((float)mcuTemperature / 100.0);
                             Serial.print(",");
                             Serial.print((float)inputVoltage / 1000.0);
                             Serial.print(",");
                             Serial.print((float)currentPosition / 100.0);
                             Serial.print(",");
                             Serial.print((float)currentDesiredPosition / 100.0);

                             Serial.println();
                             Serial.flush();

                             digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                           });
}

void loop()
{
  Tasks.update();

  if (LoRa.parsePacket())
  {
    MsgPacketizer::parse();
  }
}

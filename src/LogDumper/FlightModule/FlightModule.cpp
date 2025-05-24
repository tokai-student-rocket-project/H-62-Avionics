#include <MsgPacketizer.h>
#include "Lib_FRAM.hpp"

FRAM fram0(7);
FRAM fram1(A1);
FRAM fram2(A2);

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
  Serial.print("ident");
  Serial.print(",");
  Serial.print("millis");
  Serial.print(",");
  Serial.print("flightTime");
  Serial.print(",");
  Serial.print("flightMode");
  Serial.print(",");
  Serial.print("loggerUsage");
  Serial.print(",");

  Serial.print("flightPinIsOpen");
  Serial.print(",");
  Serial.print("buzzerIsOn");
  Serial.print(",");
  Serial.print("sn3IsOn");
  Serial.print(",");

  Serial.print("isFalling");
  Serial.print(",");
  Serial.print("altitude");
  Serial.print(",");
  Serial.print("isLaunchMode");
  Serial.print(",");
  Serial.print("forceX_N");
  Serial.print(",");
  Serial.print("jerkX_mps3");
  Serial.print(",");

  Serial.print("gnssIsAvailable");
  Serial.print(",");
  Serial.print("unixEpoch");
  Serial.print(",");
  Serial.print("isFixed");
  Serial.print(",");
  Serial.print("fixType");
  Serial.print(",");
  Serial.print("satelliteCount");
  Serial.print(",");
  Serial.print("latitude");
  Serial.print(",");
  Serial.print("longitude");
  Serial.print(",");
  Serial.print("accuracy");
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
  Serial.print(",");
  Serial.print("currentSupplyPosition");
  Serial.print(",");
  Serial.print("voltage");
  Serial.print("\n");
}

void setup()
{
  Serial.begin(115200);
  SPI.begin();

  MsgPacketizer::subscribe_manual(0x0A,
                                  [&](char ident, uint32_t millis, uint16_t flightTime, uint8_t flightMode, float loggerUsage,
                                      bool flightPinIsOpen, bool buzzerIsOn, bool sn3IsOn,
                                      bool isFalling, float altitude, bool isLaunchMode, float forceX_N, float jerkX_mps3,
                                      bool gnssIsAvailable, uint32_t unixEpoch, bool isFixed, uint8_t fixType, uint8_t satelliteCount, float latitude, float longitude, float accuracy,
                                      float motorTemperature, float mcuTemperature, float current, float inputVoltage,
                                      float currentPosition, float currentDesiredPosition, float currentVelocity, float currentSupplyPosition, float voltage)
                                  {
                                    Serial.print(ident); Serial.print(",");
                                    Serial.print(millis); Serial.print(",");
                                    Serial.print(flightTime); Serial.print(",");
                                    Serial.print(flightMode); Serial.print(",");
                                    Serial.print(loggerUsage); Serial.print(",");

                                    Serial.print(flightPinIsOpen); Serial.print(",");
                                    Serial.print(buzzerIsOn); Serial.print(",");
                                    Serial.print(sn3IsOn); Serial.print(",");

                                    Serial.print(isFalling); Serial.print(",");
                                    Serial.print(altitude); Serial.print(",");
                                    Serial.print(isLaunchMode); Serial.print(",");
                                    Serial.print(forceX_N); Serial.print(",");
                                    Serial.print(jerkX_mps3); Serial.print(",");

                                    Serial.print(gnssIsAvailable); Serial.print(",");
                                    Serial.print(unixEpoch); Serial.print(",");
                                    Serial.print(isFixed); Serial.print(",");
                                    Serial.print(fixType); Serial.print(",");
                                    Serial.print(satelliteCount); Serial.print(",");
                                    Serial.print(latitude, 8); Serial.print(",");
                                    Serial.print(longitude, 8); Serial.print(",");
                                    Serial.print(accuracy); Serial.print(",");

                                    Serial.print(motorTemperature); Serial.print(",");
                                    Serial.print(mcuTemperature); Serial.print(",");
                                    Serial.print(current); Serial.print(",");
                                    Serial.print(inputVoltage); Serial.print(",");
                                    Serial.print(currentPosition); Serial.print(",");
                                    Serial.print(currentDesiredPosition); Serial.print(",");
                                    Serial.print(currentVelocity); Serial.print(", ");
                                    Serial.print(currentSupplyPosition); Serial.print(", ");
                                    Serial.print(voltage); Serial.print(", ");
                                    Serial.print("\n");
                                  });

  while (!Serial)
    ;
  delay(5000);
  printHeader();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  dump(&fram0);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
}

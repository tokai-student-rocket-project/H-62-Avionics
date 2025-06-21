#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>
#include <ArduinoJson.h>

StaticJsonDocument<4096> packet;

uint16_t separation1ProtectionTime = 8605;
uint16_t separation1ForceTime = 11605;
uint16_t separation2ProtectionTime = 1000;
uint16_t separation2ForceTime = 1000;
uint16_t landingTime = 30305;

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  LoRa.begin(925.6E6);
  LoRa.setSignalBandwidth(500E3);

  MsgPacketizer::subscribe(LoRa, 0x0A,

                           [](
                               char ident,
                               uint32_t millis,
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
                               uint32_t unixEpoch,
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
                               int16_t currentDesiredPosition,
                               int16_t currentVelocity,
                               int16_t currentSupplyPosition,
                               int16_t voltage,
                               uint16_t separation1ProtectionTime,
                               uint16_t separation1ForceTime,
                               uint16_t separation2ProtectionTime,
                               uint16_t separation2ForceTime,
                               uint16_t landingTime)
                           {
                             packet.clear();

                             packet["packet"]["module"] = "F";
                             packet["packet"]["rssi_dBm"] = LoRa.packetRssi();
                             packet["packet"]["snr_dBm"] = LoRa.packetSnr();
                             packet["packet"]["ident"] = (String)ident;
                             packet["packet"]["uptime_s"] = (float)millis / 1000.0;

                             packet["logger"]["doLogging"] = doLogging;
                             packet["logger"]["usage"] = loggerUsage;
                             packet["logger"]["number"] = framNumber;

                             packet["flight"]["mode"] = flightMode;
                             packet["flight"]["time_s"] = (float)flightTime / 1000.0;
                             packet["flight"]["detection"]["valveModeIsLaunch"] = isLaunchMode;
                             packet["flight"]["detection"]["flightPinIsOpen"] = flightPinIsOpen;
                             packet["flight"]["detection"]["isFalling"] = isFalling;
                             packet["flight"]["separation"]["sn3IsOn"] = sn3IsOn;
                             packet["flight"]["separation"]["sn4IsOn"] = sn4IsOn;

                             packet["gnss"]["unixEpoch"] = unixEpoch;
                             packet["gnss"]["fixType"] = fixType;
                             packet["gnss"]["satellites"] = satelliteCount;
                             packet["gnss"]["latitude_deg"] = latitude;
                             packet["gnss"]["longitude_deg"] = longitude;
                             packet["gnss"]["height_m"] = (float)height / 10.0;
                             packet["gnss"]["speed_mps"] = (float)speed / 10.0;
                             packet["gnss"]["accuracy_m"] = (float)accuracy / 10.0;

                             packet["valve"]["motorTemperature_degC"] = (float)motorTemperature / 100.0;
                             packet["valve"]["mcuTemperature_degC"] = (float)mcuTemperature / 100.0;
                             packet["valve"]["inputVoltage_V"] = (float)inputVoltage / 1000.0;
                             packet["valve"]["currentPosition_deg"] = (float)currentPosition / 100.0;
                             packet["valve"]["currentDesiredPosition_deg"] = (float)currentDesiredPosition / 100.0;
                             packet["valve"]["currentVelocity_degps"] = (float)currentVelocity / 100.0;
                             packet["valve"]["currentSupplyPosition_deg"] = (float)currentSupplyPosition / 100.0;
                             packet["valve"]["voltage"] = (float)voltage / 100.0;

                             packet["timer"]["separation_1_protection_time"] = (float)separation1ProtectionTime / 1000.0;
                             packet["timer"]["separation_1_force_time"] = (float)separation1ForceTime / 1000.0;
                             packet["timer"]["separation_2_protection_time"] = (float)separation2ProtectionTime / 1000.0;
                             packet["timer"]["separation_2_force_time"] = (float)separation2ForceTime / 1000.0;
                             packet["timer"]["landing_time"] = (float)landingTime / 1000.0;

                             serializeJson(packet, Serial);
                             Serial.println();
                             Serial.flush();

                             digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                           });
}

void loop()
{
  Tasks.update();

  if (Serial.available())
  {
    char command = Serial.read();

    // F: フライトモードオン
    if (command == 'F')
    {
      const auto &packet = MsgPacketizer::encode(0xF1, (uint8_t)0);
      LoRa.beginPacket();
      LoRa.write(packet.data.data(), packet.data.size());
      LoRa.endPacket();
    }

    // R: フライトモードリセット
    if (command == 'R')
    {
      const auto &packet = MsgPacketizer::encode(0xF2, (uint8_t)0);
      LoRa.beginPacket();
      LoRa.write(packet.data.data(), packet.data.size());
      LoRa.endPacket();
    }

    // C: タイマー設定
    if (command == 'C')
    {
      const auto &packet = MsgPacketizer::encode(0xF3, separation1ProtectionTime, separation1ForceTime, separation2ProtectionTime, separation2ForceTime, landingTime);
      LoRa.beginPacket();
      LoRa.write(packet.data.data(), packet.data.size());
      LoRa.endPacket();
    }
  }

  if (LoRa.parsePacket())
  {
    MsgPacketizer::parse();
  }
}

// 点火点で運用する MKR WAN 1310 用

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>
#include <ArduinoJson.h>
#include "LoRaBoards.h"

StaticJsonDocument<4096> packet;

// 分離保護・強制分離時間設定
uint32_t separation1ProtectionTime = 22390; // (- 1.0 s)
uint32_t separation1ForceTime = 25390;      // (+ 2.0 s)
uint32_t separation2ProtectionTime = 84290; // (- 1.0 s)
uint32_t separation2ForceTime = 85290;      // (± 0 s)
uint32_t landingTime = 88890;

void setup()
{
    setupBoards();
    Serial.begin(115200);
    Serial.println("loraRssi,loraSnr,millis,flightMode,flightTime,loggerUsage,doLogging,framNumber,flightPinIsOpen,sn3IsOn,sn4IsOn,isLaunchMode,isFalling,unixEpoch,fixType,satelliteCount,latitude,longitude,height,speed,accuracy,motorTemperature,mcuTemperature,inputVoltage,current,currentPosition,currentDesiredPosition,currentVelocity,motorTemperature_SUPPLY,mcuTemperature_SUPPLY,inputVoltage_SUPPLY,current_SUPPLY,currentPosition_SUPPLY,currentDesiredPosition_SUPPLY,currentVelocity_SUPPLY,separation1ProtectionTime,separation1ForceTime,separation2ProtectionTime,separation2ForceTime,landingTime");

    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    LoRa.begin(925.6E6);
    LoRa.setSignalBandwidth(500E3);

    MsgPacketizer::subscribe(LoRa, 0x0A,

                             [](
                                 char ident,
                                 uint32_t millis,
                                 uint8_t flightMode,
                                 uint32_t flightTime,
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
                                 int16_t current,
                                 int16_t currentPosition,
                                 int16_t currentDesiredPosition,
                                 int16_t currentVelocity,

                                 int16_t motorTemperature_SUPPLY,
                                 int16_t mcuTemperature_SUPPLY,
                                 int16_t inputVoltage_SUPPLY,
                                 int16_t current_SUPPLY,
                                 int16_t currentPosition_SUPPLY,
                                 int16_t currentDesiredPosition_SUPPLY,
                                 int16_t currentVelocity_SUPPLY,
                                 uint32_t separation1ProtectionTime,
                                 uint32_t separation1ForceTime,
                                 uint32_t separation2ProtectionTime,
                                 uint32_t separation2ForceTime,
                                 uint32_t landingTime)
                             {
                                 Serial.print(millis / 1000.0);
                                 Serial.print(",");
                                 Serial.print(ident);
                                 Serial.print(",");
                                 Serial.print(loggerUsage);
                                 Serial.print(",");
                                 Serial.print(doLogging);
                                 Serial.print(",");
                                 Serial.print(framNumber);
                                 Serial.print(",");
                                 Serial.print(flightMode);
                                 Serial.print(",");
                                 Serial.print(flightTime / 1000.0);
                                 Serial.print(",");
                                 float loraRssi = LoRa.packetRssi();
                                 Serial.print(loraRssi);
                                 Serial.print(",");
                                 float loraSnr = LoRa.packetSnr();
                                 Serial.print(loraSnr);
                                 Serial.print(",");
                                 Serial.print(flightPinIsOpen);
                                 Serial.print(",");
                                 Serial.print(sn3IsOn);
                                 Serial.print(",");
                                 Serial.print(sn4IsOn);
                                 Serial.print(",");
                                 Serial.print(isLaunchMode);
                                 Serial.print(",");
                                 Serial.print(isFalling);
                                 Serial.print(",");
                                 Serial.print(unixEpoch);
                                 Serial.print(",");
                                 Serial.print(fixType);
                                 Serial.print(",");
                                 Serial.print(satelliteCount);
                                 Serial.print(",");
                                 Serial.print(latitude);
                                 Serial.print(",");
                                 Serial.print(longitude);
                                 Serial.print(",");
                                 Serial.print(height);
                                 Serial.print(",");
                                 Serial.print(speed);
                                 Serial.print(",");
                                 Serial.print(accuracy);
                                 Serial.print(",");

                                 Serial.print((float)motorTemperature / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)mcuTemperature / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)inputVoltage / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)current / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)currentPosition / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)currentDesiredPosition / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)currentVelocity / 100.0);
                                 Serial.print(",");

                                 Serial.print((float)motorTemperature_SUPPLY / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)mcuTemperature_SUPPLY / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)inputVoltage_SUPPLY / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)current_SUPPLY / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)currentPosition_SUPPLY / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)currentDesiredPosition_SUPPLY / 100.0);
                                 Serial.print(",");
                                 Serial.print((float)currentVelocity_SUPPLY / 100.0);
                                 Serial.print(",");

                                 Serial.print(separation1ProtectionTime);
                                 Serial.print(",");
                                 Serial.print(separation1ForceTime);
                                 Serial.print(",");
                                 Serial.print(separation2ProtectionTime);
                                 Serial.print(",");
                                 Serial.print(separation2ForceTime);
                                 Serial.print(",");
                                 Serial.println(landingTime);
                                 Serial.flush();
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

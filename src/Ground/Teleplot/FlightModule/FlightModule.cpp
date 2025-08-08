#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>
#include <ArduinoJson.h>

StaticJsonDocument<4096> packet;

uint32_t separation1ProtectionTime = 22390; // (- 1.0 s)
uint32_t separation1ForceTime = 25390;      // (+ 2.0 s)
uint32_t separation2ProtectionTime = 84290; // (- 1.0 s)
uint32_t separation2ForceTime = 85290;      // (± 0 s)
uint32_t landingTime = 88890;

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
                                 Serial.print(">LoRa_RSSI_dBm: ");
                                 float loraRssi = LoRa.packetRssi();
                                 Serial.println(loraRssi);

                                 float loraSnr = LoRa.packetSnr();
                                 Serial.print(">LoRa_SNR_dBm:");
                                 Serial.println(loraSnr);

                                 Serial.print(">ident:");
                                 Serial.println(ident);
                                 Serial.print(">upTime_s:");
                                 Serial.println((float)millis / 1000.0);
                                 Serial.print(">flightMode:");
                                 Serial.println(flightMode);
                                 Serial.print(">flightTime_s:");
                                 Serial.println((float)flightTime / 1000.0);
                                 Serial.print(">loggerUsage_percent:");
                                 Serial.println(loggerUsage);
                                 Serial.print(">doLogging_bool:");
                                 Serial.println(doLogging);
                                 Serial.print(">framNumber:");
                                 Serial.println(framNumber);
                                 Serial.print(">flightPinIsOpen_bool:");
                                 Serial.println(flightPinIsOpen);
                                 Serial.print(">sn3IsOn_bool:");
                                 Serial.println(sn3IsOn);
                                 Serial.print(">sn4IsOn_bool:");
                                 Serial.println(sn4IsOn);
                                 Serial.print(">isLaunchMode_bool:");
                                 Serial.println(isLaunchMode);
                                 Serial.print(">isFalling_bool:");
                                 Serial.println(isFalling);

                                 Serial.print(">unixEpoch_s:");
                                 Serial.println(unixEpoch);
                                 Serial.print(">fixType:");
                                 Serial.println(fixType);
                                 Serial.print(">satelliteCount:");
                                 Serial.println(satelliteCount);
                                 Serial.print(">latitude_deg:");
                                 Serial.println(latitude);
                                 Serial.print(">longitude_deg:");
                                 Serial.println(longitude);
                                 Serial.print(">height_m:");
                                 Serial.println(height);
                                 Serial.print(">speed_mps:");
                                 Serial.println((float)speed / 1000.0);
                                 Serial.print(">accuracy_m:");
                                 Serial.println((float)accuracy / 100.0);

                                 Serial.print(">MAIN_motorTemperature_C:");
                                 Serial.println((float)motorTemperature / 100.0);
                                 Serial.print(">MAIN_mcuTemperature_C:");
                                 Serial.println((float)mcuTemperature / 100.0);
                                 Serial.print(">MAIN_inputVoltage_V:");
                                 Serial.println((float)inputVoltage / 100.0);
                                 Serial.print(">MAIN_current_A:");
                                 Serial.println((float)current / 100.0);
                                 Serial.print(">MAIN_valvePosition_deg:");
                                 Serial.println((float)currentPosition / 100.0);
                                 Serial.print(">MAIN_desiredPosition_deg:");
                                 Serial.println((float)currentDesiredPosition / 100.0);
                                 Serial.print(">MAIN_velocity_dps:");
                                 Serial.println((float)currentVelocity / 100.0);

                                 Serial.print(">SUPPLY_motorTemperature_C:");
                                 Serial.println((float)motorTemperature_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_mcuTemperature_C:");
                                 Serial.println((float)mcuTemperature_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_inputVoltage_V:");
                                 Serial.println((float)inputVoltage_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_current_A:");
                                 Serial.println((float)current_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_valvePosition_deg:");
                                 Serial.println((float)currentPosition_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_desiredPosition_deg:");
                                 Serial.println((float)currentDesiredPosition_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_velocity_dps:");
                                 Serial.println((float)currentVelocity_SUPPLY / 100.0);

                                 Serial.print(">separation1ProtectionTime_ms:");
                                 Serial.println(separation1ProtectionTime);
                                 Serial.print(">separation1ForceTime_ms:");
                                 Serial.println(separation1ForceTime);
                                 Serial.print(">separation2ProtectionTime_ms:");
                                 Serial.println(separation2ProtectionTime);
                                 Serial.print(">separation2ForceTime_ms:");
                                 Serial.println(separation2ForceTime);
                                 Serial.print(">landingTime_ms:");
                                 Serial.println(landingTime);

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

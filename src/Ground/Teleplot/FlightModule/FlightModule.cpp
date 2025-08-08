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
                                 uint16_t separation1ProtectionTime,
                                 uint16_t separation1ForceTime,
                                 uint16_t separation2ProtectionTime,
                                 uint16_t separation2ForceTime,
                                 uint16_t landingTime)
                             {
                                 Serial.print(">LoRa_RSSI_dBm: ");
                                 float loraRssi = LoRa.packetRssi();
                                 Serial.println(loraRssi);

                                 float loraSnr = LoRa.packetSnr();
                                 Serial.print(">LoRa_SNR_dBm: ");
                                 Serial.println(loraSnr);

                                 Serial.print(">upTime_sec: ");
                                 Serial.println(millis / 1000);
                                 Serial.print(">flightTime: ");
                                 Serial.println(flightTime / 1000);
                                 Serial.print(">doLogging_bool: ");
                                 Serial.println(doLogging);
                                 Serial.print(">loggerUsage_%: ");
                                 Serial.println(loggerUsage);
                                 Serial.print(">framNumber: ");
                                 Serial.println(framNumber);

                                 Serial.print(">sn3IsOn: ");
                                 Serial.println(sn3IsOn);
                                 Serial.print(">sn4IsOn: ");
                                 Serial.println(sn4IsOn);
                                 Serial.print(">isFalling: ");
                                 Serial.println(isFalling);

                                 Serial.print(">FLIGHTMODE: ");
                                 Serial.println(flightMode);
                                 Serial.print(">flightTime_s");
                                 Serial.println((float)flightTime / 1000.0);
                                 Serial.print(">flightPinIsOpen: ");
                                 Serial.println(flightPinIsOpen);

                                 Serial.print(">latitude: ");
                                 Serial.println(latitude);
                                 Serial.print(">longitude: ");
                                 Serial.println(longitude);

                                 Serial.print(">MAIN_motorTemperature_*C: ");
                                 Serial.println((float)motorTemperature / 100.0);
                                 Serial.print(">MAIN_mcuTemperature_*C: ");
                                 Serial.println((float)mcuTemperature / 100.0);
                                 Serial.print(">MAIN_inputVoltageMain_V");
                                 Serial.println((float)inputVoltage / 100.0);
                                 Serial.print("MAIN_current_A: ");
                                 Serial.println((float)current / 100.0);
                                 Serial.print(">MAIN_ValvePosition_deg: ");
                                 Serial.println((float)currentPosition / 100.0);
                                 Serial.print(">MAIN_desiredPosition_deg: ");
                                 Serial.println((float)currentDesiredPosition / 100.0);
                                 Serial.print(">MAIN_velocity_m/s^2: ");
                                 Serial.println((float)currentVelocity / 100.0);

                                 Serial.print(">SUPPLY_morotTemperature_*C: ");
                                 Serial.println((float)mcuTemperature_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_mcuTemperature_*C: ");
                                 Serial.println((float)mcuTemperature_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_inputVoltage_V");
                                 Serial.println((float)inputVoltage_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_current_A");
                                 Serial.println((float)current_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_ValvePosition_deg: ");
                                 Serial.println((float)currentPosition_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_desiredPosition_deg: ");
                                 Serial.println((float)currentDesiredPosition_SUPPLY / 100.0);
                                 Serial.print(">SUPPLY_velocity_m/s^2");
                                 Serial.println((float)currentVelocity_SUPPLY / 100.0);

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

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>
#include "LoRaBoards.h"
#include <TinyGPS++.h>

float onbordLatitude;
float onbordLongtitude;
TinyGPSPlus onbordGps;

void task5Hz()
{

    while (SerialGPS.available())
    {
        onbordGps.encode(SerialGPS.read());
    }

    if (onbordGps.location.isValid())
    {
        Serial.print(F("Lat: "));
        Serial.print(onbordGps.location.lat(), 6);
        Serial.print(F(", Lng: "));
        Serial.print(onbordGps.location.lng(), 6);
        Serial.print(F(", Alt: "));
        Serial.print(onbordGps.altitude.meters(), 2);
        Serial.print(F("m"));

        if (onbordGps.date.isValid() && onbordGps.time.isValid())
        {
            Serial.print(F(", Date: "));
            Serial.print(onbordGps.date.year());
            Serial.print(F("-"));
            Serial.print(onbordGps.date.month(), DEC);
            Serial.print(F("-"));
            Serial.print(onbordGps.date.day(), DEC);
            Serial.print(F(", Time: "));
            Serial.print(onbordGps.time.hour(), DEC);
            Serial.print(F(":"));
            Serial.print(onbordGps.time.minute(), DEC);
            Serial.print(F(":"));
            Serial.print(onbordGps.time.second(), DEC);
        }

        Serial.print(F(", Sats: "));
        Serial.print(onbordGps.satellites.value());
        Serial.print(F(", HDOP: "));
        Serial.print(onbordGps.hdop.hdop(), 1);

        Serial.println();
    }
    else
    {
        // GPSデータがまだ有効でない場合
        Serial.println(F("No valid GPS data yet."));
    }
}

void setup()
{
    setupBoards();
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    LoRa.begin(924.2E6);
    LoRa.setSignalBandwidth(500E3);

    MsgPacketizer::subscribe(LoRa, 0x0A,

                             [](
                                 uint32_t millis,
                                 char ident,
                                 uint8_t loggerUsage,
                                 bool doLogging,
                                 uint8_t framNumber,
                                 int16_t accelerationX_mps2,
                                 int16_t accelerationY_mps2,
                                 int16_t accelerationZ_mps2,
                                 int16_t accelerationNorm_mps2,
                                 int16_t roll_deg,
                                 int16_t pitch_deg,
                                 int16_t yaw_deg,
                                 int16_t forceX_N,
                                 int16_t jerkX_mps3,
                                 int16_t altitude_m,
                                 int16_t verticalSpeed_mps,
                                 int16_t estimated,
                                 int16_t apogee,
                                 int16_t externalVoltage_V,
                                 int16_t batteryVoltage_V,
                                 int16_t busVoltage_V,
                                 int16_t externalCurrent_mA,
                                 int16_t batteryCurrent_mA,
                                 int16_t busCurrent_mA,
                                 int8_t externalPower_mW,
                                 int8_t batteryPower_mW,
                                 int8_t busPower_mW,
                                 int16_t externalDieTemperature_C,
                                 int16_t batteryDieTemperature_C,
                                 int16_t busDieTemperature_C)
                             {
                                 Serial.print(">LoRa_RSSI_dBm: ");
                                 float loraRssi = LoRa.packetRssi();
                                 Serial.println(loraRssi);

                                 float loraSnr = LoRa.packetSnr();
                                 Serial.print(">LoRa_SNR_dBm: ");
                                 Serial.println(loraSnr);

                                 Serial.print(">upTime_sec: ");
                                 Serial.println((float)millis / 1000);
                                 Serial.print(">doLogging_bool: ");
                                 Serial.println(doLogging);
                                 Serial.print(">loggerUsage_%: ");
                                 Serial.println(loggerUsage);
                                 Serial.print(">framNumber: ");
                                 Serial.println(framNumber);

                                 Serial.print(">acceleration_mps2_norm: ");
                                 Serial.println((float)accelerationNorm_mps2 / 10.0);
                                 Serial.print(">acceleration_mps2_x: ");
                                 Serial.println((float)accelerationX_mps2 / 10.0);
                                 Serial.print(">acceleration_mps2_y: ");
                                 Serial.println((float)accelerationY_mps2 / 10.0);
                                 Serial.print(">acceleration_mps2_z: ");
                                 Serial.println((float)accelerationZ_mps2 / 10.0);
                                 Serial.print(">orientation_deg_roll: ");
                                 Serial.println((float)roll_deg / 10.0);
                                 Serial.print(">orientation_deg_pitch: ");
                                 Serial.println((float)pitch_deg / 10.0);
                                 Serial.print(">orientation_deg_yaw: ");
                                 Serial.println((float)yaw_deg / 10.0);
                                 Serial.print(">forceX_N: ");
                                 Serial.println((float)forceX_N / 10.0);
                                 Serial.print(">jerkX_mps3: ");
                                 Serial.println((float)jerkX_mps3 / 10.0);

                                 Serial.print(">altitude_m: ");
                                 Serial.println((float)altitude_m / 10.0);
                                 Serial.print(">vertiaclSpeed_mps: ");
                                 Serial.println((float)verticalSpeed_mps / 10.0);
                                 Serial.print(">apogee_m: ");
                                 Serial.println((float)apogee / 10.0);
                                 Serial.print(">estimated_s: ");
                                 Serial.println((float)estimated / 10.0);

                                 Serial.print(">externalVoltage_V: ");
                                 Serial.println((float)externalVoltage_V / 100.0);
                                 Serial.print(">batteryVoltage_V: ");
                                 Serial.println((float)batteryVoltage_V / 100.0);
                                 Serial.print(">busVoltage_V: ");
                                 Serial.println((float)busVoltage_V / 100.0);
                                 Serial.print(">externalCurrent_mA: ");
                                 Serial.println((float)externalCurrent_mA / 100.0);
                                 Serial.print(">batteryCurrent_mA: ");
                                 Serial.println((float)batteryCurrent_mA / 100.0);
                                 Serial.print(">busCurrent_mA: ");
                                 Serial.println((float)busCurrent_mA / 100.0);
                                 Serial.print(">externalPower_mW: ");
                                 Serial.println((float)externalPower_mW / 100.0);
                                 Serial.print(">batteryPower_mW");
                                 Serial.println((float)batteryPower_mW / 100.0);
                                 Serial.print(">busPower_mW: ");
                                 Serial.println((float)busPower_mW / 10.0);

                                 Serial.print(">groundTemperature_degC: ");
                                 Serial.println((float)externalDieTemperature_C / 10.0);
                                 Serial.print(">batteryDieTemperature_degC: ");
                                 Serial.println((float)batteryDieTemperature_C / 10.0);
                                 Serial.print(">busDieTemperature_degC: ");
                                 Serial.println((float)busDieTemperature_C / 10.0);

                                 Serial.println();
                                 Serial.flush();
                             });

    Tasks.add(&task5Hz)->startFps(5);
}

void loop()
{
    Tasks.update();

    if (LoRa.parsePacket())
    {
        MsgPacketizer::parse();
    }
}

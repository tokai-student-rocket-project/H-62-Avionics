#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>
#include <ArduinoJson.h>


StaticJsonDocument<4096> packet;


void setup(){
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

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
                               uint8_t groundVoltage_V,
                               uint8_t batteryVoltage_V,
                               uint8_t tieVoltage_V,
                               uint8_t busVoltage_V,
                               int16_t groundCurrent_mA,
                               int16_t batteryCurrent_mA,
                               int16_t tieCurrent_mA,
                               int16_t busCurrent_mA,
                               int8_t groundPower_mW,
                               int8_t batteryPower_mW,
                               int8_t tiePower_mW,
                               int8_t busPower_mW,
                               int16_t temperatureRegulator1_degC,
                               int16_t temperatureRegulator2_degC,
                               int16_t temperatureRegulator3_degC,
                               int16_t temperatureConduction_degC,
                               int16_t temperatureOutside_degC,
                               int16_t temperatureInside_degC,
                               int16_t temperatureVentPort_degC,
                               int16_t temperatureTankAtmosphere_degC,
                               uint32_t sutegomaTime_ms,
                               int16_t sutegomeTaskRate_Hz)
                           {
                             packet.clear();

                             packet["packet"]["module"] = "S";
                             packet["packet"]["rssi_dBm"] = LoRa.packetRssi();
                             packet["packet"]["snr_dBm"] = LoRa.packetSnr();
                             packet["packet"]["ident"] = (String)ident;
                             packet["packet"]["uptime_s"] = (float)millis / 1000.0;

                             packet["logger"]["doLogging"] = doLogging;
                             packet["logger"]["usage"] = loggerUsage;
                             packet["logger"]["number"] = framNumber;

                             packet["dynamics"]["acceleration_mps2"]["norm"] = (float)accelerationNorm_mps2 / 10.0;
                             packet["dynamics"]["acceleration_mps2"]["x"] = (float)accelerationX_mps2 / 10.0;
                             packet["dynamics"]["acceleration_mps2"]["y"] = (float)accelerationY_mps2 / 10.0;
                             packet["dynamics"]["acceleration_mps2"]["z"] = (float)accelerationZ_mps2 / 10.0;
                             packet["dynamics"]["orientation_deg"]["roll"] = (float)roll_deg / 10.0;
                             packet["dynamics"]["orientation_deg"]["pitch"] = (float)pitch_deg / 10.0;
                             packet["dynamics"]["orientation_deg"]["yaw"] = (float)yaw_deg / 10.0;
                             packet["dynamics"]["forceX_N"] = (float)forceX_N / 10.0;
                             packet["dynamics"]["jerkX_mps3"] = (float)jerkX_mps3 / 10.0;

                             packet["trajectory"]["altitude_m"] = (float)altitude_m / 10.0;
                             packet["trajectory"]["verticalSpeed_mps"] = (float)verticalSpeed_mps / 10.0;
                             packet["trajectory"]["apogee_m"] = (float)apogee / 10.0;
                             packet["trajectory"]["estimated_s"] = (float)estimated / 10.0;

                             packet["electrical"]["voltage_V"]["ground"] = (float)groundVoltage_V / 10.0;
                             packet["electrical"]["voltage_V"]["battery"] = (float)batteryVoltage_V / 10.0;
                             packet["electrical"]["voltage_V"]["tie"] = (float)tieVoltage_V / 10.0;
                             packet["electrical"]["voltage_V"]["bus"] = (float)busVoltage_V / 10.0;
                             packet["electrical"]["current_mA"]["ground"] = (float)groundCurrent_mA / 10.0;
                             packet["electrical"]["current_mA"]["battery"] = (float)batteryCurrent_mA / 10.0;
                             packet["electrical"]["current_mA"]["tie"] = (float)tieCurrent_mA / 10.0;
                             packet["electrical"]["current_mA"]["bus"] = (float)busCurrent_mA / 10.0;
                             packet["electrical"]["power_W"]["ground"] = (float)groundPower_mW / 10.0;
                             packet["electrical"]["power_W"]["battery"] = (float)batteryPower_mW / 10.0;
                             packet["electrical"]["power_W"]["tie"] = (float)tiePower_mW / 10.0;
                             packet["electrical"]["power_W"]["bus"] = (float)busPower_mW / 10.0;

                             packet["thermal"]["temperature_degC"]["regulator1"] = (float)temperatureRegulator1_degC / 10.0;
                             packet["thermal"]["temperature_degC"]["regulator2"] = (float)temperatureRegulator2_degC / 10.0;
                             packet["thermal"]["temperature_degC"]["regulator3"] = (float)temperatureRegulator3_degC / 10.0;
                             packet["thermal"]["temperature_degC"]["conduction"] = (float)temperatureConduction_degC / 10.0;
                             packet["thermal"]["temperature_degC"]["outside"] = (float)temperatureOutside_degC / 10.0;
                             packet["thermal"]["temperature_degC"]["inside"] = (float)temperatureInside_degC / 10.0;
                             packet["thermal"]["temperature_degC"]["ventPort"] = (float)temperatureVentPort_degC / 10.0;
                             packet["thermal"]["temperature_degC"]["tankAtmosphere"] = (float)temperatureTankAtmosphere_degC / 10.0;

                             packet["sutegoma"]["uptime_s"] = (float)sutegomaTime_ms / 1000.0;
                             packet["sutegoma"]["taskRate_Hz"] = (float)sutegomeTaskRate_Hz / 10.0;

                             serializeJson(packet, Serial);
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

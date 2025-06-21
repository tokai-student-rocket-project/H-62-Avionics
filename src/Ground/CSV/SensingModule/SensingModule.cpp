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

  LoRa.begin(923.8E6);
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

                             Serial.print((float)accelerationNorm_mps2 / 10.0);
                             Serial.print(",");
                             Serial.print((float)accelerationX_mps2 / 10.0);
                             Serial.print(",");
                             Serial.print((float)accelerationY_mps2 / 10.0);
                             Serial.print(",");
                             Serial.print((float)accelerationZ_mps2 / 10.0);
                             Serial.print(",");
                             Serial.print((float)roll_deg / 10.0);
                             Serial.print(",");
                             Serial.print((float)pitch_deg / 10.0);
                             Serial.print(",");
                             Serial.print((float)yaw_deg / 10.0);
                             Serial.print(",");
                             Serial.print((float)forceX_N / 10.0);
                             Serial.print(",");
                             Serial.print((float)jerkX_mps3 / 10.0);
                             Serial.print(",");

                             Serial.print((float)altitude_m / 10.0);
                             Serial.print(",");
                             Serial.print((float)verticalSpeed_mps / 10.0);
                             Serial.print(",");
                             Serial.print((float)apogee / 10.0);
                             Serial.print(",");
                             Serial.print((float)estimated / 10.0);
                             Serial.print(",");

                             Serial.print((float)groundVoltage_V / 10.0);
                             Serial.print(",");
                             Serial.print((float)batteryVoltage_V / 10.0);
                             Serial.print(",");
                             Serial.print((float)tieVoltage_V / 10.0);
                             Serial.print(",");
                             Serial.print((float)busVoltage_V / 10.0);
                             Serial.print(",");
                             Serial.print((float)groundCurrent_mA / 10.0);
                             Serial.print(",");
                             Serial.print((float)batteryCurrent_mA / 10.0);
                             Serial.print(",");
                             Serial.print((float)tieCurrent_mA / 10.0);
                             Serial.print(",");
                             Serial.print((float)busCurrent_mA / 10.0);
                             Serial.print(",");
                             Serial.print((float)groundPower_mW / 10.0);
                             Serial.print(",");
                             Serial.print((float)batteryPower_mW / 10.0);
                             Serial.print(",");
                             Serial.print((float)tiePower_mW / 10.0);
                             Serial.print(",");
                             Serial.print((float)busPower_mW / 10.0);
                             Serial.print(",");

                             Serial.print((float)temperatureRegulator1_degC / 10.0);
                             Serial.print(",");
                             Serial.print((float)temperatureRegulator2_degC / 10.0);
                             Serial.print(",");
                             Serial.print((float)temperatureRegulator3_degC / 10.0);
                             Serial.print(",");
                             Serial.print((float)temperatureConduction_degC / 10.0);
                             Serial.print(",");
                             Serial.print((float)temperatureOutside_degC / 10.0);
                             Serial.print(",");
                             Serial.print((float)temperatureInside_degC / 10.0);
                             Serial.print(",");
                             Serial.print((float)temperatureVentPort_degC / 10.0);
                             Serial.print(",");
                             Serial.print((float)temperatureTankAtmosphere_degC / 10.0);
                             Serial.print(",");

                             Serial.print((float)sutegomaTime_ms / 1000.0);
                             Serial.print(",");
                             Serial.print((float)sutegomeTaskRate_Hz / 10.0);

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

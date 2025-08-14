#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TaskManager.h>
#include <MsgPacketizer.h>
#include <movingAvg.h>
#include <Calculus.h>
#include <Filters.h>
#include "Lib_Var.hpp"
#include "Lib_CAN.hpp"
#include "Lib_Telemeter.hpp"
#include "Lib_BNO055.hpp"
#include "Lib_Logger3.hpp"
#include "Lib_Altimeter.hpp"
#include "Lib_Thermistor.hpp"
#include "Lib_RateMonitor.hpp"
#include "Lib_OutputPin.hpp"

char ident = '\0';
bool doLogging = false;

CAN can(7);
Telemeter telemeter;
Logger logger(14, A2, 13);

OutputPin ledWork(LED_BUILTIN);
OutputPin ledCanRx(0);
OutputPin ledCanTx(1);
OutputPin ledLoRaRx(4);
OutputPin ledLoRaTx(5);

uint8_t flightMode;
uint32_t flightTime;
RateMonitor monitor10Hz;
RateMonitor monitor100Hz;

Altimeter primary;
Altimeter secondary;
BNO055 bno055;
Thermistor regulator1(A3);
Thermistor regulator2(A4);
Thermistor regulator3(A5);
Thermistor conduction(A6);
Thermistor outside(A0);

float accelerationX_mps2, accelerationY_mps2, accelerationZ_mps2;
float gyroscopeX_dps, gyroscopeY_dps, gyroscopeZ_dps;
float roll_deg, pitch_deg, yaw_deg;
float magnetometerX_nT, magnetometerY_nT, magnetometerZ_nT;

float externalVoltage_V, batteryVoltage_V, busVoltage_V;
float externalCurrent_mA, batteryCurrent_mA, busCurrent_mA;
float externalPower_W, batteryPower_W, busPower_W;
float externalDieTemperature_C, batteryDieTemperature_C, busDieTemperature_C;

float referencePressure_hPa;
float primaryPressure_hPa, secondaryPressure_hPa;
float primaryAltitude_m, secondaryAltitude_m, altitude_m;
float primaryTemperature_C, secondaryTemperature_C;
movingAvg altitudeAverage(10);
Calculus::Differential<float> altitudeGradient(2); // 初期値 5
Calculus::Differential<float> verticalSpeedGradient(0.5);
float verticalSpeed_mps;
float verticalAcceleration_msp2;
float estimated;
float apogee;
bool isFalling;

Calculus::Integral<float> gyroscopeIntegralX;
Calculus::Integral<float> gyroscopeIntegralY;
Calculus::Integral<float> gyroscopeIntegralZ;

Filter::HPF<float> accelerationHighPassX(5);
Filter::HPF<float> accelerationHighPassY(5);
Filter::HPF<float> accelerationHighPassZ(5);

constexpr float gravity_mps2 = 9.80665;
constexpr float initialPitch_deg = 90.0;
constexpr float characteristicMass_kg = 12.41;
float gravityX_mps2, gravityY_mps2, gravityZ_mps2;
float linearAccelerationX_mps2, linearAccelerationY_mps2, linearAccelerationZ_mps2;
float forceX_N;
Calculus::Differential<float> linearAccelerationGradient(0.25);
float jerkX_mps3;

// 気象庁，毎日の全国データ一覧　https://www.data.jma.go.jp/stats/data/mdrr/synopday/index.html
void calibrationPressure()
{
    float referencePrimaryPressure_hPa = 0;
    float referenceSecondaryPressure_hPa = 0;
    const int calibrationSamples = 100;
    for (int i = 0; i < calibrationSamples; i++)
    {
        referencePrimaryPressure_hPa += primary.getPressure();
        referenceSecondaryPressure_hPa += secondary.getPressure();
    }
    primary.setReferencePressure(referencePrimaryPressure_hPa / calibrationSamples);
    secondary.setReferencePressure(referenceSecondaryPressure_hPa / calibrationSamples);

    // Serial.print(">referencePrimaryPressure_hPa: ");
    // Serial.println(primary.getReferencePressure());

    // Serial.print(">referenceSecondaryPressure_hPa: ");
    // Serial.println(secondary.getReferencePressure());

    doLogging = false;
}

void task200Hz()
{
    primaryTemperature_C = primary.getTemperature();
    secondaryTemperature_C = secondary.getTemperature();

    primaryPressure_hPa = primary.getPressure();
    secondaryPressure_hPa = secondary.getPressure();

    float temperature_C = (primaryTemperature_C + secondaryTemperature_C) / 2;
    altitude_m = (primary.getAltitude(temperature_C) + secondary.getAltitude(temperature_C)) / 2;

    Serial.print(">temperature: ");
    Serial.println(temperature_C);

    Serial.print(">altitude_m: ");
    Serial.println(altitude_m);

    // Serial.print(">primaryAltitude_m: ");
    // Serial.println(primary.getAltitude(temperature_C));
}

void task100Hz()
{
    ledWork.toggle();

    float deltaTime = 1.0 / monitor100Hz.updateRate();

    bno055.getAcceleration(&accelerationX_mps2, &accelerationY_mps2, &accelerationZ_mps2);
    bno055.getGyroscope(&gyroscopeX_dps, &gyroscopeY_dps, &gyroscopeZ_dps);

    roll_deg = gyroscopeIntegralX.get(gyroscopeX_dps, deltaTime);
    pitch_deg = gyroscopeIntegralY.get(gyroscopeY_dps, deltaTime) + initialPitch_deg;
    yaw_deg = gyroscopeIntegralZ.get(gyroscopeZ_dps, deltaTime);

    gravityX_mps2 = gravity_mps2 * sin(radians(pitch_deg)) * cos(radians(yaw_deg));
    gravityY_mps2 = gravity_mps2 * sin(radians(yaw_deg)) * cos(radians(roll_deg));
    gravityZ_mps2 = gravity_mps2 * cos(radians(pitch_deg)) * cos(radians(roll_deg));

    /*
    // task10Hzから移植
    verticalSpeed_mps = altitudeGradient.get(altitude_m, deltaTime);
    verticalAcceleration_msp2 = verticalSpeedGradient.get(verticalSpeed_mps, deltaTime);

    estimated = -verticalSpeed_mps / verticalAcceleration_msp2;
    apogee = altitude_m + (verticalSpeed_mps * estimated + 0.5 * verticalAcceleration_msp2 * estimated * estimated);
    isFalling = verticalSpeed_mps < 0;

    Serial.print(">isFalling: ");
    Serial.println(isFalling);

    Serial.print(">verticalSpeed_mps: ");
    Serial.println(verticalSpeed_mps);
    ////
    */

    // Serial.print(">gravityX_mps2:");
    // Serial.println(gravityX_mps2);
    // Serial.print(">gravityY_mps2:");
    // Serial.println(gravityY_mps2);
    // Serial.print(">gravityZ_mps2:");
    // Serial.println(gravityZ_mps2);
    // Serial.print(">roll_deg:");
    // Serial.println(roll_deg);
    // Serial.print(">pitch_deg:");
    // Serial.println(pitch_deg);
    // Serial.print(">yaw_deg:");
    // Serial.println(yaw_deg);

    // Serial.print(">accelerationX_mps2: ");
    // Serial.println(accelerationX_mps2);
    // Serial.print(">accelerationY_mps2: ");
    // Serial.println(accelerationY_mps2);
    // Serial.print(">accelerationZ_mps2: ");
    // Serial.println(accelerationZ_mps2);

    linearAccelerationX_mps2 = accelerationHighPassX.get(accelerationX_mps2 - gravityX_mps2, deltaTime);
    linearAccelerationY_mps2 = accelerationHighPassY.get(accelerationY_mps2 - gravityY_mps2, deltaTime);
    linearAccelerationZ_mps2 = accelerationHighPassZ.get(accelerationZ_mps2 - gravityZ_mps2, deltaTime);

    const auto &logPacket = MsgPacketizer::encode(0x0A,
                                                  ident, millis(), flightTime, flightMode, logger.getUsage(),
                                                  accelerationX_mps2, accelerationY_mps2, accelerationZ_mps2,
                                                  gyroscopeX_dps, gyroscopeY_dps, gyroscopeZ_dps,
                                                  magnetometerX_nT, magnetometerY_nT, magnetometerZ_nT,
                                                  roll_deg, pitch_deg, yaw_deg,
                                                  forceX_N, jerkX_mps3,
                                                  primaryPressure_hPa, secondaryPressure_hPa, referencePressure_hPa, altitude_m,
                                                  verticalSpeed_mps, verticalAcceleration_msp2, estimated, apogee, isFalling,
                                                  externalVoltage_V, batteryVoltage_V, busVoltage_V,
                                                  externalCurrent_mA, batteryCurrent_mA, busCurrent_mA,
                                                  externalPower_W, batteryPower_W, busPower_W,
                                                  externalDieTemperature_C, batteryDieTemperature_C, busDieTemperature_C);

    if (doLogging)
    {
        logger.write(logPacket.data.data(), logPacket.data.size());
    }
}

void task50Hz()
{
    // 現状特になし
}

void task20Hz()
{
    ledWork.toggle();

    bno055.getMagnetometer(&magnetometerX_nT, &magnetometerY_nT, &magnetometerZ_nT);
}

void task10Hz()
{
    ledWork.toggle();

    float deltaTime = 1.0 / monitor10Hz.updateRate();

    forceX_N = characteristicMass_kg * linearAccelerationX_mps2;
    jerkX_mps3 = linearAccelerationGradient.get(linearAccelerationX_mps2, deltaTime);

    can.sendDynamics(forceX_N, jerkX_mps3);

    // verticalSpeed_mps = altitudeGradient.get((float)altitudeAverage.reading((int16_t)(altitude_m * 10)) / 10.0, deltaTime); // LPS28DFW内で平均化処理を行ってもらう．

    verticalSpeed_mps = altitudeGradient.get(altitude_m, deltaTime);
    verticalAcceleration_msp2 = verticalSpeedGradient.get(verticalSpeed_mps, deltaTime);

    estimated = -verticalSpeed_mps / verticalAcceleration_msp2;
    apogee = altitude_m + (verticalSpeed_mps * estimated + 0.5 * verticalAcceleration_msp2 * estimated * estimated);
    isFalling = verticalSpeed_mps < 0;

    can.sendTrajectory(isFalling, altitude_m);

    ledCanTx.toggle();

    /*x
    Serial.print(">isFalling: ");
    Serial.println(isFalling);

    Serial.print(">verticalSpeed_mps: ");
    Serial.println(verticalSpeed_mps);

    Serial.print(">jerkX_mps3: ");
    Serial.println(jerkX_mps3);

    Serial.print(">Altitude: ");
    Serial.println(altitude_m);
    */
}

void task5Hz()
{
    ledWork.toggle();
    // temperatureInside_degC = altimeter.getTemperature();
}

void task2Hz()
{
    const auto &airTelemetryPacket = MsgPacketizer::encode(0x0A,
                                                           static_cast<uint32_t>(millis()),
                                                           static_cast<char>(ident),
                                                           static_cast<uint8_t>(logger.getUsage()),
                                                           static_cast<bool>(doLogging),
                                                           static_cast<uint8_t>(logger.framNumber()),
                                                           static_cast<int16_t>(accelerationX_mps2 * 10),
                                                           static_cast<int16_t>(accelerationY_mps2 * 10),
                                                           static_cast<int16_t>(accelerationZ_mps2 * 10),
                                                           static_cast<int16_t>(sqrt(accelerationX_mps2 * accelerationX_mps2 + accelerationY_mps2 * accelerationY_mps2 + accelerationZ_mps2 * accelerationZ_mps2) * 10),
                                                           static_cast<int16_t>(roll_deg * 10),
                                                           static_cast<int16_t>(pitch_deg * 10),
                                                           static_cast<int16_t>(yaw_deg * 10),
                                                           static_cast<int16_t>(forceX_N * 10),
                                                           static_cast<int16_t>(jerkX_mps3 * 10),
                                                           static_cast<int16_t>(altitude_m * 10),
                                                           static_cast<int16_t>(verticalSpeed_mps * 10),
                                                           static_cast<int16_t>(estimated * 10),
                                                           static_cast<int16_t>(apogee * 10),
                                                           static_cast<int16_t>(externalVoltage_V * 100),
                                                           static_cast<int16_t>(batteryVoltage_V * 100),
                                                           static_cast<int16_t>(busVoltage_V * 100),
                                                           static_cast<int16_t>(externalCurrent_mA * 100),
                                                           static_cast<int16_t>(batteryCurrent_mA * 100),
                                                           static_cast<int16_t>(busCurrent_mA * 100),
                                                           static_cast<int16_t>(externalPower_W * 10),
                                                           static_cast<int16_t>(batteryPower_W * 10),
                                                           static_cast<int16_t>(busPower_W * 10),
                                                           static_cast<int16_t>(externalDieTemperature_C * 10),
                                                           static_cast<int16_t>(batteryDieTemperature_C * 10),
                                                           static_cast<int16_t>(busDieTemperature_C * 10));

    telemeter.reserveData(airTelemetryPacket.data.data(), airTelemetryPacket.data.size());
    telemeter.sendReservedData();
    ledLoRaTx.toggle();
}

void setup()
{
    analogReadResolution(12);
    Serial.begin(115200);
    Wire.begin();
    SPI.begin();

    can.begin();
    telemeter.initialize(924.2E6, 500E3);

    bno055.begin();

    // altitudeAverage.begin(); // LPS28DFW内部で実行してもらっているのでソフト側では処理しない．

    primary.initialize(0x5C);
    secondary.initialize(0x5D);
    primary.setReferencePressure(1014.0);   // METARのQから始まる値を基準気圧に設定してみる．
    secondary.setReferencePressure(1014.0); // METARのQから始まる値を基準気圧に設定してみる．

    Tasks.add(&task200Hz)->startFps(200);
    Tasks.add(&task100Hz)->startFps(100);
    // Tasks.add(&task50Hz)->startFps(50);
    Tasks.add(&task20Hz)->startFps(20);
    Tasks.add(&task10Hz)->startFps(10);
    Tasks.add(&task5Hz)->startFps(5);
    Tasks.add(&task2Hz)->startFps(2);
    // Tasks.add(&calibrationPressure)->startOnceAfterSec(10);
}

void loop()
{
    Tasks.update();

    if (can.available())
    {
        switch (can.getLatestLabel())
        {
        case Var::Label::FLIGHT_DATA:
        {
            bool newDoLogging;
            can.receiveFlight(&flightMode, &flightTime, &newDoLogging, &ident);
            ledCanRx.toggle();

            if (doLogging != newDoLogging)
            {
                doLogging = newDoLogging;

                if (doLogging)
                {
                    logger.reset();
                    // altimeter.setReferencePressure();
                    gyroscopeIntegralX.reset();
                    gyroscopeIntegralY.reset();
                    gyroscopeIntegralZ.reset();
                    accelerationHighPassX.reset();
                    accelerationHighPassY.reset();
                    accelerationHighPassZ.reset();
                }
            }

            break;
        }
        case Var::Label::MONITOR_BUS:
        {
            can.receiveBusMonitor(&busVoltage_V, &busCurrent_mA, &busPower_W, &busDieTemperature_C);
            ledCanRx.toggle();

            break;
        }

        case Var::Label::MONITOR_BATTERY:
        {
            can.receiveBatteryMonitor(&batteryVoltage_V, &batteryCurrent_mA, &batteryPower_W, &batteryDieTemperature_C);
            ledCanRx.toggle();

            break;
        }

        case Var::Label::MONITOR_EXTERNAL:
        {
            can.receiveExternalMonitor(&externalVoltage_V, &externalCurrent_mA, &externalPower_W, &externalDieTemperature_C);
            ledCanRx.toggle();

            break;
        }
        }
    }
}
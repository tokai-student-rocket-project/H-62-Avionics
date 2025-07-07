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
#include "Lib_FlightMode.hpp"
#include "Lib_FlightTime.hpp"

// Flight Module を模擬
FlightMode flightMode;
FlightTime flightTime;
uint8_t flightModeTest;
uint16_t flightTimeTest;
bool flightPin = false;
////

char ident = '\0';
bool doLogging = true;

CAN can(7);
Telemeter telemeter;
Logger logger(14, A2, 13);

OutputPin ledWork(LED_BUILTIN);
OutputPin ledCanRx(0);
OutputPin ledCanTx(1);
OutputPin ledLoRaRx(4);
OutputPin ledLoRaTx(5);

// uint8_t flightMode;
// uint16_t flightTime;
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

float groundVoltage_V, batteryVoltage_V, busVoltage_V;
float groundCurrent_mA, batteryCurrent_mA, busCurrent_mA;
float groundPower_mW, batteryPower_mW, busPower_mW;
float groundDieTemperature_C, batteryDieTemperature_C, busDieTemperature_C;

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

// Flight Module の模擬
void flightPinIsOpen()
{
    flightPin = true;
}
///

// Flight Module の模擬
void setTimer(uint32_t separation1ProtectionTime, uint32_t separation1ForceTime, uint32_t separation2ProtectionTime, uint32_t separation2ForceTime, uint32_t landingTime)
{
    flightTime.SEPARATION_1_PROTECTION_TIME = separation1ProtectionTime;
    flightTime.SEPARATION_1_FORCE_TIME = separation1ForceTime;
    flightTime.SEPARATION_2_PROTECTION_TIME = separation2ProtectionTime;
    flightTime.SEPARATION_2_FORCE_TIME = separation2ForceTime;
    flightTime.LANDING_TIME = landingTime;
}
///

// Flight Module の模擬
void flightModeOn()
{
    if (flightMode.isNot(Var::FlightMode::STANDBY))
    {
        return;
    }

    if (flightMode.is(Var::FlightMode::DATA_PROTECTION))
    {
        return;
    }

    flightMode.change(Var::FlightMode::READY_TO_FLY);
}
///

void task200Hz()
{
    primaryTemperature_C = primary.getTemperature();
    secondaryTemperature_C = secondary.getTemperature();

    primaryPressure_hPa = primary.getPressure();
    secondaryPressure_hPa = secondary.getPressure();

    float temperature_C = (primaryTemperature_C + secondaryTemperature_C) / 2;
    altitude_m = (primary.getAltitude(temperature_C) + secondary.getAltitude(temperature_C)) / 2;

    // Serial.print(">Altitude: ");
    // Serial.println(altitude_m);

    // Serial.print(">primaryAltitude_m: ");
    // Serial.println(primary.getAltitude(temperature_C));
}

void task100Hz()
{
    // Flight Module の模擬
    switch (flightMode.current())
    {
    case (Var::FlightMode::STANDBY):
    {
        if (flightPin)
        {
            flightModeOn();
            Serial.println("WKUP");
        }

        break;
    }

    case (Var::FlightMode::READY_TO_FLY):
    {
        if (flightPin)
        {
            flightMode.change(Var::FlightMode::POWERED_CLIMB);
            flightTime.setZero();
            Serial.println("IGNT");
        }

        break;
    }

    case (Var::FlightMode::POWERED_CLIMB):
    {
        if (flightTime.isElapsed(2000) && forceX_N < 0 && jerkX_mps3 < 0)
        {
            flightMode.change(Var::FlightMode::FREE_CLIMB);
            Serial.println("BOUT");
        }

        break;
    }

    case (Var::FlightMode::FREE_CLIMB):
    {
        if (isFalling)
        {
            flightMode.change(Var::FlightMode::FREE_DESCENT);
            Serial.println("APOG");
        }

        break;
    }

    case (Var::FlightMode::FREE_DESCENT):
    {
        if (flightTime.isElapsed(flightTime.SEPARATION_1_PROTECTION_TIME))
        {
            flightMode.change(Var::FlightMode::DROGUE_CHUTE_DESCENT);
            Serial.println("SEP1");
        }

        break;
    }

    case (Var::FlightMode::DROGUE_CHUTE_DESCENT):
    {
        if (flightTime.isElapsed(flightTime.SEPARATION_2_PROTECTION_TIME) && altitude_m < 100)
        {
            flightMode.change(Var::FlightMode::MAIN_CHUTE_DESCENT);
            Serial.println("SEP2");
        }

        break;
    }

    case (Var::FlightMode::MAIN_CHUTE_DESCENT):
    {
        if (flightTime.isElapsed(flightTime.LANDING_TIME))
        {
            flightMode.change(Var::FlightMode::LANDED);
            Serial.println("LAND");
        }

        break;
    }

    case (Var::FlightMode::LANDED):
    {
        if (flightTime.isElapsed(flightTime.LANDING_TIME + 5000))
        {
            flightMode.change(Var::FlightMode::SHUTDOWN);
            Serial.println("SDWN");
        }

        break;
    }
    }
    // flightMode.print();
    // flightModeTest = flightMode.currentNumber();
    /// Flight Module の模擬

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

    linearAccelerationX_mps2 = accelerationHighPassX.get(accelerationX_mps2 - gravityX_mps2, deltaTime);
    linearAccelerationY_mps2 = accelerationHighPassY.get(accelerationY_mps2 - gravityY_mps2, deltaTime);
    linearAccelerationZ_mps2 = accelerationHighPassZ.get(accelerationZ_mps2 - gravityZ_mps2, deltaTime);

    const auto &logPacket = MsgPacketizer::encode(0x0A,
                                                  ident, millis(), flightTimeTest, flightModeTest, logger.getUsage(),
                                                  accelerationX_mps2, accelerationY_mps2, accelerationZ_mps2,
                                                  gyroscopeX_dps, gyroscopeY_dps, gyroscopeZ_dps,
                                                  magnetometerX_nT, magnetometerY_nT, magnetometerZ_nT,
                                                  roll_deg, pitch_deg, yaw_deg,
                                                  forceX_N, jerkX_mps3,
                                                  referencePressure_hPa, altitude_m, verticalSpeed_mps, verticalAcceleration_msp2, estimated, apogee, isFalling,
                                                  groundVoltage_V, batteryVoltage_V, busVoltage_V,
                                                  groundCurrent_mA, batteryCurrent_mA, busCurrent_mA,
                                                  groundPower_mW, batteryPower_mW, busPower_mW);

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

    flightMode.print();

    Serial.print(">isFalling: ");
    Serial.println(isFalling);

    Serial.print(">verticalSpeed_mps: ");
    Serial.println(verticalSpeed_mps);

    Serial.print(">jerkX_mps3: ");
    Serial.println(jerkX_mps3);

    Serial.print(">Altitude: ");
    Serial.println(altitude_m);
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
                                                           static_cast<uint8_t>(groundVoltage_V * 10),
                                                           static_cast<uint8_t>(batteryVoltage_V * 10),
                                                           static_cast<uint8_t>(busVoltage_V * 10),
                                                           static_cast<int16_t>(groundCurrent_mA * 10),
                                                           static_cast<int16_t>(batteryCurrent_mA * 10),
                                                           static_cast<int16_t>(busCurrent_mA * 10),
                                                           static_cast<int8_t>(groundPower_mW / 100),
                                                           static_cast<int8_t>(batteryPower_mW / 100),
                                                           static_cast<int8_t>(busPower_mW / 100));

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
    // altimeter.initialize();
    // altimeter.setReferencePressure();

    // altitudeAverage.begin();

    primary.initialize(0x5C);
    secondary.initialize(0x5D);
    primary.setReferencePressure(1003.5);
    secondary.setReferencePressure(1003.5);

    setTimer(
        10000, // SEPARATION_1_PROTECTION_TIME
        12000, // SEPARATION_1_FORCE_TIME
        15000, // SEPARATION_2_PROTECTION_TIME // 18382
        17000, // SEPARATION_2_FORCE_TIME // 19382
        20000  // LANDING_TIME
    );

    Tasks.add(&task200Hz)->startFps(200);
    Tasks.add(&task100Hz)->startFps(100);
    // Tasks.add(&task50Hz)->startFps(50);
    Tasks.add(&task20Hz)->startFps(20);
    Tasks.add(&task10Hz)->startFps(10);
    Tasks.add(&task5Hz)->startFps(5);
    Tasks.add(&task2Hz)->startFps(2);
    // Tasks.add(&calibrationPressure)->startOnceAfterSec(10);
    Tasks.add(&flightPinIsOpen)->startOnceAfterSec(15);
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
            // can.receiveFlight(&flightMode, &flightTime, &newDoLogging, &ident);
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
        case Var::Label::MONITOR_VOLTAGE:
        {
            can.receiveVoltage(&groundVoltage_V, &batteryVoltage_V, &busVoltage_V);
            ledCanRx.toggle();

            Serial.print(">groundVoltage_V: ");
            Serial.println(groundVoltage_V);

            break;
        }
        }
    }
}

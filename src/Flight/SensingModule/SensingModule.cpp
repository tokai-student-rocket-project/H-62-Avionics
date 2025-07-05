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
#include <Adafruit_LPS28.h>

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
uint16_t flightTime;
RateMonitor monitor10Hz;
RateMonitor monitor100Hz;

// Altimeter altimeter;
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

float groundVoltage_V, batteryVoltage_V, tieVoltage_V, busVoltage_V;
float groundCurrent_mA, batteryCurrent_mA, tieCurrent_mA, busCurrent_mA;
float groundPower_mW, batteryPower_mW, tiePower_mW, busPower_mW;

float temperatureRegulator1_degC, temperatureRegulator2_degC, temperatureRegulator3_degC, temperatureConduction_degC;
float temperatureOutside_degC, temperatureInside_degC;

float temperatureVentPort_degC, temperatureTankAtmosphere_degC;
// uint32_t sutegomaTime_ms;
// float sutegomeTaskRate_Hz;

float pressure_kPa;
float altitude_m;
movingAvg altitudeAverage(10);
Calculus::Differential<float> altitudeGradient(5);
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

namespace sensor
{
    Altimeter primary;
    Altimeter secondary;

    void measurePressure();
    void referencePressure();
    void calibrationPressure();
}

void sensor::measurePressure()
{
    sensor::primary.setReferencePressure(1013.25);
    Serial.print(">Primary Pressure: ");
    Serial.println(sensor::primary.getPressure());

    Serial.print(">Sencondary Pressure: ");
    Serial.println(sensor::secondary.getPressure());

    float temperature_C = (sensor::primary.getTemperature() + sensor::secondary.getTemperature()) / 2;
    Serial.print(">Altitude: ");
    Serial.println(sensor::primary.getAltitude(temperature_C));

    Serial.print(">Primary ReferencePressure: ");
    Serial.println(sensor::primary.getReferencePressure());

    Serial.print(">Secondary ReferencePressure: ");
    Serial.println(sensor::secondary.getReferencePressure());
}

// void sensor::calibrationPressure()
// {
//     float pressureSum = 0;
//     const int calibrationSamples = 100;
//     for (int i = 0; i < calibrationSamples; i++)
//     {
//         sensors_event_t pressure, temp;
//         sensor::primary.getEvent(&pressure, &temp);
//     }
// }

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
                                                  ident, millis(), flightTime, flightMode, logger.getUsage(),
                                                  accelerationX_mps2, accelerationY_mps2, accelerationZ_mps2,
                                                  gyroscopeX_dps, gyroscopeY_dps, gyroscopeZ_dps,
                                                  magnetometerX_nT, magnetometerY_nT, magnetometerZ_nT,
                                                  roll_deg, pitch_deg, yaw_deg,
                                                  forceX_N, jerkX_mps3,
                                                  pressure_kPa, altitude_m, verticalSpeed_mps, verticalAcceleration_msp2, estimated, apogee, isFalling,
                                                  groundVoltage_V, batteryVoltage_V, tieVoltage_V, busVoltage_V,
                                                  groundCurrent_mA, batteryCurrent_mA, tieCurrent_mA, busCurrent_mA,
                                                  groundPower_mW, batteryPower_mW, tiePower_mW, busPower_mW,
                                                  temperatureRegulator1_degC, temperatureRegulator2_degC, temperatureRegulator3_degC, temperatureConduction_degC,
                                                  temperatureOutside_degC);

    if (doLogging)
    {
        logger.write(logPacket.data.data(), logPacket.data.size());
    }
}

void task50Hz()
{
    ledWork.toggle();

    temperatureOutside_degC = outside.getTemperature_degC();
    // pressure_kPa = altimeter.getPressure();
    // altitude_m = altimeter.getAltitude(temperatureOutside_degC + 273.15);

    /*
    ////////////////////////// Teleplot出力
    Serial.print(">pressure:");
    Serial.println(pressure_kPa);
    Serial.print(">alti:");
    Serial.println(altitude_m);
    //////////////////////////
    */
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

    verticalSpeed_mps = altitudeGradient.get((float)altitudeAverage.reading((int16_t)(altitude_m * 10)) / 10.0, deltaTime);
    verticalAcceleration_msp2 = verticalSpeedGradient.get(verticalSpeed_mps, deltaTime);

    estimated = -verticalSpeed_mps / verticalAcceleration_msp2;
    apogee = altitude_m + (verticalSpeed_mps * estimated + 0.5 * verticalAcceleration_msp2 * estimated * estimated);
    isFalling = verticalSpeed_mps < 0;

    can.sendTrajectory(isFalling, altitude_m);

    ledCanTx.toggle();
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
                                                           static_cast<uint8_t>(tieVoltage_V * 10),
                                                           static_cast<uint8_t>(busVoltage_V * 10),
                                                           static_cast<int16_t>(groundCurrent_mA * 10),
                                                           static_cast<int16_t>(batteryCurrent_mA * 10),
                                                           static_cast<int16_t>(tieCurrent_mA * 10),
                                                           static_cast<int16_t>(busCurrent_mA * 10),
                                                           static_cast<int8_t>(groundPower_mW / 100),
                                                           static_cast<int8_t>(batteryPower_mW / 100),
                                                           static_cast<int8_t>(tiePower_mW / 100),
                                                           static_cast<int8_t>(busPower_mW / 100),
                                                           static_cast<int16_t>(temperatureRegulator1_degC * 10),
                                                           static_cast<int16_t>(temperatureRegulator2_degC * 10),
                                                           static_cast<int16_t>(temperatureRegulator3_degC * 10),
                                                           static_cast<int16_t>(temperatureConduction_degC * 10),
                                                           static_cast<int16_t>(temperatureOutside_degC * 10),
                                                           static_cast<int16_t>(temperatureInside_degC * 10),
                                                           static_cast<int16_t>(temperatureVentPort_degC * 10),
                                                           static_cast<int16_t>(temperatureTankAtmosphere_degC * 10));

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

    // sensor::primary.initialize(0x5C);
    // sensor::secondary.initialize(0x5D);

    Tasks.add(&task100Hz)->startFps(100);
    Tasks.add(&task50Hz)->startFps(50);
    Tasks.add(&task20Hz)->startFps(20);
    Tasks.add(&task10Hz)->startFps(10);
    Tasks.add(&task5Hz)->startFps(5);
    Tasks.add(&task2Hz)->startFps(2);
    // Tasks.add(&sensor::measurePressure)->startFps(200);
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

            /*
            case Var::Label::SUTEGOMA_TEMPERATURE:
            {
              can.receiveSutegomaTemperature(&temperatureVentPort_degC, &temperatureTankAtmosphere_degC);
              ledCanRx.toggle();

              break;
            }
            */

            /*
            case Var::Label::SUTEGOMA_PERFORMANCE:
            {
              can.receiveSutegomaPerformance(&sutegomaTime_ms, &sutegomeTaskRate_Hz);
              ledCanRx.toggle();

              break;
            }
            */
        }
    }
}

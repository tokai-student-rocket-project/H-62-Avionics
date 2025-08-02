#include <Arduino.h>
#include <TaskManager.h>
#include "Lib_CAN.hpp"
#include "Lib_PowerMonitor.hpp"
#include "Lib_Var.hpp"
#include "Lib_OutputPin.hpp"
#include "Lib_Neopixel.hpp"

char ident = '\0';
bool doLogging = false;
uint8_t flightMode = 0;
uint32_t flightTime = 0;

Neopixel status(12);

CAN can(26);
PowerMonitor powerMonitor;

int16_t batteryMinimumVoltage_V = 11.0;
int16_t externalThresholdVoltage = 10.0;
OutputPin batteryEn(0);
OutputPin ltc4353GroundEn(28);
OutputPin ltc4353BatteryEn(29);

// RGB LED Config
OutputPin redLed(17);
OutputPin greenLed(16);
OutputPin blueLed(25);

float externalVoltage_V, batteryVoltage_V, busVoltage_V;
float externalCurrent_mA, batteryCurrent_mA, busCurrent_mA;
float externalPower_W, batteryPower_W, busPower_W;
float externalTemperature_C, batteryTemperature_C, busTemperature_C;

void task5Hz()
{
    powerMonitor.getVoltage(&externalVoltage_V, &batteryVoltage_V, &busVoltage_V);
    powerMonitor.getCurrent(&externalCurrent_mA, &batteryCurrent_mA, &busCurrent_mA);
    powerMonitor.getPower(&externalPower_W, &batteryPower_W, &busPower_W);
    powerMonitor.getTemperature(&externalTemperature_C, &batteryTemperature_C, &busTemperature_C);

    // Ground
    Serial.print(">Gpu_Voltage_V: ");
    Serial.println(externalVoltage_V);
    Serial.print(">Gpu_Current_mA: ");
    Serial.println(externalCurrent_mA);
    Serial.print(">Gpu_Power_W: ");
    Serial.println(externalPower_W);
    Serial.print(">Gpu_Temperature_C: ");
    Serial.println(externalTemperature_C);

    // Battery
    Serial.print(">Bat_Voltage_V: ");
    Serial.println(batteryVoltage_V);
    Serial.print(">Bat_Current_mA: ");
    Serial.println(batteryCurrent_mA);
    Serial.print(">Bat_Power_W: ");
    Serial.println(batteryPower_W);
    Serial.print(">Bat_Temperature_C: ");
    Serial.println(batteryTemperature_C);

    // Bus
    Serial.print(">Bus_Voltage_V: ");
    Serial.println(busVoltage_V);
    Serial.print(">Bus_Current_mA: ");
    Serial.println(busCurrent_mA);
    Serial.print(">Bus_Power_W: ");
    Serial.println(busPower_W);
    Serial.print(">Bus_Temperature_C: ");
    Serial.println(busTemperature_C);

    greenLed.toggle();
    can.sendBusMonitor(busVoltage_V, busCurrent_mA, busPower_W, busTemperature_C);
    can.sendBatteryMonitor(batteryVoltage_V, batteryCurrent_mA, batteryPower_W, batteryTemperature_C);
    can.sendExternalMonitor(externalVoltage_V, externalCurrent_mA, externalPower_W, externalTemperature_C);
}

void task2Hz()
{
    Serial.print(">batteryEnable: ");
    Serial.println(batteryEn.get());

    Serial.print(">ltc3119Powergood: ");
    Serial.println(digitalRead(1));
    status.noticedRainbow();
}

void setup()
{
    Serial.begin(115200);
    SPI.begin();

    pinMode(1, INPUT);

    batteryEn.low(); // 実際の実装
    ltc4353BatteryEn.high();
    // batteryEn.high(); // テスト実装

    can.begin();
    powerMonitor.initialize();

    status.init(11);
    // status.noticedGreen();

    redLed.high();
    blueLed.high();
    greenLed.high();

    Tasks.add(&task5Hz)->startFps(5);
    Tasks.add(&task2Hz)->startFps(2);
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
            blueLed.toggle();
            can.receiveFlight(&flightMode, &flightTime, &doLogging, &ident);

            switch (flightMode)
            {
            case (0):
            {
                // if (externalVoltage_V > externalThresholdVoltage)
                // {
                //     batteryEn.high();
                // }
                // else
                batteryEn.low();

                break;
            }

            case (1):
            {
                if (externalVoltage_V > externalThresholdVoltage)
                {
                    batteryEn.high();
                }
                else
                    batteryEn.low();

                break;
            }

            case (2):
            {
                if (batteryVoltage_V < batteryMinimumVoltage_V)
                {
                    batteryEn.low();
                }
                else
                    batteryEn.high();

                break;
            }

            case (3):
            {
                if (batteryVoltage_V < batteryMinimumVoltage_V)
                {
                    batteryEn.low();
                }
                else
                    batteryEn.high();

                break;
            }

            case (4):
            {
                if (batteryVoltage_V < batteryMinimumVoltage_V)
                {
                    batteryEn.low();
                }
                else
                    batteryEn.high();

                break;
            }

            case (5):
            {
                if (batteryVoltage_V < batteryMinimumVoltage_V)
                {
                    batteryEn.low();
                }
                else
                    batteryEn.high();

                break;
            }

            case (6):
            {
                if (batteryVoltage_V < batteryMinimumVoltage_V)
                {
                    batteryEn.low();
                }
                else
                    batteryEn.high();
                Serial.println(batteryEn.get());

                break;
            }

            case (7):
            {
                if (batteryVoltage_V < batteryMinimumVoltage_V)
                {
                    batteryEn.low();
                }
                else
                    batteryEn.high();

                break;
            }

            case (8):
            {
                if (batteryVoltage_V < batteryMinimumVoltage_V)
                {
                    batteryEn.low();
                }
                else
                    batteryEn.high();

                break;
            }
            }

            break;
        }
        }
    }
}

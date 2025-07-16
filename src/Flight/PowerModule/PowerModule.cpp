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
uint16_t flightTime = 0;

Neopixel status(12);

CAN can(26);
PowerMonitor powerMonitor;

OutputPin batteryEn(0);
OutputPin ltc4353GroundEn(28);
OutputPin ltc4353BatteryEn(29);
OutputPin redLed(17);
OutputPin greenLed(16);
OutputPin blueLed(25);

float groundVoltage_V, batteryVoltage_V, busVoltage_V;
float groundCurrent_mA, batteryCurrent_mA, busCurrent_mA;
float groundPower_mW, batteryPower_mW, busPower_mW;
float groundTemperature_C, batteryTemperature_C, busTemperature_C;

void task5Hz()
{
    powerMonitor.getVoltage(&groundVoltage_V, &batteryVoltage_V, &busVoltage_V);
    powerMonitor.getCurrent(&groundCurrent_mA, &batteryCurrent_mA, &busCurrent_mA);
    powerMonitor.getPower(&groundPower_mW, &batteryPower_mW, &busPower_mW);
    powerMonitor.getTemperature(&groundTemperature_C, &batteryTemperature_C, &busTemperature_C);

    // Ground
    Serial.print(">Gpu_Voltage_V: ");
    Serial.println(groundVoltage_V);
    Serial.print(">Gpu_Current_mA: ");
    Serial.println(groundCurrent_mA);
    Serial.print(">Gpu_Power_mW: ");
    Serial.println(groundPower_mW);
    Serial.print(">Gpu_Temperature_C: ");
    Serial.println(groundTemperature_C);

    // Battery
    Serial.print(">Bat_Voltage_V: ");
    Serial.println(batteryVoltage_V);
    Serial.print(">Bat_Current_mA: ");
    Serial.println(batteryCurrent_mA);
    Serial.print(">Bat_Power_mW: ");
    Serial.println(batteryPower_mW);
    Serial.print(">Bat_Temperature_C: ");
    Serial.println(batteryTemperature_C);

    // Bus
    Serial.print(">Bus_Voltage_V: ");
    Serial.println(busVoltage_V);
    Serial.print(">Bus_Current_mA: ");
    Serial.println(busCurrent_mA);
    Serial.print(">Bus_Power_mW: ");
    Serial.println(busPower_mW);
    Serial.print(">Bus_Temperature_C: ");
    Serial.println(busTemperature_C);

    Serial.print(">Battery EN: ");
    Serial.println(batteryEn.get());
    greenLed.toggle();
    can.sendVoltage(groundVoltage_V, batteryVoltage_V);
}

void setup()
{
    Serial.begin(115200);
    SPI.begin();

    // batteryEn.low(); // 実際の実装
    batteryEn.high(); // テスト実装

    can.begin();
    powerMonitor.initialize();

    status.init(11);
    status.noticedGreen();

    redLed.high();
    blueLed.high();
    greenLed.high();

    Tasks.add(&task5Hz)
        ->startFps(5);
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
                batteryEn.low();
                Serial.println(batteryEn.get());

                Serial.println("STANDBY");
                break;
            }

            case (1):
            {
                batteryEn.high();
                Serial.println(batteryEn.get());

                Serial.println("READT_TO_FLY");
                break;
            }

            case (2):
            {
                batteryEn.high();
                Serial.println(batteryEn.get());

                break;
            }

            case (3):
            {
                batteryEn.high();
                Serial.println(batteryEn.get());

                break;
            }

            case (4):
            {
                batteryEn.high();
                Serial.println(batteryEn.get());

                break;
            }

            case (5):
            {
                batteryEn.high();
                Serial.println(batteryEn.get());

                break;
            }

            case (6):
            {
                batteryEn.high();
                Serial.println(batteryEn.get());

                break;
            }

            case (7):
            {
                batteryEn.high();
                Serial.println(batteryEn.get());

                break;
            }

            case (8):
            {
                batteryEn.high();
                Serial.println(batteryEn.get());

                break;
            }
            }

            break;
        }
        }
    }

    if (batteryVoltage_V < 10.8)
    {
        batteryEn.low();
    }
    else
        batteryEn.high();
}

#include <Arduino.h>
#include <TaskManager.h>
#include "Lib_CAN.hpp"
#include "Lib_PowerMonitor.hpp"
#include "Lib_Var.hpp"
#include "Lib_OutputPin.hpp"

char ident = '\0';
bool doLogging = false;
uint8_t flightMode = 0;
uint16_t flightTime = 0;

CAN can(26);
PowerMonitor powerMonitor;

OutputPin batteryEn(0);
OutputPin ltc4353GroundEn(28);
OutputPin ltc4353BatteryEn(29);

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

    Serial.print(">Voltage_V ");
}

void setup()
{
    Serial.begin(115200);
    SPI.begin();

    batteryEn.low();

    can.begin();
    powerMonitor.initialize();

    Tasks.add(&task5Hz)->startFps(5);
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

            break;
        }
        }
    }
}

#include <Arduino.h>
#include <Adafruit_INA228.h>
#include <TaskManager.h>

// Adafruit_INA228 External = Adafruit_INA228();
// Adafruit_INA228 Battery = Adafruit_INA228();

namespace external
{
    Adafruit_INA228 ina228 = Adafruit_INA228();
    void init();
    void readData();
}

namespace battery
{
    Adafruit_INA228 ina228 = Adafruit_INA228();
    void init();
    void readData();
}

void external::init()
{
    external::ina228.setShunt(0.15, 10.0);
    external::ina228.setAveragingCount(INA228_COUNT_16);
    external::ina228.setVoltageConversionTime(INA228_TIME_150_us);
    external::ina228.setCurrentConversionTime(INA228_TIME_280_us);
}

void battery::init()
{
    battery::ina228.setShunt(0.15, 10.0);
    battery::ina228.setAveragingCount(INA228_COUNT_16);
    battery::ina228.setVoltageConversionTime(INA228_TIME_150_us);
    battery::ina228.setCurrentConversionTime(INA228_TIME_280_us);
}

void external::readData()
{
    Serial.println("=== EXT ===");
    Serial.print("Current: ");
    Serial.print(external::ina228.getCurrent_mA());
    Serial.println(" mA");

    Serial.print("Bus Voltage: ");
    Serial.print(external::ina228.getBusVoltage_V());
    Serial.println(" V");

    Serial.print("Shunt Voltage: ");
    Serial.print(external::ina228.getShuntVoltage_mV());
    Serial.println(" mV");

    Serial.print("Power: ");
    Serial.print(external::ina228.getPower_mW());
    Serial.println(" mW");

    Serial.print("Energy: ");
    Serial.print(external::ina228.readEnergy());
    Serial.println(" J");

    Serial.print("Charge: ");
    Serial.print(external::ina228.readCharge());
    Serial.println(" C");

    Serial.print("Temperature: ");
    Serial.print(external::ina228.readDieTemp());
    Serial.println(" *C");

    Serial.println();
}

void battery::readData()
{
    Serial.println("=== BAT ===");
    Serial.print("Current: ");
    Serial.print(battery::ina228.getCurrent_mA());
    Serial.println(" mA");

    Serial.print("Bus Voltage: ");
    Serial.print(battery::ina228.getBusVoltage_V());
    Serial.println(" V");

    Serial.print("Shunt Voltage: ");
    Serial.print(battery::ina228.getShuntVoltage_mV());
    Serial.println(" mV");

    Serial.print("Power: ");
    Serial.print(battery::ina228.getPower_mW());
    Serial.println(" mW");

    Serial.print("Energy: ");
    Serial.print(battery::ina228.readEnergy());
    Serial.println(" J");

    Serial.print("Charge: ");
    Serial.print(battery::ina228.readCharge());
    Serial.println(" C");

    Serial.print("Temperature: ");
    Serial.print(battery::ina228.readDieTemp());
    Serial.println(" *C");

    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        if (!external::ina228.begin())
        {
            Serial.println("Couldn't find External Monitor");
            while (1)
                ;
        }

    if (!battery::ina228.begin())
    {
        Serial.println("Couldn't find Battery Monitor");
        while (1)
            ;
    }

    Serial.println("Found External Monitor!");
    Serial.println("Found Battery Monitor!");

    Tasks.add(&external::readData)->startFps(10);
    Tasks.add(&battery::readData)->startFps(10);
}

void loop()
{
    Tasks.update();
}
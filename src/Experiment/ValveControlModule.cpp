#include <Arduino.h>
#include <SPI.h>
#include <TaskManager.h>
#include <MsgPacketizer.h>
#include <Adafruit_NeoPixel.h>
#include "Lib_GseSignal.hpp"
#include "Lib_FlightMode.hpp"
#include "Lib_CountDetector.hpp"
#include "Lib_B3MSC1170A.hpp"
#include "Lib_Neopixel.hpp"
#include "Lib_Var.hpp"

// LED Config
const uint8_t redled = 17;   // GPIO17 // OK
const uint8_t greenled = 16; // GPIO16 // OK
const uint8_t blueled = 25;  // GPIO25 // OK
Neopixel Status(12);         // OK

// LoadSwitch Config
const uint8_t powerEN = 29;

// ID:0x01--MAIN
// ID:0x02--SUPPLY

B3MSC1170A mainValve;
void closeMainValve()
{
    mainValve.setPosition(0x01, -7000, 0);
    int16_t mainValveCurrentPossition = mainValve.readCurrentPosition(0x01);
    // if (mainValveCurrentPossition < -5000 || mainValveCurrentPossition > -6000)
    // {
    mainValve.setPosition(0x01, -7000, 0);
    // }
}
void openMainValve()
{
    mainValve.setPosition(0x01, 0, 0);
    int16_t mainValveCurrentPossition = mainValve.readCurrentPosition(0x01);
    // if (mainValveCurrentPossition < -500 || mainValveCurrentPossition > 500)
    // {
    mainValve.setPosition(0x01, 0, 0);
    // }
}
void closeMainValveToFlight()
{
    mainValve.setPosition(0x01, -100, 200);
    int16_t mainValveCurrentPossition = mainValve.readCurrentPosition(0x01);
    // if (mainValveCurrentPossition < -200 || mainValveCurrentPossition > 100)
    // {
    //   mainValve.setPosition(0x01, -100, 200);
    // }
    mainValve.setPosition(0x01, -100, 200);
}

B3MSC1170A supplyValve;
void openSupplyValve()
{
    supplyValve.setPosition(0x02, 0, 0);
    int16_t supplyValveCurrentPossition = supplyValve.readCurrentPosition(0x02);
    if (supplyValveCurrentPossition < -500 || supplyValveCurrentPossition > 500)
    {
        supplyValve.setPosition(0x02, 0, 0);
    }
}
void closeSupplyValve()
{
    supplyValve.setPosition(0x02, -9000, 0);
    int16_t supplyValveCurrentPossition = supplyValve.readCurrentPosition(0x02);
    if (supplyValveCurrentPossition < -8500 || supplyValveCurrentPossition > -9500)
    {
        supplyValve.setPosition(0x02, -9000, 0);
    }
}

Var::ValveMode currentValveMode = Var::ValveMode::WAITING;
Var::GseSignal currentGseSignal = Var::GseSignal::IGNITION_OFF;

GseSignal gseValve(6);   // OK
GseSignal gseIgniter(7); // OK

constexpr uint16_t MODE_CHANGING_THRESHOLD = 0;
CountDetector valveSignalCounter(MODE_CHANGING_THRESHOLD);
CountDetector igniterSignalCounter(MODE_CHANGING_THRESHOLD);

uint32_t retryCount = 0;
uint32_t lastSendTime = 0;
bool isVerified = true;

void setup()
{
    Serial.begin(115200);
    // Serial1.begin(115200, SERIAL_8N1);
    Serial.begin(115200, SERIAL_8N1);

    pinMode(redled, OUTPUT);
    pinMode(greenled, OUTPUT);
    pinMode(blueled, OUTPUT);

    pinMode(powerEN, OUTPUT);
    digitalWrite(powerEN, LOW);

    Status.init(11);

    mainValve.initialize(0x01);
    supplyValve.initialize(0x02);

    Tasks.add("openMainValve", [&]()
              { openMainValve(); });
    Tasks.add("openSupplyValve", [&]()
              { openSupplyValve(); });
}

void loop()
{
    Tasks.update();

    if (gseValve.isSignaled())
    {
        openMainValve();
        closeSupplyValve();
        Status.noticedRed();
    }
    else
    {
        closeMainValve();
        openSupplyValve();
        Status.noticedGreen();
    }
}

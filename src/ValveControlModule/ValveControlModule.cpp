#include <Arduino.h>
#include <SPI.h>
#include <TaskManager.h>
#include <MsgPacketizer.h>
#include <Adafruit_NeoPixel.h>
#include "Lib_Var.hpp"
#include "Lib_CAN.hpp"
#include "Lib_GseSignal.hpp"
#include "Lib_CountDetector.hpp"
#include "Lib_B3MSC1170A.hpp"
#include "Lib_Logger2.hpp"
#include "Lib_FRAM.hpp"
#include "Lib_Neopixel.hpp"

char ident = '\0';
bool doLogging = false;
uint8_t flightMode = 0;
uint16_t flightTime = 0;

const uint8_t redled = 17; //GPIO17 // OK
const uint8_t greenled = 16; //GPIO16 // OK
const uint8_t blueled = 25; //GPIO25 // OK
Neopixel Status(12); // OK

CAN can(26);
Logger logger(28, 29);
FRAM fram0(28);
FRAM fram1(29);

B3MSC1170A mainValve;
void openMainValve() { mainValve.setPosition(0x01, -5500, 0); }
void closeMainValve() { mainValve.setPosition(0x01, 0, 1000); }


B3MSC1170A supplyValve;
void openSupplyValve() { supplyValve.setPosition(0x02, 0, 200); }
void closeSupplyValve() { supplyValve.setPosition(0x02, -800, 300); }

Var::ValveMode currentValveMode = Var::ValveMode::LAUNCH;
Var::GseSignal currentGseSignal = Var::GseSignal::IGNITION_ON;

GseSignal gseValve(6); // OK
GseSignal gseIgniter(7); // OK

constexpr uint16_t MODE_CHANGING_THRESHOLD = 5;
CountDetector valveSignalCounter(MODE_CHANGING_THRESHOLD);
CountDetector igniterSignalCounter(MODE_CHANGING_THRESHOLD);

uint32_t retryCount = 0;
uint32_t lastSendTime = 0;
bool isVerified = true;

void verifyValve()
{
  if (isVerified)
  {
    return;
  }

  if (retryCount == 0 && (millis() - lastSendTime) < 300)
  {
    return;
  }

  if (retryCount >= 5)
  {
    return;
  }

  if ((millis() - lastSendTime) < 50)
  {
    return;
  }

  if (currentValveMode == Var::ValveMode::WAITING)
  {
    bool isClosed = (mainValve.readDesiredPosition(0x01) > -1000) && (mainValve.readDesiredPosition(0x01) < 1000);

    if (isClosed)
    {
      isVerified = true;
    }
    else
    {
      retryCount++;
      lastSendTime = millis();
      closeMainValve();
    }
  }

  if (currentValveMode == Var::ValveMode::LAUNCH)
  {
    bool isOpen = (mainValve.readDesiredPosition(0x01) > -7500) && (mainValve.readDesiredPosition(0x01) < -5500);

    if (isOpen)
    {
      isVerified = true;
    }
    else
    {
      retryCount++;
      lastSendTime = millis();
      openMainValve();
    }
  }
}

void addTaskIfNotExisted(const String &name, void (*callback)())
{
  if (!Tasks.exists(name))
  {
    Tasks.add(name, callback);
  }
}

void changeMode(Var::ValveMode nextMode)
{
  if (nextMode == currentValveMode)
    return;

  retryCount = 0;
  lastSendTime = millis();
  isVerified = false;

  if (nextMode == Var::ValveMode::LAUNCH)
  {
    closeSupplyValve();

    addTaskIfNotExisted("delayed-open-main-valve", &openMainValve);
    Tasks["delayed-open-main-valve"]->startOnceAfterSec(0.3);
  }

  if (nextMode == Var::ValveMode::WAITING)
  {
    closeMainValve();
    openSupplyValve();
  }

  currentValveMode = nextMode;
}

void changeIgnition(Var::GseSignal nextMode)
{
  if (nextMode == currentGseSignal)
    return;
  if (nextMode == Var::GseSignal::IGNITION_ON)
  {
    Status.noticedGreen();
  }

  if (nextMode == Var::GseSignal::IGNITION_OFF)
  {
    Status.noticedBlue();
  }

  currentGseSignal = nextMode;
}

void syncFlightMode()
{
  if (can.available())
  {
    switch (can.getLatestLabel())
    {
    case Var::Label::FLIGHT_DATA:
    {
      can.receiveFlight(&flightMode, &flightTime, &doLogging, &ident);
      switch (flightMode) {
        case (0): // STANDBY
        {
          igniterSignalCounter.update(gseIgniter.isSignaled());

          if (igniterSignalCounter.isExceeded())
          {
            changeIgnition(Var::GseSignal::IGNITION_ON);
          }
          else
          {
            changeIgnition(Var::GseSignal::IGNITION_OFF);
          }

          valveSignalCounter.update(gseValve.isSignaled());
          if (valveSignalCounter.isExceeded())
          {
            changeMode(Var::ValveMode::LAUNCH);
          }
          else
          {
            changeMode(Var::ValveMode::WAITING);
          }
          verifyValve();

          Serial.println("STANDBY");
          break;
        }

        case (1): // READY_TO_FLY
        {
          igniterSignalCounter.update(gseIgniter.isSignaled());

          if (igniterSignalCounter.isExceeded())
          {
            changeIgnition(Var::GseSignal::IGNITION_ON);
          }
          else
          {
            changeIgnition(Var::GseSignal::IGNITION_OFF);
          }

          valveSignalCounter.update(gseValve.isSignaled());
          if (valveSignalCounter.isExceeded())
          {
            changeMode(Var::ValveMode::LAUNCH);
          }
          else
          {
            changeMode(Var::ValveMode::WAITING);
          }
          verifyValve();

          Serial.println("READY_TO_FLY");
          break;
        }

        case (2): // POWERED_CLIMB
        {
          changeMode(Var::ValveMode::LAUNCH);
          Serial.println("POWERED_CLIMB");
          break;
        }

        case (3): // FREE_CLIMB
        {
          changeMode(Var::ValveMode::LAUNCH);
          Serial.println("FREE_CLIMB");
          break;
        }

        case (4): // FREE_DESCENT
        {
          changeMode(Var::ValveMode::LAUNCH);
          Serial.println("FREE_DESCENT");
          break;
        }

        case (5): // DROGUE_CHUTE_DESCENT
        {
          changeMode(Var::ValveMode::LAUNCH);
          Serial.println("DROGUE_CHUTE_DESCENT");
          break;
        }

        case (6): // MAIN_CHUTE_DESCENT
        {
          changeMode(Var::ValveMode::LAUNCH);
          Serial.println("MAIN_CHUTE_DESCENT");
          break;
        }

        case (7): // LANDED
        {
          closeSupplyValve();
          closeMainValve();
          Serial.println("LANDED");
          break;
        }

        case (8): // SHUTDOWN
        {
          mainValve.torqueOff(0x01);
          Serial.println("SHUTDOWN");
          break;
        }
      }
    }
    }
  }
}

void sendValveMode()
{
  can.sendValveMode(currentValveMode == Var::ValveMode::LAUNCH);
}

void sendIgnition()
{
  can.sendIgnition(currentGseSignal == Var::GseSignal::IGNITION_ON);
}

void sendValveData()
{
  can.sendValveDataPart1(
      mainValve.readMotorTemperature(0x01),
      mainValve.readMcuTemperature(0x01),
      mainValve.readCurrent(0x01),
      mainValve.readVoltage(0x01));

  can.sendValveDataPart2(
      mainValve.readCurrentPosition(0x01),
      mainValve.readDesiredPosition(0x01),
      mainValve.readCurrentVelosity(0x01));
  
  can.sendValveDataPart3(
      supplyValve.readCurrentPosition(0x02),
      supplyValve.readVoltage(0x02));
}

void readData()
{
  Serial.println(digitalRead(7)); // Signal Valve
  Serial.println(digitalRead(6)); // Signal Ignition
}

void task10Hz() {
  const auto &logPacket = MsgPacketizer::encode(0x0A, millis(), doLogging);
  logger.write(logPacket.data.data(), logPacket.data.size());

  uint8_t fram0_statusByte;
  uint8_t fram1_statusByte;
  fram0.getStatus(&fram0_statusByte);
  fram1.getStatus(&fram1_statusByte);
  Serial.print("Fram0_Status: 0b");
  Serial.println(fram0_statusByte);

  Serial.print("Fram1_Status: 0b");
  Serial.println(fram1_statusByte);


  // bool isWriteEnabled = (fram0_statusByte >> 1)& 0x01;
  // Serial.print("Write Enable Latch(WEL) status: ");
  // Serial.println(isWriteEnabled ? "Enabled" : "Disabled");

  digitalWrite(redled, !digitalRead(redled));
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1);

  
  SPI.begin();
  can.begin();


  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT); 
  pinMode(blueled, OUTPUT);
  Status.init(11);

  digitalWrite(redled, LOW); // RGB LED OFF
  digitalWrite(greenled, HIGH); // RGB LED OFF
  digitalWrite(blueled, HIGH); // RGB LED OFF
  
  supplyValve.initialize(0x01);
  mainValve.initialize(0x02);

  // Tasks.add(&syncFlightMode)->startFps(100);
  // Tasks.add(&sendValveMode)->startFps(60);
  // Tasks.add(&sendIgnition)->startFps(60);
  // Tasks.add(&sendValveData)->startFps(10);
  // Tasks.add(&readData)->startFps(5);
  Tasks.add(&task10Hz)->startFps(10);

  changeMode(Var::ValveMode::WAITING);
  changeIgnition(Var::GseSignal::IGNITION_OFF);
}

void loop()
{
  Tasks.update();
}
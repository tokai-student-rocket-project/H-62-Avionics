#include <Arduino.h>
#include <SPI.h>
#include <TaskManager.h>
#include <MsgPacketizer.h>
#include <Adafruit_NeoPixel.h>
#include "Lib_Var.hpp"
#include "Lib_CAN.hpp"
#include "Lib_GseSignal.hpp"
#include "Lib_FlightMode.hpp"
#include "Lib_CountDetector.hpp"
#include "Lib_B3MSC1170A.hpp"
#include "Lib_Logger2.hpp"
#include "Lib_Neopixel.hpp"

char ident = '\0';
bool doLogging = false;
uint8_t flightMode = 0;
uint16_t flightTime = 0;

const uint8_t redled = 17;   // GPIO17 // OK
const uint8_t greenled = 16; // GPIO16 // OK
const uint8_t blueled = 25;  // GPIO25 // OK
Neopixel Status(12);         // OK

CAN can(26);
Logger logger(28, 29);

B3MSC1170A supplyValve;
void openSupplyValve()
{
  supplyValve.setPosition(0x01, 0, 1000);
  int16_t supplyValveCurrentPossition = supplyValve.readCurrentPosition(0x01);
  if (supplyValveCurrentPossition < -110 || supplyValveCurrentPossition > 100)
  {
    supplyValve.setPosition(0x01, 0, 0);
  }
}
void closeSupplyValve()
{
  supplyValve.setPosition(0x01, -8000, 0);
  int16_t supplyValveCurrentPossition = supplyValve.readCurrentPosition(0x01);
  if (supplyValveCurrentPossition < -8500 || supplyValveCurrentPossition > -7500)
  {
    supplyValve.setPosition(0x01, -8000, 0);
  }
}

B3MSC1170A mainValve;
void openMainValve()
{
  mainValve.setPosition(0x02, -5500, 0);
  int16_t mainValveCurrentPossition = mainValve.readCurrentPosition(0x02);
  if (mainValveCurrentPossition < -5000 || mainValveCurrentPossition > -4500)
  {
    mainValve.setPosition(0x02, -5500, 0);
  }
}
void closeMainValve()
{
  mainValve.setPosition(0x02, 0, 0);
  int16_t mainValveCurrentPossition = mainValve.readCurrentPosition(0x02);
  if (mainValveCurrentPossition < -100 || mainValveCurrentPossition > 100)
  {
    mainValve.setPosition(0x02, 0, 0);
  }
}
void closeMainValveToFlight()
{
  mainValve.setPosition(0x02, 0, 1000);
  int16_t mainValveCurrentPossition = mainValve.readCurrentPosition(0x02);
  if (mainValveCurrentPossition < -100 || mainValveCurrentPossition > 100)
  {
    mainValve.setPosition(0x02, 0, 1000);
  }
}

Var::ValveMode currentValveMode = Var::ValveMode::LAUNCH;
Var::GseSignal currentGseSignal = Var::GseSignal::IGNITION_ON;

GseSignal gseValve(6);   // OK
GseSignal gseIgniter(7); // OK

constexpr uint16_t MODE_CHANGING_THRESHOLD = 0;
CountDetector valveSignalCounter(MODE_CHANGING_THRESHOLD);
CountDetector igniterSignalCounter(MODE_CHANGING_THRESHOLD);

uint32_t retryCount = 0;
uint32_t lastSendTime = 0;
bool isVerified = true;

// float motorTemperature, mcuTemperature, current, inputVoltage;
// float currentPosition, currentDesiredPosition, currentVelocity;
// float currentSupplyPosition, voltage;

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
    bool isClosed = (mainValve.readDesiredPosition(0x02) > -1000) && (mainValve.readDesiredPosition(0x02) < 1000);

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
    bool isOpen = (mainValve.readDesiredPosition(0x02) > -7500) && (mainValve.readDesiredPosition(0x02) < -5500);

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
    // addTaskIfNotExisted("delayed-open-main-valve", &openMainValve);
    // Tasks["delayed-open-main-valve"]->startOnceAfterSec(1.0);
    openMainValve();
    Serial.println("Valve mode changed to LAUNCH");
    Status.noticedRed();
  }

  if (nextMode == Var::ValveMode::WAITING)
  {
    closeMainValve();
    openSupplyValve();
    Serial.println("Valve mode changed to WAITING");
    Status.noticedGreen();
  }

  currentValveMode = nextMode;
}

void changeIgnition(Var::GseSignal nextMode)
{
  if (nextMode == currentGseSignal)
    return;
  if (nextMode == Var::GseSignal::IGNITION_ON)
  {
    Serial.println("Ignition ON");
  }

  if (nextMode == Var::GseSignal::IGNITION_OFF)
  {
    Serial.println("Ignition OFF");
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
      bool newDoLogging;
      can.receiveFlight(&flightMode, &flightTime, &newDoLogging, &ident);
      digitalWrite(blueled, !digitalRead(blueled));

      if (doLogging != newDoLogging)
      {
        doLogging = newDoLogging;

        if (doLogging)
        {
          logger.reset();
        }
      }

      switch (flightMode)
      {
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
        // verifyValve();
        digitalWrite(redled, LOW);
        digitalWrite(greenled, LOW);
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
        // verifyValve();

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
        closeSupplyValve();
        closeMainValve();
        Serial.println("MAIN_CHUTE_DESCENT");
        break;
      }

      case (7): // LANDED
      {
        Serial.println("LANDED");
        break;
      }

      case (8): // SHUTDOWN
      {
        mainValve.torqueOff(0x01);
        Status.noticedBlue();
        Serial.println("SHUTDOWN");
        break;
      }
      }
    }
    break;
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

// void task10Hz()
// {
//   motorTemperature = mainValve.readMotorTemperature(0x02);
//   mcuTemperature = mainValve.readMcuTemperature(0x02);
//   current = mainValve.readCurrent(0x02);
//   inputVoltage = mainValve.readVoltage(0x02);
//   currentPosition = mainValve.readCurrentPosition(0x02);
//   currentDesiredPosition = mainValve.readDesiredPosition(0x02);
//   currentVelocity = mainValve.readCurrentVelosity(0x02);

//   const auto &logPacket = MsgPacketizer::encode(0x0A,
//                                                 millis(),
//                                                 motorTemperature,
//                                                 mcuTemperature,
//                                                 current,
//                                                 inputVoltage,
//                                                 currentPosition,
//                                                 currentDesiredPosition,
//                                                 currentVelocity);

//   if (doLogging)
//   {
//     // Serial.println("doLogging is TRUE, writing log...");
//     logger.write(logPacket.data.data(), logPacket.data.size());
//     // Serial.print("First byte: ");
//     // Serial.println(logPacket.data[0], HEX);
//     // Serial.print("Packet size: ");
//     // Serial.println(logPacket.data.size());
//     digitalWrite(redled, LOW);
//   }
//   ////////////////////////////////////////////////////
//   // Serial.print("VALVE: ");
//   // Serial.println(digitalRead(6) ? "ON" : "OFF");
//   // Serial.print("IGN: ");
//   // Serial.println(digitalRead(7) ? "ON" : "OFF");
//   ////////////////////////////////////////////////////
//   digitalWrite(redled, HIGH);
//   digitalWrite(greenled, HIGH);
// }

void sendValveData()
{
  digitalWrite(redled, HIGH);
  can.sendValveDataPart1(
      mainValve.readMotorTemperature(0x02),
      mainValve.readMcuTemperature(0x02),
      mainValve.readCurrent(0x02),
      mainValve.readVoltage(0x02));

  can.sendValveDataPart2(
      mainValve.readCurrentPosition(0x02),
      mainValve.readDesiredPosition(0x02),
      mainValve.readCurrentVelosity(0x02));

  can.sendValveDataPart3(
      supplyValve.readMotorTemperature(0x01),
      supplyValve.readMcuTemperature(0x01),
      supplyValve.readCurrent(0x01),
      supplyValve.readVoltage(0x01));

  can.sendValveDataPart4(
      supplyValve.readCurrentPosition(0x01),
      supplyValve.readDesiredPosition(0x01),
      supplyValve.readCurrentVelosity(0x01));
}

void setup()
{
  Serial.begin(115200);
  // Serial1.begin(115200, SERIAL_8N1);
  Serial.begin(115200, SERIAL_8N1);

  SPI.begin();
  can.begin();

  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(blueled, OUTPUT);

  digitalWrite(redled, HIGH);   // RGB LED OFF
  digitalWrite(greenled, HIGH); // RGB LED OFF
  digitalWrite(blueled, HIGH);  // RGB LED OFF

  Status.init(11);

  // logger.reset();

  supplyValve.initialize(0x01);
  mainValve.initialize(0x02);

  changeMode(Var::ValveMode::WAITING);
  changeIgnition(Var::GseSignal::IGNITION_OFF);

  Tasks.add(&syncFlightMode)->startFps(200);
  Tasks.add(&sendValveMode)->startFps(20);
  Tasks.add(&sendIgnition)->startFps(20);
  Tasks.add(&sendValveData)->startFps(10);
  // Tasks.add(&task10Hz)->startFps(10);

  // Tasks.add("waterProofOn", [&]()
  //           { closeMainValve(); });
  // Tasks.add(&changeMode)->startFps(10);
  // Tasks.add(&readData)->startFps(5);
}

void loop()
{
  Tasks.update();
  // logger.dump();
}

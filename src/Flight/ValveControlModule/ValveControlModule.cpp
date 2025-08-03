
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
#include "Lib_Logger1.hpp"
#include "Lib_Neopixel.hpp"
#include "Lib_OutputPin.hpp"

char ident = '\0';
bool doLogging = false;
uint8_t flightMode = 0;
uint32_t flightTime = 0;

Neopixel Status(12); // OK

CAN can(26);
Logger logger(28);

OutputPin servoEn(29);
OutputPin redLed(17);
OutputPin greenLed(16);
OutputPin blueLed(25);

Var::ValveMode currentValveMode = Var::ValveMode::LAUNCH;
Var::GseSignal currentGseSignal = Var::GseSignal::IGNITION_ON;

GseSignal gseValve(6);   // OK
GseSignal gseIgniter(7); // OK

constexpr uint16_t MODE_CHANGING_THRESHOLD = 3;
CountDetector valveSignalCounter(MODE_CHANGING_THRESHOLD);
CountDetector igniterSignalCounter(MODE_CHANGING_THRESHOLD);

uint32_t retryCount = 0;
uint32_t lastSendTime = 0;
bool isVerified = true;

int16_t motorTemperature;
int16_t mcuTemperature;
int16_t current;
int16_t inputVoltage;
int16_t currentPosition;
int16_t currentDesiredPosition;
int16_t currentVelocity;

uint8_t servoId = 0;
int16_t servoAngle = 0;

B3MSC1170A supplyValve;
void openSupplyValve()
{
  supplyValve.setPosition(0x02, 0, 0);
  // int16_t supplyValveCurrentPossition = supplyValve.readCurrentPosition(0x02);
  // if (-200 < supplyValveCurrentPossition || supplyValveCurrentPossition < 200)
  // {
  //   supplyValve.setPosition(0x02, 0, 0);
  // }
}
void closeSupplyValve()
{
  supplyValve.setPosition(0x02, -9000, 0);
  // int16_t supplyValveCurrentPossition = supplyValve.readCurrentPosition(0x02);
  // if (-8500 < supplyValveCurrentPossition || supplyValveCurrentPossition < -9500)
  // {
  //   supplyValve.setPosition(0x02, -9000, 0);
  // }
}

B3MSC1170A mainValve;
void openMainValve()
{
  mainValve.setPosition(0x01, 0, 0);
  // int16_t mainValveCurrentPossition = mainValve.readCurrentPosition(0x01);
  // if (-200 < mainValveCurrentPossition || mainValveCurrentPossition < 200)
  // {
  //   mainValve.setPosition(0x01, 0, 0);
  // }
}
void closeMainValve()
{
  mainValve.setPosition(0x01, -7000, 0);
  // int16_t mainValveCurrentPossition = mainValve.readCurrentPosition(0x01);
  // if (-6800 < mainValveCurrentPossition || mainValveCurrentPossition < -7200)
  // {
  //   mainValve.setPosition(0x01, -7000, 0);
  // }
}

void closeMainValveToFlight()
{
  mainValve.setPosition(0x01, -7000, 0);
}

void valveInitialize()
{
  mainValve.initialize(0x01);
  supplyValve.initialize(0x02);
}

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
    addTaskIfNotExisted("delayed-open-main-valve", &openMainValve);
    Tasks["delayed-open-main-valve"]->startOnceAfterMsec(500.0);
    Status.noticedRed();
  }

  if (nextMode == Var::ValveMode::WAITING)
  {
    closeMainValve();
    addTaskIfNotExisted("delayed-open-supply-valve", &openSupplyValve);
    Tasks["delayed-open-supply-valve"]->startOnceAfterMsec(500.0);
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
    Status.noticedPink();
  }

  if (nextMode == Var::GseSignal::IGNITION_OFF)
  {
    Status.noticedWhite();
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
      blueLed.toggle();
      bool newDoLogging;
      can.receiveFlight(&flightMode, &flightTime, &newDoLogging, &ident);
      Serial.print(">flightMode: ");
      Serial.println(flightMode);

      switch (flightMode)
      {
      case (0): // STANDBY
      {
        changeMode(Var::ValveMode::WAITING);

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

        servoEn.high();
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
        addTaskIfNotExisted("flight-close-main-valve", &closeMainValveToFlight);
        Tasks["flight-close-main-valve"]->startOnceAfterSec(1.0);
        closeSupplyValve();
        Serial.println("DROGUE_CHUTE_DESCENT");
        break;
      }

      case (6): // MAIN_CHUTE_DESCENT
      {
        closeSupplyValve();
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
        supplyValve.torqueOff(0x01);
        mainValve.torqueOff(0x02);
        servoEn.low();
        Status.noticedBlue();
        Serial.println("SHUTDOWN");
        break;
      }
      }
      }
    }
    break;
    }

    case Var::Label::SERVO_COMMAND:
    {
      can.receiveServoCommand(&servoId, &servoAngle);
      if (servoId == 1) {
        mainValve.setPosition(servoId, servoAngle, 0);
      } else if (servoId == 2) {
        supplyValve.setPosition(servoId, servoAngle, 0);
      }
      break;
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

void task10Hz()
{
  /*
  motorTemperature = mainValve.readMotorTemperature(0x02);
  mcuTemperature = mainValve.readMcuTemperature(0x02);
  current = mainValve.readCurrent(0x02);
  inputVoltage = mainValve.readVoltage(0x02);
  currentPosition = mainValve.readCurrentPosition(0x02);
  currentDesiredPosition = mainValve.readDesiredPosition(0x02);
  currentVelocity = mainValve.readCurrentVelosity(0x02);

  const auto &logPacket = MsgPacketizer::encode(0x0A,
                                                millis(),
                                                motorTemperature,
                                                mcuTemperature,
                                                current,
                                                inputVoltage,
                                                currentPosition,
                                                currentDesiredPosition,
                                                currentVelocity);

  if (doLogging)
  {
    // Serial.println("doLogging is TRUE, writing log...");
    logger.write(logPacket.data.data(), logPacket.data.size());
    // Serial.print("First byte: ");
    // Serial.println(logPacket.data[0], HEX);
    // Serial.print("Packet size: ");
    // Serial.println(logPacket.data.size());
  }
  */

  //////////////////////////////////////////////////
  Serial.print(">VALVE: ");
  Serial.println(gseValve.isSignaled());
  Serial.print(">IGN: ");
  Serial.println(gseIgniter.isSignaled());
  //////////////////////////////////////////////////
}

void sendValveData()
{
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
  servoEn.high();
  Serial.begin(115200);
  // Serial1.begin(115200, SERIAL_8N1);
  Serial.begin(115200, SERIAL_8N1);

  SPI.begin();
  can.begin();

  Status.init(11);
  redLed.high();
  greenLed.high();
  blueLed.high();

  supplyValve.initialize(0x01);
  mainValve.initialize(0x02);

  changeMode(Var::ValveMode::WAITING);
  changeIgnition(Var::GseSignal::IGNITION_OFF);

  Tasks.add(&syncFlightMode)->startFps(200);
  Tasks.add(&sendValveMode)->startFps(20);
  Tasks.add(&sendIgnition)->startFps(20);
  Tasks.add(&sendValveData)->startFps(20);
  // Tasks.add(&task10Hz)->startFps(10);
}

void loop()
{
  Tasks.update();
}

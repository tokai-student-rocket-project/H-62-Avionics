#include <Arduino.h>
#include <TaskManager.h>
#include <MsgPacketizer.h>
#include "Lib_Var.hpp"
#include "Lib_CAN.hpp"
#include "Lib_Telemeter.hpp"
#include "Lib_FlightMode.hpp"
#include "Lib_FlightTime.hpp"
#include "Lib_Logger3.hpp"
#include "Lib_FlightPin.hpp"
#include "Lib_Buzzer.hpp"
#include "Lib_Shiranui.hpp"
#include "Lib_GNSS.hpp"

char ident = 0;

CAN can(6);
Telemeter telemeter;
Logger logger(7, A1, A2);

OutputPin ledWork(LED_BUILTIN);
OutputPin ledCanRx(0);
OutputPin ledCanTx(1);
OutputPin ledLoRaRx(A5);
OutputPin ledLoRaTx(A6);
OutputPin ledGnssRx(5);

FlightMode flightMode;
FlightTime flightTime;

FlightPin flightPin(2);
Buzzer buzzer(A3, "buzzer");
Shiranui sn3(3, "sn3");
Shiranui sn4(4, "sn4");
GNSS gnss;

bool isSeparation1Announced = false;
bool isSeparation2Announced = false;

bool doLogging = false;

bool isFalling;
float altitude;
bool isLaunchMode;
bool isIgnition;
float forceX_N;
float jerkX_mps3;

bool gnssIsAvailable;
uint32_t unixEpoch;
bool isFixed;
uint8_t fixType;
uint8_t satelliteCount;
float latitude;
float longitude;
float height;
float speed;
float accuracy;

float motorTemperature, mcuTemperature, current, inputVoltage;
float currentPosition, currentDesiredPosition, currentVelocity;
float motorTemperature_SUPPLY, mcuTemperature_SUPPLY, current_SUPPLY, inputVoltage_SUPPLY;
float currentPosition_SUPPLY, currentDesiredPosition_SUPPLY, currentVelocity_SUPPLY;

bool sensingModuleAvailable = false;
bool sensingModuleAvailableAnnounced = false;

uint8_t receiveCommand = 0;
bool isInRecoveryMode = false;

void endRecoveryMode()
{
  isInRecoveryMode = false;
  buzzer.beepLongOnce();
  // Serial.println("Recovery mode finished.");
}

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
  isSeparation1Announced = false;
  isSeparation2Announced = false;
  buzzer.beepLongOnce();
}

void flightModeReset()
{
  if (flightMode.is(Var::FlightMode::STANDBY))
  {
    return;
  }

  if (flightMode.is(Var::FlightMode::DATA_PROTECTION))
  {
    return;
  }

  flightMode.change(Var::FlightMode::STANDBY);
  logger.reset();
  ident = static_cast<char>(random(65, 91));
  buzzer.beepLongOnce();
}

void recoveryMode()
{
  if (isInRecoveryMode)
  {
    return; // Already in recovery mode
  }
  isInRecoveryMode = true;
  buzzer.beepOnce();

  uint8_t command = 76;
  for (int i = 0; i < 50; i++)
  {
    can.sendServoCommand(command);
  }
  // Serial.println("Recovery mode started for 5 seconds.");

  // Schedule endRecoveryMode to be called after 5 seconds
  Tasks["end-recovery"]->startOnceAfterMsec(5000);
}

void task100Hz()
{
  // gnss
  gnssIsAvailable = gnss.available();
  if (gnssIsAvailable)
  {
    ledGnssRx.toggle();
    unixEpoch = gnss.getUnixEpoch();
    isFixed = gnss.isFixed();
    fixType = gnss.getFixType();
    satelliteCount = gnss.getSatelliteCount();
    latitude = gnss.getLatitude();
    longitude = gnss.getLongitude();
    height = gnss.getAltitude();
    speed = gnss.getSpeed();
    accuracy = gnss.getAccuracy();

    ledGnssRx.set(fixType == 3);
  }

  if (flightMode.is(Var::FlightMode::DATA_PROTECTION))
  {
    return;
  }

  // reset
  if (flightMode.isNot(Var::FlightMode::STANDBY) && flightMode.isNot(Var::FlightMode::READY_TO_FLY) && flightPin.isClosed())
  {
    flightModeReset();
    Serial.println("REST");
  }

  // announce force sep1
  if (!isSeparation1Announced && flightMode.isBetween(Var::FlightMode::POWERED_CLIMB, Var::FlightMode::FREE_DESCENT) && flightTime.isElapsed(flightTime.SEPARATION_1_FORCE_TIME - 300))
  {
    isSeparation1Announced = true;
    buzzer.beepAttention();
  }

  // force sep1
  if (flightMode.isBetween(Var::FlightMode::POWERED_CLIMB, Var::FlightMode::FREE_DESCENT) && flightTime.isElapsed(flightTime.SEPARATION_1_FORCE_TIME))
  {
    flightMode.change(Var::FlightMode::DROGUE_CHUTE_DESCENT);
    sn3.separate();
    buzzer.beepTwice();
    Serial.println("SEP1(F)");
  }

  // announce force sep2
  if (!isSeparation2Announced && flightMode.is(Var::FlightMode::DROGUE_CHUTE_DESCENT) && flightTime.isElapsed(flightTime.SEPARATION_2_FORCE_TIME - 300))
  {
    isSeparation2Announced = true;
    buzzer.beepAttention();
  }

  // force sep2
  if (flightMode.is(Var::FlightMode::DROGUE_CHUTE_DESCENT) && flightTime.isElapsed(flightTime.SEPARATION_2_FORCE_TIME))
  {
    flightMode.change(Var::FlightMode::MAIN_CHUTE_DESCENT);
    sn4.separate();
    buzzer.beepTwice();
    Serial.println("SEP2(F)");
  }

  switch (flightMode.current())
  {
  case (Var::FlightMode::STANDBY):
  {
    if (flightPin.isOpen() || isLaunchMode || isIgnition)
    {
      flightModeOn();
      Serial.println("WKUP");
    }

    break;
  }

  case (Var::FlightMode::READY_TO_FLY):
  {
    if (flightPin.isOpen())
    {
      flightMode.change(Var::FlightMode::POWERED_CLIMB);
      flightTime.setZero();
      buzzer.beepOnce();
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
      sn3.separate();
      buzzer.beepTwice();
      Serial.println("SEP1");
    }

    break;
  }

  case (Var::FlightMode::DROGUE_CHUTE_DESCENT):
  {
    if (flightTime.isElapsed(flightTime.SEPARATION_2_PROTECTION_TIME) && altitude < 100)
    {
      flightMode.change(Var::FlightMode::MAIN_CHUTE_DESCENT);
      sn4.separate();
      buzzer.beepTwice();
      Serial.println("SEP2");
    }

    break;
  }

  case (Var::FlightMode::MAIN_CHUTE_DESCENT):
  {
    if (flightTime.isElapsed(flightTime.LANDING_TIME))
    {
      flightMode.change(Var::FlightMode::LANDED);
      buzzer.beepLongThreeTimes();
      Serial.println("LAND");
    }

    break;
  }

  case (Var::FlightMode::LANDED):
  {
    if (flightTime.isElapsed(flightTime.LANDING_TIME + 5000))
    {
      flightMode.change(Var::FlightMode::SHUTDOWN);
      // buzzer.beepEndless(); // 海なので動作させない
      Serial.println("SDWN");
    }

    break;
  }
  }

  bool newDoLogging = flightMode.isBetween(Var::FlightMode::READY_TO_FLY, Var::FlightMode::SHUTDOWN); // LANDED // Var::FlightMode::SHUTDOWN まででもいいかも

  if (newDoLogging != doLogging)
  {
    can.sendFlight(flightMode.currentNumber(), flightTime.get(), doLogging, ident);
    ledCanTx.toggle();
  }

  doLogging = newDoLogging;
  ledWork.set(doLogging);

  // フライトモードの表示
  // Serial.println(flightMode.currentNumber());

  //
  //   const auto &logPacket = MsgPacketizer::encode(0x0A,
  //                                                 ident, millis(), flightTime.get(), flightMode.currentNumber(), logger.getUsage(),
  //                                                 flightPin.isOpen(), buzzer.isOn(), sn3.isOn(), sn4.isOn(),
  //                                                 isFalling, altitude, isLaunchMode, forceX_N, jerkX_mps3,
  //                                                 gnssIsAvailable, unixEpoch, isFixed, fixType, satelliteCount, latitude, longitude, height, speed, accuracy,
  //                                                 motorTemperature, mcuTemperature, current, inputVoltage,
  //                                                 currentPosition, currentDesiredPosition, currentVelocity, currentSupplyPosition, voltage);
  //

  const auto &logPacket = MsgPacketizer::encode(0x0A,
                                                ident,
                                                millis(),
                                                flightTime.get(),
                                                flightMode.currentNumber(),
                                                logger.getUsage(),
                                                flightPin.isOpen(),
                                                buzzer.isOn(),
                                                sn3.isOn(),
                                                sn4.isOn(),
                                                isFalling,
                                                altitude,
                                                isLaunchMode,
                                                forceX_N,
                                                jerkX_mps3,
                                                gnssIsAvailable,
                                                unixEpoch,
                                                isFixed,
                                                fixType,
                                                satelliteCount,
                                                latitude,
                                                longitude,
                                                height,
                                                speed,
                                                accuracy,
                                                motorTemperature,
                                                mcuTemperature,
                                                current,
                                                inputVoltage,
                                                currentPosition,
                                                currentDesiredPosition,
                                                currentVelocity,
                                                motorTemperature_SUPPLY,
                                                mcuTemperature_SUPPLY,
                                                current_SUPPLY,
                                                inputVoltage_SUPPLY,
                                                currentPosition_SUPPLY,
                                                currentDesiredPosition_SUPPLY,
                                                currentVelocity_SUPPLY);

  if (doLogging)
  {
    logger.write(logPacket.data.data(), logPacket.data.size());
  }
}

void task10Hz()
{
  can.sendFlight(flightMode.currentNumber(), flightTime.get(), doLogging, ident);
  ledCanTx.toggle();

  // Serial.print(gnss.getLatitude(), 8);    // GNSSのテスト用
  // Serial.print(",");                      // GNSSのテスト用
  // Serial.println(gnss.getLongitude(), 8); // GNSSのテスト用
}

void task2Hz()
{
  const auto &telemetryPacket = MsgPacketizer::encode(0x0A,
                                                      static_cast<char>(ident),
                                                      static_cast<uint32_t>(millis()),
                                                      static_cast<uint8_t>(flightMode.currentNumber()),
                                                      static_cast<uint16_t>(flightMode.isBetween(Var::FlightMode::READY_TO_FLY, Var::FlightMode::LANDED) ? flightTime.get() : 0),
                                                      static_cast<uint8_t>(logger.getUsage()),
                                                      static_cast<bool>(doLogging),
                                                      static_cast<uint8_t>(logger.framNumber()),
                                                      static_cast<bool>(flightPin.isOpen()),
                                                      static_cast<bool>(sn3.isOn()),
                                                      static_cast<bool>(sn4.isOn()),
                                                      static_cast<bool>(isLaunchMode),
                                                      static_cast<bool>(isFalling),
                                                      static_cast<uint32_t>(unixEpoch),
                                                      static_cast<uint8_t>(fixType),
                                                      static_cast<uint8_t>(satelliteCount),
                                                      static_cast<float>(latitude),
                                                      static_cast<float>(longitude),
                                                      static_cast<int16_t>(height * 10),
                                                      static_cast<int16_t>(speed * 10),
                                                      static_cast<uint16_t>(accuracy * 10),
                                                      static_cast<int16_t>(motorTemperature * 100),
                                                      static_cast<int16_t>(mcuTemperature * 100),
                                                      static_cast<uint16_t>(inputVoltage * 100),
                                                      static_cast<int16_t>(current * 100),
                                                      static_cast<int16_t>(currentPosition * 100),
                                                      static_cast<int16_t>(currentDesiredPosition * 100),
                                                      static_cast<int16_t>(currentVelocity * 100),
                                                      static_cast<int16_t>(motorTemperature_SUPPLY * 100),
                                                      static_cast<int16_t>(mcuTemperature_SUPPLY * 100),
                                                      static_cast<int16_t>(inputVoltage_SUPPLY * 100),
                                                      static_cast<int16_t>(current_SUPPLY * 100),
                                                      static_cast<int16_t>(currentPosition_SUPPLY * 100),
                                                      static_cast<int16_t>(currentDesiredPosition_SUPPLY * 100),
                                                      static_cast<int16_t>(currentVelocity_SUPPLY * 100),
                                                      static_cast<uint16_t>(flightTime.SEPARATION_1_PROTECTION_TIME),
                                                      static_cast<uint16_t>(flightTime.SEPARATION_1_FORCE_TIME),
                                                      static_cast<uint16_t>(flightTime.SEPARATION_2_PROTECTION_TIME),
                                                      static_cast<uint16_t>(flightTime.SEPARATION_2_FORCE_TIME),
                                                      static_cast<uint16_t>(flightTime.LANDING_TIME));

  telemeter.reserveData(telemetryPacket.data.data(), telemetryPacket.data.size());
  telemeter.sendReservedData();
  ledLoRaTx.toggle();
}

void setTimer(uint32_t separation1ProtectionTime, uint32_t separation1ForceTime, uint32_t separation2ProtectionTime, uint32_t separation2ForceTime, uint32_t landingTime)
{
  flightTime.SEPARATION_1_PROTECTION_TIME = separation1ProtectionTime;
  flightTime.SEPARATION_1_FORCE_TIME = separation1ForceTime;
  flightTime.SEPARATION_2_PROTECTION_TIME = separation2ProtectionTime;
  flightTime.SEPARATION_2_FORCE_TIME = separation2ForceTime;
  flightTime.LANDING_TIME = landingTime;
}

void setup()
{
  randomSeed(analogRead(A4));
  ident = static_cast<char>(random(65, 91));

  Serial.begin(115200);
  SPI.begin();

  can.begin();
  telemeter.initialize(925.6E6, 500E3);

  gnss.begin();

  setTimer(
      23000, // SEPARATION_1_PROTECTION_TIME
      30000, // SEPARATION_1_FORCE_TIME
      40000, // SEPARATION_2_PROTECTION_TIME // 18382
      45000, // SEPARATION_2_FORCE_TIME // 19382
      50000  // LANDING_TIME
  );

  if (flightPin.isOpen())
  {
    flightMode.change(Var::FlightMode::DATA_PROTECTION);
    buzzer.beepWarning();
  }
  else
  {
    // buzzer.beepMorse("FM " + (String)ident);
  }

  Tasks.add(&task100Hz)->startFps(100);
  Tasks.add(&task10Hz)->startFps(10);
  Tasks.add(&task2Hz)->startFps(2);
  Tasks.add("end-recovery", &endRecoveryMode);

  MsgPacketizer::subscribe(LoRa, 0xCC, [](uint8_t payload)
                           {
    ledLoRaRx.toggle();
    recoveryMode(); });

  // フライトモードオン
  MsgPacketizer::subscribe(LoRa, 0xF1, [](uint8_t payload)
                           {
    ledLoRaRx.toggle();
    flightModeOn(); });

  // フライトモードリセット
  MsgPacketizer::subscribe(LoRa, 0xF2, [](uint8_t payload)
                           {
    ledLoRaRx.toggle();
    flightModeReset(); });

  // タイマー設定
  MsgPacketizer::subscribe(LoRa, 0xF3, [](uint16_t separation1ProtectionTime, uint16_t separation1ForceTime, uint16_t separation2ProtectionTime, uint16_t separation2ForceTime, uint16_t landingTime)
                           {
    ledLoRaRx.toggle();
    setTimer(
      separation1ProtectionTime,
      separation1ForceTime,
      separation2ProtectionTime,
      separation2ForceTime,
      landingTime
    ); });
}

void loop()
{
  if (sensingModuleAvailable && !sensingModuleAvailableAnnounced && millis() > 5000)
  {
    sensingModuleAvailableAnnounced = true;
    // buzzer.beepMorse("SM");
  }

  Tasks.update();

  if (LoRa.parsePacket())
  {
    MsgPacketizer::parse();
  }

  if (can.available())
  {
    // sensingModuleAvailable = true; // CAN 通信できているかの確認用 // モールス信号OK
    switch (can.getLatestLabel())
    {
    case Var::Label::TRAJECTORY_DATA:
    {
      can.receiveTrajectory(&isFalling, &altitude); // Altitude を送信していないのでいったんコメントアウト．
      ledCanRx.toggle();
      sensingModuleAvailable = true;

      break;
    }

    case Var::Label::VALVE_MODE:
    {
      can.receiveValveMode(&isLaunchMode);
      ledCanRx.toggle();

      break;
    }
    case Var::Label::GSE_SIGNAL:
    {
      can.receiveIgnition(&isIgnition);
      ledCanRx.toggle();

      break;
    }

    case Var::Label::DYNAMICS_DATA:
    {
      can.receiveDynamics(&forceX_N, &jerkX_mps3);
      ledCanRx.toggle();

      break;
    }

    case Var::Label::VALVE_DATA_PART_1:
    {
      can.receiveValveDataPart1(&motorTemperature, &mcuTemperature, &current, &inputVoltage);
      ledCanRx.toggle();

      break;
    }

    case Var::Label::VALVE_DATA_PART_2:
    {
      can.receiveValveDataPart2(&currentPosition, &currentDesiredPosition, &currentVelocity);
      ledCanRx.toggle();

      break;
    }

    case Var::Label::VALVE_DATA_PART_3:
    {
      can.receiveValveDataPart3(&motorTemperature_SUPPLY, &mcuTemperature_SUPPLY, &current_SUPPLY, &inputVoltage_SUPPLY);
      ledCanRx.toggle();

      break;
    }

    case Var::Label::VALVE_DATA_PART_4:
    {
      can.receiveValveDataPart4(&currentPosition_SUPPLY, &currentDesiredPosition_SUPPLY, &currentVelocity_SUPPLY);
      ledCanRx.toggle();

      break;
    }
    }
  }
}
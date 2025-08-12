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

// --- 定数の定義 ---

// Pin Configuration
constexpr uint8_t PINNEOPIXEL = 12;
constexpr uint8_t PIN_CAN_TX = 26; // Note: CAN_RX is likely configured in the library
constexpr uint8_t PIN_LOGGER_CS = 28;
constexpr uint8_t PIN_SERVO_EN = 29;
constexpr uint8_t PIN_LED_RED = 17;
constexpr uint8_t PIN_LED_GREEN = 16;
constexpr uint8_t PIN_LED_BLUE = 25;
constexpr uint8_t PIN_GSE_VALVE = 6;
constexpr uint8_t PIN_GSE_IGNITER = 7;

// B3M Servo Configuration
constexpr uint8_t B3M_ID_MAIN_VALVE = 0x01;
constexpr uint8_t B3M_ID_SUPPLY_VALVE = 0x02;

// Servo Positions (Unit: 0.01 degree)
constexpr int16_t VALVE_POSITION_OPEN = 0;
constexpr int16_t VALVE_POSITION_CLOSED_MAIN = -7000;
constexpr int16_t VALVE_POSITION_CLOSED_SUPPLY = -9000;

// Servo Position Tolerances for Verification
constexpr int16_t VALVE_POSITION_TOLERANCE = 500; // +/- 5 degrees

// Task & Verification Timings
constexpr uint16_t MODE_CHANGING_THRESHOLD = 3; // Counts for GSE signal detection
constexpr uint32_t VERIFY_RETRY_DELAY_MS = 50;
constexpr uint32_t VERIFY_INITIAL_DELAY_MS = 300;
constexpr uint8_t VERIFY_MAX_RETRIES = 5;
constexpr float VALVE_ACTION_DELAY_SEC = 0.5f;
constexpr float FLIGHT_CLOSE_DELAY_SEC = 1.0f;
constexpr float FLIGHT_OPEN_DELAY_SEC = 2.0f;

// --- グローバル変数 ---

char ident = '\0';
bool doLogging = false;
uint8_t flightMode = 0;
uint32_t flightTime = 0;

Neopixel Status(PINNEOPIXEL);
CAN can(PIN_CAN_TX);
Logger logger(PIN_LOGGER_CS);

OutputPin servoEn(PIN_SERVO_EN);
OutputPin redLed(PIN_LED_RED);
OutputPin greenLed(PIN_LED_GREEN);
OutputPin blueLed(PIN_LED_BLUE);

Var::ValveMode currentValveMode = Var::ValveMode::LAUNCH;
Var::GseSignal currentGseSignal = Var::GseSignal::IGNITION_ON;

GseSignal gseValve(PIN_GSE_VALVE);
GseSignal gseIgniter(PIN_GSE_IGNITER);

CountDetector valveSignalCounter(MODE_CHANGING_THRESHOLD);
CountDetector igniterSignalCounter(MODE_CHANGING_THRESHOLD);

uint32_t retryCount = 0;
uint32_t lastSendTime = 0;
bool isVerified = true;

uint8_t command = 0;

B3MSC1170A supplyValve;
B3MSC1170A mainValve;

// --- バルブ制御関数 ---

void openSupplyValve()
{
  supplyValve.setPosition(B3M_ID_SUPPLY_VALVE, VALVE_POSITION_OPEN, 0);
}
void closeSupplyValve()
{
  supplyValve.setPosition(B3M_ID_SUPPLY_VALVE, VALVE_POSITION_CLOSED_SUPPLY, 0);
}

void openMainValve()
{
  mainValve.setPosition(B3M_ID_MAIN_VALVE, VALVE_POSITION_OPEN, 0);
}
void closeMainValve()
{
  mainValve.setPosition(B3M_ID_MAIN_VALVE, VALVE_POSITION_CLOSED_MAIN, 0);
}

void closeMainValveToFlight()
{
  mainValve.setPosition(B3M_ID_MAIN_VALVE, VALVE_POSITION_CLOSED_MAIN, 0);
}

void valveInitialize()
{
  mainValve.initialize(B3M_ID_MAIN_VALVE);
  supplyValve.initialize(B3M_ID_SUPPLY_VALVE);
}

/**
 * @brief バルブが指示通りの位置にいるかを確認し、違っていれば再試行する
 * @details
 * changeMode()でバルブに指示を出した後、この関数が定期的に呼び出される。
 * VERIFY_MAX_RETRIES 回リトライしても目標位置に到達しない場合は、諦めて処理を中断する。
 * これは、サーボの故障や物理的な障害でプログラムが無限ループに陥るのを防ぐため。
 */
void verifyValve()
{
  if (isVerified)
  {
    return;
  }

  if (retryCount == 0 && (millis() - lastSendTime) < VERIFY_INITIAL_DELAY_MS)
  {
    return;
  }

  if (retryCount >= VERIFY_MAX_RETRIES)
  {
    return; // Max retries reached
  }

  if ((millis() - lastSendTime) < VERIFY_RETRY_DELAY_MS)
  {
    return;
  }

  if (currentValveMode == Var::ValveMode::WAITING)
  {
    int16_t desiredPos = mainValve.readDesiredPosition(B3M_ID_MAIN_VALVE);
    bool isClosed = abs(desiredPos - VALVE_POSITION_CLOSED_MAIN) < VALVE_POSITION_TOLERANCE;

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
  else if (currentValveMode == Var::ValveMode::LAUNCH)
  {
    int16_t desiredPos = mainValve.readDesiredPosition(B3M_ID_MAIN_VALVE);
    bool isOpen = abs(desiredPos - VALVE_POSITION_OPEN) < VALVE_POSITION_TOLERANCE;

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

// --- タスクとモード管理 ---

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
    Tasks["delayed-open-main-valve"]->startOnceAfterMsec(VALVE_ACTION_DELAY_SEC * 1000);
    Status.noticedRed();
  }
  else if (nextMode == Var::ValveMode::WAITING)
  {
    closeMainValve();
    addTaskIfNotExisted("delayed-open-supply-valve", &openSupplyValve);
    Tasks["delayed-open-supply-valve"]->startOnceAfterMsec(VALVE_ACTION_DELAY_SEC * 1000);
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
  else if (nextMode == Var::GseSignal::IGNITION_OFF)
  {
    Status.noticedWhite();
  }

  currentGseSignal = nextMode;
}

void handleGseInputs()
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
}

void syncFlightMode()
{
}

// --- CAN通信 送信関数 ---

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
  // Main Valve Data
  can.sendValveDataPart1(
      mainValve.readMotorTemperature(B3M_ID_MAIN_VALVE),
      mainValve.readMcuTemperature(B3M_ID_MAIN_VALVE),
      mainValve.readCurrent(B3M_ID_MAIN_VALVE),
      mainValve.readVoltage(B3M_ID_MAIN_VALVE));

  can.sendValveDataPart2(
      mainValve.readCurrentPosition(B3M_ID_MAIN_VALVE),
      mainValve.readDesiredPosition(B3M_ID_MAIN_VALVE),
      mainValve.readCurrentVelosity(B3M_ID_MAIN_VALVE));

  // Supply Valve Data
  can.sendValveDataPart3(
      supplyValve.readMotorTemperature(B3M_ID_SUPPLY_VALVE),
      supplyValve.readMcuTemperature(B3M_ID_SUPPLY_VALVE),
      supplyValve.readCurrent(B3M_ID_SUPPLY_VALVE),
      supplyValve.readVoltage(B3M_ID_SUPPLY_VALVE));

  can.sendValveDataPart4(
      supplyValve.readCurrentPosition(B3M_ID_SUPPLY_VALVE),
      supplyValve.readDesiredPosition(B3M_ID_SUPPLY_VALVE),
      supplyValve.readCurrentVelosity(B3M_ID_SUPPLY_VALVE));
}

// --- 初期化とメインループ ---

void setup()
{
  servoEn.high();
  Serial.begin(115200);
  SPI.begin();
  can.begin();

  Status.init(11);
  redLed.high();
  greenLed.high();
  blueLed.high();

  valveInitialize();

  changeMode(Var::ValveMode::WAITING);
  changeIgnition(Var::GseSignal::IGNITION_OFF);

  Tasks.add(&syncFlightMode)->startFps(200); // 使っていないから削除でよい
  Tasks.add(&sendValveMode)->startFps(20);
  Tasks.add(&sendIgnition)->startFps(20);
  Tasks.add(&sendValveData)->startFps(20);
}

void loop()
{
  Tasks.update();

  if (!can.available())
  {
    return;
  }

  switch (can.getLatestLabel())
  {
  case Var::Label::FLIGHT_DATA:
  {
    blueLed.toggle();
    bool newDoLogging;
    can.receiveFlight(&flightMode, &flightTime, &newDoLogging, &ident);

    switch (flightMode)
    {
    case static_cast<uint8_t>(Var::FlightMode::STANDBY):
      handleGseInputs();
      servoEn.high();
      verifyValve();
      valveInitialize();
      break;

    case static_cast<uint8_t>(Var::FlightMode::READY_TO_FLY):
      handleGseInputs();
      break;

    case static_cast<uint8_t>(Var::FlightMode::POWERED_CLIMB):
    case static_cast<uint8_t>(Var::FlightMode::FREE_CLIMB):
    case static_cast<uint8_t>(Var::FlightMode::FREE_DESCENT):
      changeMode(Var::ValveMode::LAUNCH);
      break;

    case static_cast<uint8_t>(Var::FlightMode::DROGUE_CHUTE_DESCENT):
      addTaskIfNotExisted("flight-close-main-valve", &closeMainValveToFlight);
      addTaskIfNotExisted("flight-open-supply-valve", &openSupplyValve);
      Tasks["flight-close-main-valve"]->startOnceAfterSec(FLIGHT_CLOSE_DELAY_SEC);
      Tasks["flight-open-supply-valve"]->startOnceAfterSec(FLIGHT_OPEN_DELAY_SEC);
      // closeSupplyValve();　// 上空で供給路バルブを開放するためコメントアウト
      break;

    case static_cast<uint8_t>(Var::FlightMode::MAIN_CHUTE_DESCENT):
      // closeSupplyValve();　// 上空で供給路バルブを開放するためコメントアウト
      break;

    case static_cast<uint8_t>(Var::FlightMode::LANDED):
      mainValve.torqueOff(B3M_ID_MAIN_VALVE);
      supplyValve.torqueOff(B3M_ID_SUPPLY_VALVE);
      servoEn.low();
      break;

    case static_cast<uint8_t>(Var::FlightMode::SHUTDOWN):
      Status.off();
      break;
    }
    break; // End of FLIGHT_DATA case
  }
  case Var::Label::VALVE_MODE:
  {
    can.receiveServoCommand(&command);
    if (command == 76) // Command for manual valve operation
    {
      openSupplyValve();
      Status.noticedBlue();
    }
    break;
  }
  }
}
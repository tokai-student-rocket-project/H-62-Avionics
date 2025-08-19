/**
 * @file FlightModule.cpp
 * @brief 地上局（退避所）用のフライトモジュールメインプログラム。
 * @details
 * T-Beamデバイス上で動作し、LoRa経由で受信したテレメトリーデータをOLEDディスプレイに表示し、シリアルポートにCSV形式で出力する。
 * 指定はないが、TeraTermを用いることで自動ログ保存が可能。
 * ボタン操作により表示ページの切り替えや、分離時間設定のLoRaコマンド送信が可能。
 * microUSBポートから給電が可能。
 * @author H-62 Avionics Team
 * @date 2025-08-20
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>
#include <ArduinoJson.h>
#include "LoRaBoards.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Lib_Var.hpp>

#define BUTTON_PIN 38                          ///< ボタンが接続されているGPIOピン
const unsigned long LONG_PRESS_TIME_MS = 1000; ///< ボタン長押しと判定する時間 (ミリ秒)

Adafruit_SSD1306 display(128, 64, &Wire, -1); ///< OLEDディスプレイ制御オブジェクト
int currentPage = 0;                          ///< 現在表示中のページ番号
const int NUM_PAGES = 5;                      ///< ディスプレイの総ページ数

// --- 分離時間設定 ---
uint32_t separation1ProtectionTime = 22390; // (- 1.0 s) ///< 第1分離保護時間 (ms)
uint32_t separation1ForceTime = 25390;      // (+ 2.0 s) ///< 第1分離強制時間 (ms)
uint32_t separation2ProtectionTime = 84290; // (- 1.0 s) ///< 第2分離保護時間 (ms)
uint32_t separation2ForceTime = 85290;      // (± 0 s) ///< 第2分離強制時間 (ms)
uint32_t landingTime = 88890;               ///< 着地予定時間 (ms)

// --- テレメトリーデータ ---
float telemetryRssi = 0.0;                          ///< LoRa RSSI値
float telemetrySnr = 0.0;                           ///< LoRa SNR値
char telemetryIdent = ' ';                          ///< テレメトリー識別子
uint32_t telemetryMillis = 0;                       ///< テレメトリー受信時のミリ秒
uint8_t telemetryFlightMode = 0;                    ///< フライトモード
uint32_t telemetryFlightTime = 0;                   ///< フライト時間 (ms)
uint8_t telemetryLoggerUsage = 0;                   ///< ロガー使用率 (%)
bool telemetryDoLogging = false;                    ///< ロギング中か
uint8_t telemetryFramNumber = 0;                    ///< FRAM番号
bool telemetryFlightPinIsOpen = false;              ///< フライトピンが抜けているか
bool telemetrySn3IsOn = false;                      ///< 不知火Ⅲ がONか
bool telemetrySn4IsOn = false;                      ///< 不知火Ⅳ がONか
bool telemetryIsLaunchMode = false;                 ///< ローンチモードか
bool telemetryIsFalling = false;                    ///< 落下中か
uint32_t telemetryUnixEpoch = 0;                    ///< GPS Unix Epoch時間
uint8_t telemetryFixType = 0;                       ///< GPS Fixタイプ
uint8_t telemetrySatelliteCount = 0;                ///< GPS衛星数
float telemetryLatitude = 0.0;                      ///< GPS緯度
float telemetryLongitude = 0.0;                     ///< GPS経度
int16_t telemetryHeight = 0;                        ///< GPS高度 (m)
int16_t telemetrySpeed = 0;                         ///< GPS速度
uint16_t telemetryAccuracy = 0;                     ///< GPS精度
float telemetryMotorTemperature = 0.0;              ///< 主流路モーター温度
float telemetryMcuTemperature = 0.0;                ///< 主流路モーターMCU温度
float telemetryInputVoltage = 0.0;                  ///< 主流路モーター入力電圧
float telemetryCurrent = 0.0;                       ///< 主流路モーター電流
float telemetryCurrentPosition = 0.0;               ///< 主流路モーター現在位置
float telemetryCurrentDesiredPosition = 0.0;        ///< 主流路モーター目標位置
float telemetryCurrentVelocity = 0.0;               ///< 主流路モーター現在速度
float telemetryMotorTemperature_SUPPLY = 0.0;       ///< 供給路モーター温度
float telemetryMcuTemperature_SUPPLY = 0.0;         ///< 供給路モーターMCU温度
float telemetryInputVoltage_SUPPLY = 0.0;           ///< 供給路モーター入力電圧
float telemetryCurrent_SUPPLY = 0.0;                ///< 供給路モーター電流
float telemetryCurrentPosition_SUPPLY = 0.0;        ///< 供給路モーター現在位置
float telemetryCurrentDesiredPosition_SUPPLY = 0.0; ///< 供給路モーター目標位置
float telemetryCurrentVelocity_SUPPLY = 0.0;        ///< 供給路モーター現在速度
float telemetrySeparation1ProtectionTime = 0;       ///< 第1分離保護時間
float telemetrySeparation1ForceTime = 0;            ///< 第1分離強制時間
float telemetrySeparation2ProtectionTime = 0;       ///< 第2分離保護時間
float telemetrySeparation2ForceTime = 0;            ///< 第2分離強制時間
float telemetryLandingTime = 0;                     ///< 着地時間

unsigned long buttonPressTime = 0; ///< ボタンが押され始めた時刻
bool longPressSent = false;        ///< 長押しコマンドが送信済みかどうかのフラグ

void updateDisplay();
void sendLoRaCommand();
void loraRssiBar();

/**
 * @brief フライトモードの数値から対応する文字列を取得する。
 * @param[in] mode フライトモードを示す数値 (Var::FlightMode)。
 * @return const char* フライトモードを表す文字列。不明なモードの場合は "UNKNOWN" を返す。
 * @see Var::FlightMode
 */
const char *getFlightModeString(uint8_t mode)
{
    switch ((Var::FlightMode)mode)
    {
    case Var::FlightMode::STANDBY:
        return "STANDBY";
    case Var::FlightMode::READY_TO_FLY:
        return "READY";
    case Var::FlightMode::POWERED_CLIMB:
        return "POWER_CLIMB";
    case Var::FlightMode::FREE_CLIMB:
        return "FREE_CLIMB";
    case Var::FlightMode::FREE_DESCENT:
        return "FREE_DESCENT";
    case Var::FlightMode::DROGUE_CHUTE_DESCENT:
        return "DROGUE";
    case Var::FlightMode::MAIN_CHUTE_DESCENT:
        return "MAIN";
    case Var::FlightMode::LANDED:
        return "LANDED";
    case Var::FlightMode::SHUTDOWN:
        return "SHUTDOWN";
    case Var::FlightMode::DATA_PROTECTION:
        return "PROTECTION";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief 受信したRSSI値に基づいてアンテナピクトグラムを描画する。
 * @note 画面右下に描画される。
 */
void loraRssiBar()
{
    int rssi = LoRa.packetRssi();
    int bars = 0;
    if (rssi > -95.8)
        bars = 5;
    else if (rssi > -101.6)
        bars = 4;
    else if (rssi > -107.4)
        bars = 3;
    else if (rssi > -113.2)
        bars = 2;
    else
        bars = 1;

    int barWidth = 5;
    int barSpacing = 1;
    int barHeight = 8;
    int startX = 128 - (5 * (barWidth + barSpacing)); // Position to the far right
    int startY = 64 - barHeight;                      // Position to the bottom

    for (int i = 0; i < 5; i++)
    {
        int currentBarHeight = barHeight * (i + 1) / 5;
        if (i < bars)
        {
            display.fillRect(startX + i * (barWidth + barSpacing), startY + (barHeight - currentBarHeight), barWidth, currentBarHeight, SSD1306_WHITE);
        }
    }
}

/**
 * @brief 各表示ページのヘッダーを描画する。
 * @param[in] title ページのタイトル文字列。
 */
void displayHeader(const char *title)
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("P"));
    display.print(currentPage);
    display.print(F(":"));
    display.print(title);
    loraRssiBar();
}

/**
 * @brief ページ0 (ステータス情報) を表示する。
 */
void displayPage0()
{
    displayHeader(" Status -");
    display.setCursor(0, 10);
    display.print(F("RSSI: "));
    display.print(telemetryRssi);
    display.println(F(" dBm"));

    display.print(F("SNR : "));
    display.print(telemetrySnr);
    display.println(F(" dB"));

    display.print(F("MODE: "));
    display.println(getFlightModeString(telemetryFlightMode));

    display.print(F("TIME: "));
    display.print(telemetryFlightTime / 1000.0);
    display.println(F(" s"));

    display.print(F("LOG : "));
    display.print(telemetryDoLogging ? "ON" : "OFF");

    display.setCursor(54, 42);
    display.print(F("  SN3 : "));
    display.println(telemetrySn3IsOn ? "ON" : "OFF");

    display.print(F("MEM : "));
    display.print(telemetryLoggerUsage);
    display.print(F(" %"));

    display.print(F("  SN4 : "));
    display.println(telemetrySn4IsOn ? "ON" : "OFF");

    display.display();
}

/**
 * @brief ページ1 (GPS情報) を表示する。
 */
void displayPage1()
{
    displayHeader(" GPS -");
    display.setCursor(0, 10);
    display.print(F("LAT: "));
    display.println(telemetryLatitude, 6);

    display.print(F("LON: "));
    display.println(telemetryLongitude, 6);

    display.print(F("HGT: "));
    display.print(telemetryHeight);
    display.println(F(" m"));

    display.print(F("SAT: "));
    display.println(telemetrySatelliteCount);

    display.display();
}

/**
 * @brief ページ2 (主流路バルブ情報) を表示する。
 */
void displayPage2()
{
    displayHeader(" MAIN Valve -");
    display.setCursor(0, 10);
    display.print(F("TMP: "));
    display.print(telemetryMotorTemperature);
    display.println(F(" C"));
    display.print(F("VOL: "));
    display.print(telemetryInputVoltage);
    display.println(F(" V"));
    display.print(F("CUR: "));
    display.print(telemetryCurrent);
    display.println(F(" A"));
    display.print(F("POS: "));
    display.print(telemetryCurrentPosition);
    display.println(F(" deg"));
    display.display();
}

/**
 * @brief ページ3 (供給路バルブ情報) を表示する。
 */
void displayPage3()
{
    displayHeader(" SUPPLY Valve -");
    display.setCursor(0, 10);
    display.print(F("TMP: "));
    display.print(telemetryMotorTemperature_SUPPLY);
    display.println(F(" C"));
    display.print(F("VOL: "));
    display.print(telemetryInputVoltage_SUPPLY);
    display.println(F(" V"));
    display.print(F("CUR: "));
    display.print(telemetryCurrent_SUPPLY);
    display.println(F(" A"));
    display.print(F("POS: "));
    display.print(telemetryCurrentPosition_SUPPLY);
    display.println(F(" deg"));
    display.display();
}

/**
 * @brief ページ4 (分離時間情報) を表示する。
 */
void displayPage4()
{
    displayHeader(" SEPARATION -");
    display.setCursor(0, 10);
    display.print(F("S1P : "));
    display.print(telemetrySeparation1ProtectionTime);
    display.println(" sec");

    display.print(F("S1F : "));
    display.print(telemetrySeparation1ForceTime);
    display.println(" sec");

    display.print(F("S2P : "));
    display.print(telemetrySeparation2ProtectionTime);
    display.println(" sec");

    display.print(F("S2F : "));
    display.print(telemetrySeparation2ForceTime);
    display.println(" sec");

    display.print(F("LAND: "));
    display.print(telemetryLandingTime);
    display.println(" sec");

    display.display();
}

/**
 * @brief 現在のページ番号に応じて対応する表示関数を呼び出す。
 */
void updateDisplay()
{
    switch (currentPage)
    {
    case 0:
        displayPage0();
        break;
    case 1:
        displayPage1();
        break;
    case 2:
        displayPage2();
        break;
    case 3:
        displayPage3();
        break;
    case 4:
        displayPage4();
        break;
    }
}

/**
 * @brief 分離時間設定をLoRaコマンドとして送信する。
 */
void sendLoRaCommand()
{
    const auto &packet = MsgPacketizer::encode(0xF3, separation1ProtectionTime, separation1ForceTime, separation2ProtectionTime, separation2ForceTime, landingTime);
    LoRa.beginPacket();
    LoRa.write(packet.data.data(), packet.data.size());
    LoRa.endPacket();
}

/**
 * @brief ボタンの状態をチェックし、短押し・長押しに応じた処理を行うタスク。
 * @details 短押しで表示ページを切り替え、長押しでLoRaコマンドを送信する。
 */
void taskButtonCheck()
{
    bool buttonPressed = !digitalRead(BUTTON_PIN);

    if (buttonPressed)
    {
        if (buttonPressTime == 0)
        {
            buttonPressTime = millis();
            longPressSent = false;
        }
        else if ((millis() - buttonPressTime > LONG_PRESS_TIME_MS) && !longPressSent)
        {
            // 長押し時の動作
            sendLoRaCommand();
            longPressSent = true;
        }
    }
    else
    {
        if (buttonPressTime != 0 && !longPressSent)
        {
            // 短押し時の動作
            currentPage = (currentPage + 1) % NUM_PAGES;
            updateDisplay();
        }
        buttonPressTime = 0;
    }
}

/**
 * @brief 起動時に一度だけ実行されるセットアップ処理。
 * @details ハードウェアの初期化、シリアル通信の開始、LoRaの設定、タスクの登録などを行う。
 */
void setup()
{
    setupBoards();
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println();
    display.println("GROUND FLIGHT MODULE");
    display.println();
    display.println("--- System Start ---");
    display.println("====================");
    display.println("   H-62  Avionics   ");
    display.println("     SUBARU 1.3     ");
    display.println("====================");

    display.display();
    delay(3000);

    Serial.begin(115200);
    Serial.println("loraRssi,loraSnr,millis,flightMode,flightTime,loggerUsage,doLogging,framNumber,flightPinIsOpen,sn3IsOn,sn4IsOn,isLaunchMode,isFalling,unixEpoch,fixType,satelliteCount,latitude,longitude,height,speed,accuracy,motorTemperature,mcuTemperature,inputVoltage,current,currentPosition,currentDesiredPosition,currentVelocity,motorTemperature_SUPPLY,mcuTemperature_SUPPLY,inputVoltage_SUPPLY,current_SUPPLY,currentPosition_SUPPLY,currentDesiredPosition_SUPPLY,currentVelocity_SUPPLY,separation1ProtectionTime,separation1ForceTime,separation2ProtectionTime,separation2ForceTime,landingTime");

    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);

    if (!LoRa.begin(924.6E6)) // 21st無線調整シートより
    {
        Serial.println("Starting LoRa failed!");
        display.clearDisplay();
        display.println("LoRa Failed!");
        display.display();
        while (1)
            ;
    }
    LoRa.setSignalBandwidth(500E3);

    MsgPacketizer::subscribe(LoRa, 0x0A,

                             [](
                                 char ident,
                                 uint32_t millis,
                                 uint8_t flightMode,
                                 uint32_t flightTime,
                                 uint8_t loggerUsage,
                                 bool doLogging,
                                 uint8_t framNumber,
                                 bool flightPinIsOpen,
                                 bool sn3IsOn,
                                 bool sn4IsOn,
                                 bool isLaunchMode,
                                 bool isFalling,
                                 uint32_t unixEpoch,
                                 uint8_t fixType,
                                 uint8_t satelliteCount,
                                 float latitude,
                                 float longitude,
                                 int16_t height,
                                 int16_t speed,
                                 uint16_t accuracy,
                                 int16_t motorTemperature,
                                 int16_t mcuTemperature,
                                 uint16_t inputVoltage,
                                 int16_t current,
                                 int16_t currentPosition,
                                 int16_t currentDesiredPosition,
                                 int16_t currentVelocity,

                                 int16_t motorTemperature_SUPPLY,
                                 int16_t mcuTemperature_SUPPLY,
                                 uint16_t inputVoltage_SUPPLY,
                                 int16_t current_SUPPLY,
                                 int16_t currentPosition_SUPPLY,
                                 int16_t currentDesiredPosition_SUPPLY,
                                 int16_t currentVelocity_SUPPLY,
                                 uint32_t separation1ProtectionTime,
                                 uint32_t separation1ForceTime,
                                 uint32_t separation2ProtectionTime,
                                 uint32_t separation2ForceTime,
                                 uint32_t landingTime)
                             {
                                 telemetryRssi = LoRa.packetRssi();
                                 telemetrySnr = LoRa.packetSnr();
                                 telemetryIdent = ident;
                                 telemetryMillis = millis;
                                 telemetryFlightMode = flightMode;
                                 telemetryFlightTime = flightTime;
                                 telemetryLoggerUsage = loggerUsage;
                                 telemetryDoLogging = doLogging;
                                 telemetryFramNumber = framNumber;
                                 telemetryFlightPinIsOpen = flightPinIsOpen;
                                 telemetrySn3IsOn = sn3IsOn;
                                 telemetrySn4IsOn = sn4IsOn;
                                 telemetryIsLaunchMode = isLaunchMode;
                                 telemetryIsFalling = isFalling;
                                 telemetryUnixEpoch = unixEpoch;
                                 telemetryFixType = fixType;
                                 telemetrySatelliteCount = satelliteCount;
                                 telemetryLatitude = latitude;
                                 telemetryLongitude = longitude;
                                 telemetryHeight = height;
                                 telemetrySpeed = speed;
                                 telemetryAccuracy = accuracy;
                                 telemetryMotorTemperature = (float)motorTemperature / 100.0;
                                 telemetryMcuTemperature = (float)mcuTemperature / 100.0;
                                 telemetryInputVoltage = (float)inputVoltage / 100.0;
                                 telemetryCurrent = (float)current / 100.0;
                                 telemetryCurrentPosition = (float)currentPosition / 100.0;
                                 telemetryCurrentDesiredPosition = (float)currentDesiredPosition / 100.0;
                                 telemetryCurrentVelocity = (float)currentVelocity / 100.0;
                                 telemetryMotorTemperature_SUPPLY = (float)motorTemperature_SUPPLY / 100.0;
                                 telemetryMcuTemperature_SUPPLY = (float)mcuTemperature_SUPPLY / 100.0;
                                 telemetryInputVoltage_SUPPLY = (float)inputVoltage_SUPPLY / 100.0;
                                 telemetryCurrent_SUPPLY = (float)current_SUPPLY / 100.0;
                                 telemetryCurrentPosition_SUPPLY = (float)currentPosition_SUPPLY / 100.0;
                                 telemetryCurrentDesiredPosition_SUPPLY = (float)currentDesiredPosition_SUPPLY / 100.0;
                                 telemetryCurrentVelocity_SUPPLY = (float)currentVelocity_SUPPLY / 100.0;
                                 telemetrySeparation1ProtectionTime = separation1ProtectionTime / 1000.0;
                                 telemetrySeparation1ForceTime = separation1ForceTime / 1000.0;
                                 telemetrySeparation2ProtectionTime = separation2ProtectionTime / 1000.0;
                                 telemetrySeparation2ForceTime = separation2ForceTime / 1000.0;
                                 telemetryLandingTime = landingTime / 1000.0;

                                 updateDisplay();

                                 Serial.print(millis / 1000.0);
                                 Serial.print(",");
                                 Serial.print(ident);
                                 Serial.print(",");
                                 Serial.print(loggerUsage);
                                 Serial.print(",");
                                 Serial.print(doLogging);
                                 Serial.print(",");
                                 Serial.print(framNumber);
                                 Serial.print(",");
                                 Serial.print(flightMode);
                                 Serial.print(",");
                                 Serial.print(flightTime / 1000.0);
                                 Serial.print(",");
                                 Serial.print(telemetryRssi);
                                 Serial.print(",");
                                 Serial.print(telemetrySnr);
                                 Serial.print(",");
                                 Serial.print(flightPinIsOpen);
                                 Serial.print(",");
                                 Serial.print(sn3IsOn);
                                 Serial.print(",");
                                 Serial.print(sn4IsOn);
                                 Serial.print(",");
                                 Serial.print(isLaunchMode);
                                 Serial.print(",");
                                 Serial.print(isFalling);
                                 Serial.print(",");
                                 Serial.print(unixEpoch);
                                 Serial.print(",");
                                 Serial.print(fixType);
                                 Serial.print(",");
                                 Serial.print(satelliteCount);
                                 Serial.print(",");
                                 Serial.print(latitude, 6);
                                 Serial.print(",");
                                 Serial.print(longitude, 6);
                                 Serial.print(",");
                                 Serial.print(height);
                                 Serial.print(",");
                                 Serial.print(speed);
                                 Serial.print(",");
                                 Serial.print(accuracy);
                                 Serial.print(",");
                                 Serial.print(telemetryMotorTemperature);
                                 Serial.print(",");
                                 Serial.print(telemetryMcuTemperature);
                                 Serial.print(",");
                                 Serial.print(telemetryInputVoltage);
                                 Serial.print(",");
                                 Serial.print(telemetryCurrent);
                                 Serial.print(",");
                                 Serial.print(telemetryCurrentPosition);
                                 Serial.print(",");
                                 Serial.print(telemetryCurrentDesiredPosition);
                                 Serial.print(",");
                                 Serial.print(telemetryCurrentVelocity);
                                 Serial.print(",");
                                 Serial.print(telemetryMotorTemperature_SUPPLY);
                                 Serial.print(",");
                                 Serial.print(telemetryMcuTemperature_SUPPLY);
                                 Serial.print(",");
                                 Serial.print(telemetryInputVoltage_SUPPLY);
                                 Serial.print(",");
                                 Serial.print(telemetryCurrent_SUPPLY);
                                 Serial.print(",");
                                 Serial.print(telemetryCurrentPosition_SUPPLY);
                                 Serial.print(",");
                                 Serial.print(telemetryCurrentDesiredPosition_SUPPLY);
                                 Serial.print(",");
                                 Serial.print(telemetryCurrentVelocity_SUPPLY);
                                 Serial.print(",");
                                 Serial.print(separation1ProtectionTime);
                                 Serial.print(",");
                                 Serial.print(separation1ForceTime);
                                 Serial.print(",");
                                 Serial.print(separation2ProtectionTime);
                                 Serial.print(",");
                                 Serial.print(separation2ForceTime);
                                 Serial.print(",");
                                 Serial.println(landingTime);
                                 Serial.flush();
                             });

    updateDisplay();
    Tasks.add(&taskButtonCheck)->startFps(20);
    Tasks.add("update-display", &updateDisplay);
}

/**
 * @brief メインループ。継続的に実行される。
 * @details タスクマネージャの更新、シリアルコマンドの受信処理、LoRaパケットの受信処理を行う。
 */
void loop()
{
    Tasks.update();

    if (Serial.available())
    {
        char command = Serial.read();

        // F: フライトモードオン
        if (command == 'F')
        {
            const auto &packet = MsgPacketizer::encode(0xF1, (uint8_t)0);
            LoRa.beginPacket();
            LoRa.write(packet.data.data(), packet.data.size());
            LoRa.endPacket();
        }

        // R: フライトモードリセット
        if (command == 'R')
        {
            const auto &packet = MsgPacketizer::encode(0xF2, (uint8_t)0);
            LoRa.beginPacket();
            LoRa.write(packet.data.data(), packet.data.size());
            LoRa.endPacket();
        }

        // C: タイマー設定
        if (command == 'C')
        {
            const auto &packet = MsgPacketizer::encode(0xF3, separation1ProtectionTime, separation1ForceTime, separation2ProtectionTime, separation2ForceTime, landingTime);
            LoRa.beginPacket();
            LoRa.write(packet.data.data(), packet.data.size());
            LoRa.endPacket();
        }
    }

    if (LoRa.parsePacket())
    {
        MsgPacketizer::parse();
    }
}
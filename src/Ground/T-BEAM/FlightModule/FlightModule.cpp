// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <LoRa.h>
// #include <MsgPacketizer.h>
// #include <TaskManager.h>
// #include <ArduinoJson.h>
// #include "LoRaBoards.h"
// #include <TinyGPS++.h>
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_GFX.h>

// StaticJsonDocument<4096> packet;

// uint16_t separation1ProtectionTime = 8605;
// uint16_t separation1ForceTime = 11605;
// uint16_t separation2ProtectionTime = 1000;
// uint16_t separation2ForceTime = 1000;
// uint16_t landingTime = 30305;

// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define OLED_RESET -1    // Reset pin
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// float onbordLatitude;
// float onbordLongtitude;
// float onbordAltitude;
// float receiveLatitude;
// float receiveLongtitude;
// TinyGPSPlus onbordGps;

// void task5Hz(){

//    while (SerialGPS.available()) {
//     onbordGps.encode(SerialGPS.read());
//   }

//   // GPSデータが有効な場合にのみ表示
//   if (onbordGps.location.isValid()) {
//     Serial.print(F("Lat: "));
//     Serial.print(onbordGps.location.lat(), 6); // 緯度 (小数点以下6桁まで)
//     Serial.print(F(", Lng: "));
//     Serial.print(onbordGps.location.lng(), 6); // 経度 (小数点以下6桁まで)
//     Serial.print(F(", Alt: "));
//     Serial.print(onbordGps.altitude.meters(), 2); // 高度 (メートル、小数点以下2桁まで)
//     Serial.print(F("m"));

//     // 日付と時刻の表示
//     if (onbordGps.date.isValid() && onbordGps.time.isValid()) {
//       Serial.print(F(", Date: "));
//       Serial.print(onbordGps.date.year());
//       Serial.print(F("-"));
//       Serial.print(onbordGps.date.month(), DEC);
//       Serial.print(F("-"));
//       Serial.print(onbordGps.date.day(), DEC);
//       Serial.print(F(", Time: "));
//       Serial.print(onbordGps.time.hour(), DEC);
//       Serial.print(F(":"));
//       Serial.print(onbordGps.time.minute(), DEC);
//       Serial.print(F(":"));
//       Serial.print(onbordGps.time.second(), DEC);
//     }

//     Serial.print(F(", Sats: "));
//     Serial.print(onbordGps.satellites.value()); // 捕捉衛星数
//     Serial.print(F(", HDOP: "));
//     Serial.print(onbordGps.hdop.hdop(), 1);     // 水平精度低下率

//     Serial.println(); // 改行
//   } else {
//     // GPSデータがまだ有効でない場合
//     Serial.println(F("No valid GPS data yet."));
//   }

//     // ディスプレイ表示,追記箇所
//                              display.clearDisplay(); // 画面クリア
//                              display.setTextSize(1); // テキストサイズを1に設定 (すべての情報に適用)

//                             //  //1. GNSSにて測位した情報から現在の時間を表示
//                             //  display.setCursor(0, 0); // Y座標0
//                             //  display.print("TIME: ");
//                             //  if (onbordGps.time.isValid()) { // GPS時刻が有効な場合のみ表示
//                             //      if (onbordGps.time.hour() < 10) display.print("0"); // 1桁の場合はゼロ埋め
//                             //      display.print(onbordGps.time.hour());
//                             //      display.print(":");
//                             //      if (onbordGps.time.minute() < 10) display.print("0");
//                             //      display.print(onbordGps.time.minute());
//                             //      display.print(":");
//                             //      if (onbordGps.time.second() < 10) display.print("0");
//                             //      display.println(onbordGps.time.second());
//                             //  } else {
//                             //      display.println("--:--:--"); // 無効な場合はハイフン表示
//                             //  }

//                              //「すばる」からの測位データをT-Beamのディスプレイに映す．小数点以下6桁
//                              display.setCursor(0, 20); // Y座標20
//                              display.print(" H-62 Lat:"); display.println(receiveLatitude, 6); // 小数点以下6桁
//                              display.setCursor(0, 30); // Y座標30
//                              display.print(" H-62 Lon:"); display.println(receiveLongtitude, 6); // 小数点以下6桁

//                              //T-Beam内部のGPSモジュールからの測位データをディスプレイに映す．小数点以下6桁
//                              display.setCursor(0, 40); // Y座標40
//                              onbordLatitude = onbordGps.location.lat();
//                              display.print("TBEAM Lat:"); display.println(onbordLatitude, 6); // 小数点以下6桁
//                              display.setCursor(0, 50); // Y座標50(これがギリギリ128x64ディスプレイの最終行)
//                              onbordLongtitude= onbordGps.location.lng();
//                              display.print("TBEAM Lon:"); display.println(onbordLongtitude, 6); // 小数点以下6桁
//                              // 注: RX Altは画面スペースの制約上、このレイアウトでは表示できない→すばる側の高度を消した

//                              display.setCursor(70, 0); // 表示開始位置

//     int rssi = LoRa.packetRssi();
//     int bars = 0;
//     if (rssi > -95.8) {
//         bars = 5;
//     } else if (rssi > -101.6) {
//         bars = 4;
//     } else if (rssi > -107.4) {
//         bars = 3;
//     } else if (rssi > -113.2) {
//         bars = 2;
//     } else {
//         bars = 1;
//     }

//     int barWidth = 5;
//     int barSpacing = 1;
//     int startX = 100;
//     int startY = 0;
//     int barHeight = 8;

//     for (int i = 0; i < 5; i++) {
//         int currentBarHeight = barHeight * (i + 1) / 5;
//         if (i < bars) {
//             display.fillRect(startX + i * (barWidth + barSpacing), startY + (barHeight - currentBarHeight), barWidth, currentBarHeight, SSD1306_WHITE);
//         }
//     }

//     // display.setCursor(0, 0);
//     //  display.print(" HDOP:"); display.println(LoRa.packetRssi(), 0.5); // 小数点以下6桁

//                              display.display(); // 画面更新
// }

// void setup()
// {
//   setupBoards();
//   // Serial.begin(115200);

//   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 128x64用のアドレス0x3C (一般的なアドレス)
//     Serial.println(F("SSD1306 allocation failed"));
//     for(;;); // 初期化失敗したら停止
//   }
//   display.display();
//   delay(100); //初期表示を短くした
//   display.clearDisplay();
//   display.setTextColor(SSD1306_WHITE); //テキストの色を白に設定
//   display.setTextSize(1); //テキストサイズを1に設定

//   LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
//   //LoRa初期化の成功チェックを追加
//   if (!LoRa.begin(925.6E6)) {
//     Serial.println("LoRa initialization failed!");
//     display.println("LoRa Init Fail!"); // ここを変更した: OLEDにも表示
//     display.display();
//     while (1);
//   }
//   LoRa.setSignalBandwidth(500E3);
//   Serial.println("LoRa initialized."); //成功メッセージ
//   display.println("LoRa Init OK."); //OLEDにも表示
//   display.display();
//   delay(100); //初期化成功メッセージ表示

//   LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
//   LoRa.begin(925.6E6);
//   LoRa.setSignalBandwidth(500E3);

//   MsgPacketizer::subscribe(LoRa, 0x0A,

//                            [](
//                                char ident,
//                                uint32_t millis,
//                                uint8_t flightMode,
//                                uint16_t flightTime,
//                                uint8_t loggerUsage,
//                                bool doLogging,
//                                uint8_t framNumber,
//                                bool flightPinIsOpen,
//                                bool sn3IsOn,
//                                bool sn4IsOn,
//                                bool isLaunchMode,
//                                bool isFalling,
//                                uint32_t unixEpoch,
//                                uint8_t fixType,
//                                uint8_t satelliteCount,
//                                float latitude,
//                                float longitude,
//                                int16_t height,
//                                int16_t speed,
//                                uint16_t accuracy,
//                                int16_t motorTemperature,
//                                int16_t mcuTemperature,
//                                uint16_t inputVoltage,
//                                int16_t current,
//                                int16_t currentPosition,
//                                int16_t currentDesiredPosition,
//                                int16_t currentVelocity,

//                                int16_t motorTemperature_SUPPLY,
//                                int16_t mcuTemperature_SUPPLY,
//                                int16_t inputVoltage_SUPPLY,
//                                int16_t current_SUPPLY,
//                                int16_t currentPosition_SUPPLY,
//                                int16_t currentDesiredPosition_SUPPLY,
//                                int16_t currentVelocity_SUPPLY,
//                                uint16_t separation1ProtectionTime,
//                                uint16_t separation1ForceTime,
//                                uint16_t separation2ProtectionTime,
//                                uint16_t separation2ForceTime,
//                                uint16_t landingTime)
//                            {
//                              packet.clear();

//                              packet["packet"]["module"] = "F";
//                              packet["packet"]["rssi_dBm"] = LoRa.packetRssi();
//                              packet["packet"]["snr_dBm"] = LoRa.packetSnr();
//                              packet["packet"]["ident"] = (String)ident;
//                              packet["packet"]["uptime_s"] = (float)millis / 1000.0;

//                              packet["logger"]["doLogging"] = doLogging;
//                              packet["logger"]["usage"] = loggerUsage;
//                              packet["logger"]["number"] = framNumber;

//                              packet["flight"]["mode"] = flightMode;
//                              packet["flight"]["time_s"] = (float)flightTime / 1000.0;
//                              packet["flight"]["detection"]["valveModeIsLaunch"] = isLaunchMode;
//                              packet["flight"]["detection"]["flightPinIsOpen"] = flightPinIsOpen;
//                              packet["flight"]["detection"]["isFalling"] = isFalling;
//                              packet["flight"]["separation"]["sn3IsOn"] = sn3IsOn;
//                              packet["flight"]["separation"]["sn4IsOn"] = sn4IsOn;

//                              packet["gnss"]["unixEpoch"] = unixEpoch;
//                              packet["gnss"]["fixType"] = fixType;
//                              packet["gnss"]["satellites"] = satelliteCount;
//                              packet["gnss"]["latitude_deg"] = latitude;
//                              packet["gnss"]["Longittude_beg"] = longitude;
//                              packet["gnss"]["height_m"] = (float)height / 10.0;
//                              packet["gnss"]["speed_mps"] = (float)speed / 10.0;
//                              packet["gnss"]["accuracy_m"] = (float)accuracy / 10.0;

//                              packet["valve"]["motorTemperature_degC"] = (float)motorTemperature / 100.0;
//                              packet["valve"]["mcuTemperature_degC"] = (float)mcuTemperature / 100.0;
//                              packet["valve"]["inputVoltage_V"] = (float)inputVoltage / 100.0;
//                              packet["valve"]["current"] = (float)current / 100.0;
//                              packet["valve"]["currentPosition_deg"] = (float)currentPosition / 100.0;
//                              packet["valve"]["currentDesiredPosition_deg"] = (float)currentDesiredPosition / 100.0;
//                              packet["valve"]["currentVelocity_degps"] = (float)currentVelocity / 100.0;

//                              packet["valve"]["motorTemperatureSupply_degC"] = (float)motorTemperature_SUPPLY / 100.0;
//                              packet["valve"]["mcuTemperatureSupply_degC"] = (float)mcuTemperature_SUPPLY / 100.0;
//                              packet["valve"]["inputVoltageSupply_V"] = (float)inputVoltage_SUPPLY / 100.0;
//                              packet["valve"]["currentSupply_A"] = (float)current_SUPPLY / 100.0;
//                              packet["valve"]["currentPositionSupply_deg"] = (float)currentPosition_SUPPLY / 100.0;
//                              packet["valve"]["currentDesiredPositionSupply_deg"] = (float)currentDesiredPosition_SUPPLY / 100.0;
//                              packet["valve"]["currentVelocitySupply_deg"] = (float)currentVelocity_SUPPLY / 100.0;

//                              packet["timer"]["separation_1_protection_time"] = (float)separation1ProtectionTime / 1000.0;
//                              packet["timer"]["separation_1_force_time"] = (float)separation1ForceTime / 1000.0;
//                              packet["timer"]["separation_2_protection_time"] = (float)separation2ProtectionTime / 1000.0;
//                              packet["timer"]["separation_2_force_time"] = (float)separation2ForceTime / 1000.0;
//                              packet["timer"]["landing_time"] = (float)landingTime / 1000.0;

//                              serializeJson(packet, Serial);
//                              Serial.println();
//                              Serial.flush();

//                              //フライトタイムのディスプレイ表示
//                              display.clearDisplay(); // 画面クリア
//                              display.setTextSize(1); // テキストサイズを1に設定 (すべての情報に適用)

//                              display.setCursor(0, 10); // Y座標10
//                              display.print("Flight Time:"); display.println((float)flightTime / 1000.0);

//                            });

//     Tasks.add(&task5Hz)->startFps(5);
// }

// void loop()
// {
//   Tasks.update();

//   if (Serial.available())
//   {
//     char command = Serial.read();

//     // F: フライトモードオン
//     if (command == 'F')
//     {
//       const auto &packet = MsgPacketizer::encode(0xF1, (uint8_t)0);
//       LoRa.beginPacket();
//       LoRa.write(packet.data.data(), packet.data.size());
//       LoRa.endPacket();
//     }

//     // R: フライトモードリセット
//     if (command == 'R')
//     {
//       const auto &packet = MsgPacketizer::encode(0xF2, (uint8_t)0);
//       LoRa.beginPacket();
//       LoRa.write(packet.data.data(), packet.data.size());
//       LoRa.endPacket();
//     }

//     // C: タイマー設定
//     if (command == 'C')
//     {
//       const auto &packet = MsgPacketizer::encode(0xF3, separation1ProtectionTime, separation1ForceTime, separation2ProtectionTime, separation2ForceTime, landingTime);
//       LoRa.beginPacket();
//       LoRa.write(packet.data.data(), packet.data.size());
//       LoRa.endPacket();
//     }
//   }

//   if (LoRa.parsePacket())
//   {
//     MsgPacketizer::parse();
//   }

// }
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>
#include "LoRaBoards.h"
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BUTTON_PIN 38
const unsigned long LONG_PRESS_TIME_MS = 1000;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
int currentPage = 0;
const int NUM_PAGES = 2;

TinyGPSPlus onbordGps;

// テレメトリー Sensing
float lastRssi = 0.0;
float lastSnr = 0.0;
float altitude = 0.0;
float batteryVoltage = 0.0;
float externalVoltage = 0.0;
float receiveLatitude = 0.0;
float receiveLongtitude = 0.0;

// テレメトリー Flight
float mainPosition = 0.0;

unsigned long buttonPressTime = 0;
bool longPressSent = false;

void updateDisplay();
void sendLoRaCommand();

void displayPage0()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("--- GPS ---"));

  display.setCursor(70, 0);
  display.println();

  int rssi = LoRa.packetRssi();
  int bars = 0;
  if (rssi > -95.8)
  {
    bars = 5;
  }
  else if (rssi > -101.6)
  {
    bars = 4;
  }
  else if (rssi > -107.4)
  {
    bars = 3;
  }
  else if (rssi > -113.2)
  {
    bars = 2;
  }
  else
  {
    bars = 1;
  }

  int barWidth = 5;
  int barSpacing = 1;
  int startX = 100;
  int startY = 0;
  int barHeight = 8;

  for (int i = 0; i < 5; i++)
  {
    int currentBarHeight = barHeight * (i + 1) / 5;
    if (i < bars)
    {
      display.fillRect(startX + i * (barWidth + barSpacing), startY + (barHeight - currentBarHeight), barWidth, currentBarHeight, SSD1306_WHITE);
    }
  }

  if (onbordGps.location.isValid())
  {
    display.print(" H-62 Lat:");
    display.println(receiveLatitude, 6);
    display.print(" H-62 Lng:");
    display.println(receiveLongtitude, 6);
    display.print(F("TBEAM Lat: "));
    display.println(onbordGps.location.lat(), 6);
    display.print(F("TBEAM Lng: "));
    display.println(onbordGps.location.lng(), 6);
    display.print(F("SAT: "));
    display.println(onbordGps.satellites.value());
  }
  else
  {
    display.println(F("No valid GPS data."));
  }
  display.display();
}

void displayPage1()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("--- LoRa ---"));
  display.setCursor(70, 0);
  display.println();

  int rssi = LoRa.packetRssi();
  int bars = 0;
  if (rssi > -95.8)
  {
    bars = 5;
  }
  else if (rssi > -101.6)
  {
    bars = 4;
  }
  else if (rssi > -107.4)
  {
    bars = 3;
  }
  else if (rssi > -113.2)
  {
    bars = 2;
  }
  else
  {
    bars = 1;
  }

  int barWidth = 5;
  int barSpacing = 1;
  int startX = 100;
  int startY = 0;
  int barHeight = 8;

  for (int i = 0; i < 5; i++)
  {
    int currentBarHeight = barHeight * (i + 1) / 5;
    if (i < bars)
    {
      display.fillRect(startX + i * (barWidth + barSpacing), startY + (barHeight - currentBarHeight), barWidth, currentBarHeight, SSD1306_WHITE);
    }
  }
  display.print(F("RSSI: "));
  display.print(lastRssi);
  display.println(F(" dBm"));
  display.print(F("SNR:  "));
  display.print(lastSnr);
  display.println(F(" dB"));
  display.print(F("ALT: "));
  display.print(altitude);
  display.println(F(" m"));
  display.print(F("BAT: "));
  display.print(batteryVoltage);
  display.println(F(" V"));
  display.print(F("EXT: "));
  display.print(externalVoltage);
  display.println(F(" V"));
  display.print(F("MAIN: "));
  display.print(mainPosition);
  display.println(F(" deg"));
  display.display();
}

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
  }
}

void sendLoRaCommand()
{
  Serial.println("Sending LoRa Command...");
  // Example command: label 0xCC, ident 'G', payload 1
  // MsgPacketizer::send(LoRa, 0xCC, 72, (uint8_t)0);

  const auto &packet = MsgPacketizer::encode(0xCC, (uint8_t)1);
  for (int i = 0; i < 10; i++)
  {
    LoRa.beginPacket();
    LoRa.write(packet.data.data(), packet.data.size());
    LoRa.endPacket();
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 25);
  display.println("COMMAND");
  display.display();

  Tasks["update-display"]->startOnceAfterMsec(1000);
}

void taskGpsUpdate()
{
  while (SerialGPS.available())
  {
    onbordGps.encode(SerialGPS.read());
  }

  if (currentPage == 0)
  {
    updateDisplay();
  }
}

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
      sendLoRaCommand();
      longPressSent = true;
    }
  }
  else
  {
    if (buttonPressTime != 0 && !longPressSent)
    {
      currentPage = (currentPage + 1) % NUM_PAGES;
      Serial.print("Button short press. Switching to page: ");
      Serial.println(currentPage);
      updateDisplay();
    }
    buttonPressTime = 0;
  }
}

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
  display.println("System Start");
  display.display();
  delay(1000);
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
  if (!LoRa.begin(925.6E6)) // 一時的にFlightModuleの周波数
  {
    Serial.println("Starting LoRa failed!");
    display.clearDisplay();
    display.println("LoRa Failed!");
    display.display();
    while (1)
      ;
  }
  LoRa.setSignalBandwidth(500E3);

  MsgPacketizer::subscribe(LoRa, 0x0A);

  //                          [](
  //                              uint32_t millis,
  //                              char ident,
  //                              uint8_t loggerUsage,
  //                              bool doLogging,
  //                              uint8_t framNumber,
  //                              int16_t accelerationX_mps2,
  //                              int16_t accelerationY_mps2,
  //                              int16_t accelerationZ_mps2,
  //                              int16_t accelerationNorm_mps2,
  //                              int16_t roll_deg,
  //                              int16_t pitch_deg,
  //                              int16_t yaw_deg,
  //                              int16_t forceX_N,
  //                              int16_t jerkX_mps3,
  //                              int16_t altitude_m,
  //                              int16_t verticalSpeed_mps,
  //                              int16_t estimated,
  //                              int16_t apogee,
  //                              int16_t externalVoltage_V,
  //                              int16_t batteryVoltage_V,
  //                              int16_t busVoltage_V,
  //                              int16_t externalCurrent_mA,
  //                              int16_t batteryCurrent_mA,
  //                              int16_t busCurrent_mA,
  //                              int16_t externalPower_W,
  //                              int16_t batteryPower_W,
  //                              int16_t busPower_W,
  //                              int16_t externalDieTemperature_C,
  //                              int16_t batteryDieTemperature_C,
  //                              int16_t busDieTemperature_C)
  //                          {
  //                              lastRssi = LoRa.packetRssi();
  //                              lastSnr = LoRa.packetSnr();

  //                              if (currentPage == 1)
  //                              {
  //                                  updateDisplay();
  //                              }

  //                              Serial.print(">LoRa_RSSI_dBm: ");
  //                              Serial.println(lastRssi);
  //                              Serial.print(">LoRa_SNR_dBm: ");
  //                              Serial.println(lastSnr);
  //                              Serial.print(">upTime_sec: ");
  //                              Serial.println((float)millis / 1000);
  //                              Serial.print(">doLogging_bool: ");
  //                              Serial.println(doLogging);
  //                              Serial.print(">loggerUsage_%: ");
  //                              Serial.println(loggerUsage);
  //                              Serial.print(">framNumber: ");
  //                              Serial.println(framNumber);
  //                              Serial.print(">acceleration_mps2_norm: ");
  //                              Serial.println((float)accelerationNorm_mps2 / 10.0);
  //                              Serial.print(">acceleration_mps2_x: ");
  //                              Serial.println((float)accelerationX_mps2 / 10.0);
  //                              Serial.print(">acceleration_mps2_y: ");
  //                              Serial.println((float)accelerationY_mps2 / 10.0);
  //                              Serial.print(">acceleration_mps2_z: ");
  //                              Serial.println((float)accelerationZ_mps2 / 10.0);
  //                              Serial.print(">orientation_deg_roll: ");
  //                              Serial.println((float)roll_deg / 10.0);
  //                              Serial.print(">orientation_deg_pitch: ");
  //                              Serial.println((float)pitch_deg / 10.0);
  //                              Serial.print(">orientation_deg_yaw: ");
  //                              Serial.println((float)yaw_deg / 10.0);
  //                              Serial.print(">forceX_N: ");
  //                              Serial.println((float)forceX_N / 10.0);
  //                              Serial.print(">jerkX_mps3: ");
  //                              Serial.println((float)jerkX_mps3 / 10.0);
  //                              Serial.print(">altitude_m: ");
  //                              Serial.println((float)altitude_m / 10.0);
  //                              altitude = (float)altitude_m / 10.0;
  //                              Serial.print(">vertiaclSpeed_mps: ");
  //                              Serial.println((float)verticalSpeed_mps / 10.0);
  //                              Serial.print(">apogee_m: ");
  //                              Serial.println((float)apogee / 10.0);
  //                              Serial.print(">estimated_s: ");
  //                              Serial.println((float)estimated / 10.0);
  //                              Serial.print(">externalVoltage_V: ");
  //                              Serial.println((float)externalVoltage_V / 100.0);
  //                              externalVoltage = (float)externalVoltage_V / 100.0;
  //                              Serial.print(">batteryVoltage_V: ");
  //                              Serial.println((float)batteryVoltage_V / 100.0);
  //                              batteryVoltage = (float)batteryVoltage_V / 100.0;
  //                              Serial.print(">busVoltage_V: ");
  //                              Serial.println((float)busVoltage_V / 100.0);
  //                              Serial.print(">externalCurrent_mA: ");
  //                              Serial.println((float)externalCurrent_mA / 100.0);
  //                              Serial.print(">batteryCurrent_mA: ");
  //                              Serial.println((float)batteryCurrent_mA / 100.0);
  //                              Serial.print(">busCurrent_mA: ");
  //                              Serial.println((float)busCurrent_mA / 100.0);
  //                              Serial.print(">externalPower_W: ");
  //                              Serial.println((float)externalPower_W / 10.0);
  //                              Serial.print(">batteryPower_W");
  //                              Serial.println((float)batteryPower_W / 10.0);
  //                              Serial.print(">busPower_W: ");
  //                              Serial.println((float)busPower_W / 10.0);
  //                              Serial.print(">groundTemperature_degC: ");
  //                              Serial.println((float)externalDieTemperature_C / 10.0);
  //                              Serial.print(">batteryDieTemperature_degC: ");
  //                              Serial.println((float)batteryDieTemperature_C / 10.0);
  //                              Serial.print(">busDieTemperature_degC: ");
  //                              Serial.println((float)busDieTemperature_C / 10.0);
  //                              Serial.println();
  //                              Serial.flush();

  MsgPacketizer::subscribe(LoRa, 0x0A,

                           [](
                               char ident,
                               uint32_t millis,
                               uint8_t flightMode,
                               uint16_t flightTime,
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
                               int16_t inputVoltage_SUPPLY,
                               int16_t current_SUPPLY,
                               int16_t currentPosition_SUPPLY,
                               int16_t currentDesiredPosition_SUPPLY,
                               int16_t currentVelocity_SUPPLY,
                               uint16_t separation1ProtectionTime,
                               uint16_t separation1ForceTime,
                               uint16_t separation2ProtectionTime,
                               uint16_t separation2ForceTime,
                               uint16_t landingTime)
                           {
                             Serial.print(">LoRa_RSSI_dBm: ");
                             float loraRssi = LoRa.packetRssi();
                             Serial.println(loraRssi);

                             float loraSnr = LoRa.packetSnr();
                             Serial.print(">LoRa_SNR_dBm: ");
                             Serial.println(loraSnr);

                             if (currentPage == 1)
                             {
                               updateDisplay();
                             }

                             Serial.print(">upTime_sec: ");
                             Serial.println(millis / 1000);
                             Serial.print(">doLogging_bool: ");
                             Serial.println(doLogging);
                             Serial.print(">loggerUsage_%: ");
                             Serial.println(loggerUsage);
                             Serial.print(">framNumber: ");
                             Serial.println(framNumber);

                             Serial.print(">FLIGHTMODE: ");
                             Serial.println(flightMode);
                             Serial.print(">flightTime_s");
                             Serial.println((float)flightTime / 1000.0);

                             Serial.print(">MAIN_motorTemperature_*C: ");
                             Serial.println((float)motorTemperature / 100.0);
                             Serial.print(">MAIN_mcuTemperature_*C: ");
                             Serial.println((float)mcuTemperature / 100.0);
                             Serial.print(">MAIN_inputVoltageMain_V");
                             Serial.println((float)inputVoltage / 100.0);
                             Serial.print("MAIN_current_A: ");
                             Serial.println((float)current / 100.0);
                             Serial.print(">MAIN_ValvePosition_deg: ");
                             Serial.println((float)currentPosition / 100.0);
                             mainPosition = (float)currentPosition / 100.0;
                             Serial.print(">MAIN_desiredPosition_deg: ");
                             Serial.println((float)currentDesiredPosition / 100.0);
                             Serial.print(">MAIN_velocity_m/s^2: ");
                             Serial.println((float)currentVelocity / 100.0);

                             Serial.print(">SUPPLY_morotTemperature_*C: ");
                             Serial.println((float)mcuTemperature_SUPPLY / 100.0);
                             Serial.print(">SUPPLY_mcuTemperature_*C: ");
                             Serial.println((float)mcuTemperature_SUPPLY / 100.0);
                             Serial.print(">SUPPLY_inputVoltage_V");
                             Serial.println((float)inputVoltage_SUPPLY / 100.0);
                             Serial.print(">SUPPLY_current_A");
                             Serial.println((float)current_SUPPLY / 100.0);
                             Serial.print(">SUPPLY_ValvePosition_V: ");
                             Serial.println((float)currentPosition_SUPPLY / 100.0);
                             Serial.print(">SUPPLY_desiredPosition_deg: ");
                             Serial.println((float)currentDesiredPosition_SUPPLY / 100.0);
                             Serial.print(">SUPPLY_velocity_m/s^2");
                             Serial.println((float)currentVelocity_SUPPLY / 100.0);

                             Serial.println();
                             Serial.flush();
                           });

  updateDisplay();
  Tasks.add(&taskGpsUpdate)->startFps(5);
  Tasks.add(&taskButtonCheck)->startFps(20);
  Tasks.add("update-display", &updateDisplay);
}

void loop()
{
  Tasks.update();

  if (LoRa.parsePacket())
  {
    MsgPacketizer::parse();
  }
}
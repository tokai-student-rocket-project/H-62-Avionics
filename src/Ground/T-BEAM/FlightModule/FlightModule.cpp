#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>
#include <ArduinoJson.h>
#include "LoRaBoards.h"
#include <TinyGPS++.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h> 

StaticJsonDocument<4096> packet;

uint16_t separation1ProtectionTime = 8605;
uint16_t separation1ForceTime = 11605;
uint16_t separation2ProtectionTime = 1000;
uint16_t separation2ForceTime = 1000;
uint16_t landingTime = 30305;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



float onbordLatitude;
float onbordLongtitude;
float onbordAltitude;
float receiveLatitude;
float receiveLongtitude;
TinyGPSPlus onbordGps;

void task5Hz(){

   while (SerialGPS.available()) {
    onbordGps.encode(SerialGPS.read());
  }


  // GPSデータが有効な場合にのみ表示
  if (onbordGps.location.isValid()) {
    Serial.print(F("Lat: "));
    Serial.print(onbordGps.location.lat(), 6); // 緯度 (小数点以下6桁まで)
    Serial.print(F(", Lng: "));
    Serial.print(onbordGps.location.lng(), 6); // 経度 (小数点以下6桁まで)
    Serial.print(F(", Alt: "));
    Serial.print(onbordGps.altitude.meters(), 2); // 高度 (メートル、小数点以下2桁まで)
    Serial.print(F("m"));
    
    // 日付と時刻の表示
    if (onbordGps.date.isValid() && onbordGps.time.isValid()) {
      Serial.print(F(", Date: "));
      Serial.print(onbordGps.date.year());
      Serial.print(F("-"));
      Serial.print(onbordGps.date.month(), DEC);
      Serial.print(F("-"));
      Serial.print(onbordGps.date.day(), DEC);
      Serial.print(F(", Time: "));
      Serial.print(onbordGps.time.hour(), DEC);
      Serial.print(F(":"));
      Serial.print(onbordGps.time.minute(), DEC);
      Serial.print(F(":"));
      Serial.print(onbordGps.time.second(), DEC);
    }

    Serial.print(F(", Sats: "));
    Serial.print(onbordGps.satellites.value()); // 捕捉衛星数
    Serial.print(F(", HDOP: "));
    Serial.print(onbordGps.hdop.hdop(), 1);     // 水平精度低下率

    Serial.println(); // 改行
  } else {
    // GPSデータがまだ有効でない場合
    Serial.println(F("No valid GPS data yet."));
  }

    // ディスプレイ表示,追記箇所
                             display.clearDisplay(); // 画面クリア
                             display.setTextSize(1); // テキストサイズを1に設定 (すべての情報に適用)

                             //1. GNSSにて測位した情報から現在の時間を表示
                             display.setCursor(0, 0); // Y座標0
                             display.print("TIME: ");
                             if (onbordGps.time.isValid()) { // GPS時刻が有効な場合のみ表示
                                 if (onbordGps.time.hour() < 10) display.print("0"); // 1桁の場合はゼロ埋め
                                 display.print(onbordGps.time.hour());
                                 display.print(":");
                                 if (onbordGps.time.minute() < 10) display.print("0");
                                 display.print(onbordGps.time.minute());
                                 display.print(":");
                                 if (onbordGps.time.second() < 10) display.print("0");
                                 display.println(onbordGps.time.second());
                             } else {
                                 display.println("--:--:--"); // 無効な場合はハイフン表示
                             }

                             //フライトモードの表示
                             display.setCursor(0, 10); // Y座標10
                            //  display.print("Flight Time:"); display.println((float)flightTime / 1000.0);

                             //「すばる」からの測位データをT-Beamのディスプレイに映す．小数点以下6桁
                             display.setCursor(0, 20); // Y座標20
                            //  display.print(" H-62 Lat:"); display.println(latitude, 6); // 小数点以下6桁
                             display.setCursor(0, 30); // Y座標30
                             display.print(" H-62 Lon:"); display.println(receiveLongtitude, 6); // 小数点以下6桁

                             //T-Beam内部のGPSモジュールからの測位データをディスプレイに映す．小数点以下6桁
                             display.setCursor(0, 40); // Y座標40
                             onbordLatitude = onbordGps.location.lat();
                             display.print("TBEAM Lat:"); display.println(onbordLatitude, 6); // 小数点以下6桁
                             display.setCursor(0, 50); // Y座標50(これがギリギリ128x64ディスプレイの最終行)
                             onbordLongtitude= onbordGps.location.();
                             display.print("TBEAM Lon:"); display.println(onbordLongtitude, 6); // 小数点以下6桁
                             // 注: RX Altは画面スペースの制約上、このレイアウトでは表示できない→すばる側の高度を消した

                             display.display(); // 画面更新
}

void setup()
{
  setupBoards();
  // Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 128x64用のアドレス0x3C (一般的なアドレス)
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // 初期化失敗したら停止
  }
  display.display();
  delay(100); //初期表示を短くした
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); //テキストの色を白に設定
  display.setTextSize(1); //テキストサイズを1に設定

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
  //LoRa初期化の成功チェックを追加
  if (!LoRa.begin(925.6E6)) {
    Serial.println("LoRa initialization failed!");
    display.println("LoRa Init Fail!"); // ここを変更した: OLEDにも表示
    display.display();
    while (1);
  }
  LoRa.setSignalBandwidth(500E3);
  Serial.println("LoRa initialized."); //成功メッセージ
  display.println("LoRa Init OK."); //OLEDにも表示
  display.display();
  delay(100); //初期化成功メッセージ表示

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
  LoRa.begin(925.6E6);
  LoRa.setSignalBandwidth(500E3);

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
                             packet.clear();

                             packet["packet"]["module"] = "F";
                             packet["packet"]["rssi_dBm"] = LoRa.packetRssi();
                             packet["packet"]["snr_dBm"] = LoRa.packetSnr();
                             packet["packet"]["ident"] = (String)ident;
                             packet["packet"]["uptime_s"] = (float)millis / 1000.0;

                             packet["logger"]["doLogging"] = doLogging;
                             packet["logger"]["usage"] = loggerUsage;
                             packet["logger"]["number"] = framNumber;

                             packet["flight"]["mode"] = flightMode;
                             packet["flight"]["time_s"] = (float)flightTime / 1000.0;
                             packet["flight"]["detection"]["valveModeIsLaunch"] = isLaunchMode;
                             packet["flight"]["detection"]["flightPinIsOpen"] = flightPinIsOpen;
                             packet["flight"]["detection"]["isFalling"] = isFalling;
                             packet["flight"]["separation"]["sn3IsOn"] = sn3IsOn;
                             packet["flight"]["separation"]["sn4IsOn"] = sn4IsOn;

                             packet["gnss"]["unixEpoch"] = unixEpoch;
                             packet["gnss"]["fixType"] = fixType;
                             packet["gnss"]["satellites"] = satelliteCount;
                             packet["gnss"]["latitude_deg"] = latitude;
                             receiveLongtitude = longitude;
                             packet["gnss"]["height_m"] = (float)height / 10.0;
                             packet["gnss"]["speed_mps"] = (float)speed / 10.0;
                             packet["gnss"]["accuracy_m"] = (float)accuracy / 10.0;

                             packet["valve"]["motorTemperature_degC"] = (float)motorTemperature / 100.0;
                             packet["valve"]["mcuTemperature_degC"] = (float)mcuTemperature / 100.0;
                             packet["valve"]["inputVoltage_V"] = (float)inputVoltage / 100.0;
                             packet["valve"]["current"] = (float)current / 100.0;
                             packet["valve"]["currentPosition_deg"] = (float)currentPosition / 100.0;
                             packet["valve"]["currentDesiredPosition_deg"] = (float)currentDesiredPosition / 100.0;
                             packet["valve"]["currentVelocity_degps"] = (float)currentVelocity / 100.0;

                             packet["valve"]["motorTemperatureSupply_degC"] = (float)motorTemperature_SUPPLY / 100.0;
                             packet["valve"]["mcuTemperatureSupply_degC"] = (float)mcuTemperature_SUPPLY / 100.0;
                             packet["valve"]["inputVoltageSupply_V"] = (float)inputVoltage_SUPPLY / 100.0;
                             packet["valve"]["currentSupply_A"] = (float)current_SUPPLY / 100.0;
                             packet["valve"]["currentPositionSupply_deg"] = (float)currentPosition_SUPPLY / 100.0;
                             packet["valve"]["currentDesiredPositionSupply_deg"] = (float)currentDesiredPosition_SUPPLY / 100.0;
                             packet["valve"]["currentVelocitySupply_deg"] = (float)currentVelocity_SUPPLY / 100.0;

                             packet["timer"]["separation_1_protection_time"] = (float)separation1ProtectionTime / 1000.0;
                             packet["timer"]["separation_1_force_time"] = (float)separation1ForceTime / 1000.0;
                             packet["timer"]["separation_2_protection_time"] = (float)separation2ProtectionTime / 1000.0;
                             packet["timer"]["separation_2_force_time"] = (float)separation2ForceTime / 1000.0;
                             packet["timer"]["landing_time"] = (float)landingTime / 1000.0;

                             serializeJson(packet, Serial);
                             Serial.println();
                             Serial.flush();

                             
                           });

    Tasks.add(&task5Hz)->startFps(5);
}

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

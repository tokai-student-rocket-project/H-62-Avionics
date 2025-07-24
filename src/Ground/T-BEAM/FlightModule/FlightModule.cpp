// // // #include <Arduino.h>
// // // #include <SPI.h>
// // // #include <Wire.h>
// // // #include <LoRa.h>
// // // #include <MsgPacketizer.h>
// // // #include <TaskManager.h>
// // // #include <ArduinoJson.h>

// // // StaticJsonDocument<4096> packet;

// // // uint16_t separation1ProtectionTime = 8605;
// // // uint16_t separation1ForceTime = 11605;
// // // uint16_t separation2ProtectionTime = 1000;
// // // uint16_t separation2ForceTime = 1000;
// // // uint16_t landingTime = 30305;

// // // void setup()
// // // {
// // //   Serial.begin(115200);

// // //   pinMode(LED_BUILTIN, OUTPUT);

// // //   LoRa.begin(925.6E6);
// // //   LoRa.setSignalBandwidth(500E3);

// // //   MsgPacketizer::subscribe(LoRa, 0x0A,

// // //                            [](
// // //                                char ident,
// // //                                uint32_t millis,
// // //                                uint8_t flightMode,
// // //                                uint16_t flightTime,
// // //                                uint8_t loggerUsage,
// // //                                bool doLogging,
// // //                                uint8_t framNumber,
// // //                                bool flightPinIsOpen,
// // //                                bool sn3IsOn,
// // //                                bool sn4IsOn,
// // //                                bool isLaunchMode,
// // //                                bool isFalling,
// // //                                uint32_t unixEpoch,
// // //                                uint8_t fixType,
// // //                                uint8_t satelliteCount,
// // //                                float latitude,
// // //                                float longitude,
// // //                                int16_t height,
// // //                                int16_t speed,
// // //                                uint16_t accuracy,
// // //                                int16_t motorTemperature,
// // //                                int16_t mcuTemperature,
// // //                                uint16_t inputVoltage,
// // //                                int16_t current,
// // //                                int16_t currentPosition,
// // //                                int16_t currentDesiredPosition,
// // //                                int16_t currentVelocity,

// // //                                int16_t motorTemperature_SUPPLY,
// // //                                int16_t mcuTemperature_SUPPLY,
// // //                                int16_t inputVoltage_SUPPLY,
// // //                                int16_t current_SUPPLY,
// // //                                int16_t currentPosition_SUPPLY,
// // //                                int16_t currentDesiredPosition_SUPPLY,
// // //                                int16_t currentVelocity_SUPPLY,
// // //                                uint16_t separation1ProtectionTime,
// // //                                uint16_t separation1ForceTime,
// // //                                uint16_t separation2ProtectionTime,
// // //                                uint16_t separation2ForceTime,
// // //                                uint16_t landingTime)
// // //                            {
// // //                              packet.clear();

// // //                              packet["packet"]["module"] = "F";
// // //                              packet["packet"]["rssi_dBm"] = LoRa.packetRssi();
// // //                              packet["packet"]["snr_dBm"] = LoRa.packetSnr();
// // //                              packet["packet"]["ident"] = (String)ident;
// // //                              packet["packet"]["uptime_s"] = (float)millis / 1000.0;

// // //                              packet["logger"]["doLogging"] = doLogging;
// // //                              packet["logger"]["usage"] = loggerUsage;
// // //                              packet["logger"]["number"] = framNumber;

// // //                              packet["flight"]["mode"] = flightMode;
// // //                              packet["flight"]["time_s"] = (float)flightTime / 1000.0;
// // //                              packet["flight"]["detection"]["valveModeIsLaunch"] = isLaunchMode;
// // //                              packet["flight"]["detection"]["flightPinIsOpen"] = flightPinIsOpen;
// // //                              packet["flight"]["detection"]["isFalling"] = isFalling;
// // //                              packet["flight"]["separation"]["sn3IsOn"] = sn3IsOn;
// // //                              packet["flight"]["separation"]["sn4IsOn"] = sn4IsOn;

// // //                              packet["gnss"]["unixEpoch"] = unixEpoch;
// // //                              packet["gnss"]["fixType"] = fixType;
// // //                              packet["gnss"]["satellites"] = satelliteCount;
// // //                              packet["gnss"]["latitude_deg"] = latitude;
// // //                              packet["gnss"]["longitude_deg"] = longitude;
// // //                              packet["gnss"]["height_m"] = (float)height / 10.0;
// // //                              packet["gnss"]["speed_mps"] = (float)speed / 10.0;
// // //                              packet["gnss"]["accuracy_m"] = (float)accuracy / 10.0;

// // //                              packet["valve"]["motorTemperature_degC"] = (float)motorTemperature / 100.0;
// // //                              packet["valve"]["mcuTemperature_degC"] = (float)mcuTemperature / 100.0;
// // //                              packet["valve"]["inputVoltage_V"] = (float)inputVoltage / 100.0;
// // //                              packet["valve"]["current"] = (float)current / 100.0;
// // //                              packet["valve"]["currentPosition_deg"] = (float)currentPosition / 100.0;
// // //                              packet["valve"]["currentDesiredPosition_deg"] = (float)currentDesiredPosition / 100.0;
// // //                              packet["valve"]["currentVelocity_degps"] = (float)currentVelocity / 100.0;

// // //                              packet["valve"]["motorTemperatureSupply_degC"] = (float)motorTemperature_SUPPLY / 100.0;
// // //                              packet["valve"]["mcuTemperatureSupply_degC"] = (float)mcuTemperature_SUPPLY / 100.0;
// // //                              packet["valve"]["inputVoltageSupply_V"] = (float)inputVoltage_SUPPLY / 100.0;
// // //                              packet["valve"]["currentSupply_A"] = (float)current_SUPPLY / 100.0;
// // //                              packet["valve"]["currentPositionSupply_deg"] = (float)currentPosition_SUPPLY / 100.0;
// // //                              packet["valve"]["currentDesiredPositionSupply_deg"] = (float)currentDesiredPosition_SUPPLY / 100.0;
// // //                              packet["valve"]["currentVelocitySupply_deg"] = (float)currentVelocity_SUPPLY / 100.0;

// // //                              packet["timer"]["separation_1_protection_time"] = (float)separation1ProtectionTime / 1000.0;
// // //                              packet["timer"]["separation_1_force_time"] = (float)separation1ForceTime / 1000.0;
// // //                              packet["timer"]["separation_2_protection_time"] = (float)separation2ProtectionTime / 1000.0;
// // //                              packet["timer"]["separation_2_force_time"] = (float)separation2ForceTime / 1000.0;
// // //                              packet["timer"]["landing_time"] = (float)landingTime / 1000.0;

// // //                              serializeJson(packet, Serial);
// // //                              Serial.println();
// // //                              Serial.flush();

// // //                              digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
// // //                            });
// // // }

// // // void loop()
// // // {
// // //   Tasks.update();

// // //   if (Serial.available())
// // //   {
// // //     char command = Serial.read();

// // //     // F: フライトモードオン
// // //     if (command == 'F')
// // //     {
// // //       const auto &packet = MsgPacketizer::encode(0xF1, (uint8_t)0);
// // //       LoRa.beginPacket();
// // //       LoRa.write(packet.data.data(), packet.data.size());
// // //       LoRa.endPacket();
// // //     }

// // //     // R: フライトモードリセット
// // //     if (command == 'R')
// // //     {
// // //       const auto &packet = MsgPacketizer::encode(0xF2, (uint8_t)0);
// // //       LoRa.beginPacket();
// // //       LoRa.write(packet.data.data(), packet.data.size());
// // //       LoRa.endPacket();
// // //     }

// // //     // C: タイマー設定
// // //     if (command == 'C')
// // //     {
// // //       const auto &packet = MsgPacketizer::encode(0xF3, separation1ProtectionTime, separation1ForceTime, separation2ProtectionTime, separation2ForceTime, landingTime);
// // //       LoRa.beginPacket();
// // //       LoRa.write(packet.data.data(), packet.data.size());
// // //       LoRa.endPacket();
// // //     }
// // //   }

// // //   if (LoRa.parsePacket())
// // //   {
// // //     MsgPacketizer::parse();
// // //   }
// // // }

// // #include <TinyGPS++.h>
// // #include "LoRaBoards.h" // TTGO T-BEAM特有のピン設定などが含まれるため必要

// // // TinyGPS++ オブジェクトを宣言
// // TinyGPSPlus gps;

// // void setup() {
// //   // ボードのセットアップ (シリアルポートの初期化など)
// //   setupBoards();

// //   // シリアルモニターの初期化 (デバッグ出力用)
// //   // LoRaBoards.h 内の setupBoards() で Serial.begin() が呼ばれる場合があるが、
// //   // 明示的にSerial.begin(115200); などで初期化しても良いかな。
// //   // Serial.begin(115200); // 必要であればコメントを外して

// //   Serial.println(F("TTGO T-BEAM GPS Receiver"));
// //   Serial.println(F("-------------------------"));
// //   Serial.println(F("Waiting for GPS data..."));
// // }

// // void loop() {
// //   // GPSデータを読み込み、TinyGPS++オブジェクトにエンコードする
// //   // SerialGPSはLoRaBoards.hで定義されているGPSモジュールへのシリアル通信。
// //   while (SerialGPS.available()) {
// //     gps.encode(SerialGPS.read());
// //   }

// //   // GPSデータが有効な場合にのみ表示
// //   if (gps.location.isValid()) {
// //     Serial.print(F("Lat: "));
// //     Serial.print(gps.location.lat(), 6); // 緯度 (小数点以下6桁まで)
// //     Serial.print(F(", Lng: "));
// //     Serial.print(gps.location.lng(), 6); // 経度 (小数点以下6桁まで)
// //     Serial.print(F(", Alt: "));
// //     Serial.print(gps.altitude.meters(), 2); // 高度 (メートル、小数点以下2桁まで)
// //     Serial.print(F("m"));

// //     // 日付と時刻の表示
// //     if (gps.date.isValid() && gps.time.isValid()) {
// //       Serial.print(F(", Date: "));
// //       Serial.print(gps.date.year());
// //       Serial.print(F("-"));
// //       Serial.print(gps.date.month(), DEC);
// //       Serial.print(F("-"));
// //       Serial.print(gps.date.day(), DEC);
// //       Serial.print(F(", Time: "));
// //       Serial.print(gps.time.hour(), DEC);
// //       Serial.print(F(":"));
// //       Serial.print(gps.time.minute(), DEC);
// //       Serial.print(F(":"));
// //       Serial.print(gps.time.second(), DEC);
// //     }

// //     Serial.print(F(", Sats: "));
// //     Serial.print(gps.satellites.value()); // 捕捉衛星数
// //     Serial.print(F(", HDOP: "));
// //     Serial.print(gps.hdop.hdop(), 1);     // 水平精度低下率

// //     Serial.println(); // 改行
// //   } else {
// //     // GPSデータがまだ有効でない場合
// //     Serial.println(F("No valid GPS data yet."));
// //   }

// //   // 短い遅延 (GPSデータの処理を妨げないように短くする)
// //   delay(1000);
// // }



// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <LoRa.h>
// #include <MsgPacketizer.h>
// #include <TaskManager.h>
// #include <ArduinoJson.h>
// #include "LoRaBoards.h"

// #ifndef CONFIG_RADIO_FREQ
// #define CONFIG_RADIO_FREQ           925.6
// #endif
// #ifndef CONFIG_RADIO_OUTPUT_POWER
// #define CONFIG_RADIO_OUTPUT_POWER   17
// #endif
// #ifndef CONFIG_RADIO_BW
// #define CONFIG_RADIO_BW             500.0
// #endif

// StaticJsonDocument<4096> packet;

// uint16_t separation1ProtectionTime = 8605;
// uint16_t separation1ForceTime = 11605;
// uint16_t separation2ProtectionTime = 1000;
// uint16_t separation2ForceTime = 1000;
// uint16_t landingTime = 30305;

// void setup()
// {
//   setupBoards();
//   Serial.begin(115200);

//   // pinMode(LED_BUILTIN, OUTPUT);

//   #ifdef  RADIO_TCXO_ENABLE
//     pinMode(RADIO_TCXO_ENABLE, OUTPUT);
//     digitalWrite(RADIO_TCXO_ENABLE, HIGH);
// #endif

// #ifdef RADIO_CTRL
//     Serial.println("Turn on LAN, Enter Rx mode.");
//     /*
//     * BPF LoRa LAN Control ,set HIGH turn on LAN ,RX Mode
//     * */
//     digitalWrite(RADIO_CTRL, HIGH);
// #endif /*RADIO_CTRL*/

//   LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
//       if (!LoRa.begin(CONFIG_RADIO_FREQ * 1000000)) {
//         Serial.println("Starting LoRa failed!");
//         while (1);
//     }

//   LoRa.setTxPower(CONFIG_RADIO_OUTPUT_POWER);

//   LoRa.setSignalBandwidth(CONFIG_RADIO_BW * 1000);

//   LoRa.setSpreadingFactor(10);

//   LoRa.setPreambleLength(16);

//   LoRa.setSyncWord(0xAB);

//   LoRa.disableCrc();

//   LoRa.disableInvertIQ();

//   LoRa.setCodingRate4(7);

  


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
//                              packet["gnss"]["longitude_deg"] = longitude;
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

//                             //  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//                            });
// }

// void loop()
// {
//   Tasks.update();

//   if(LoRa.available())
//   {
//     Serial.println(LoRa.available());
//     Serial.println(LoRa.packetRssi());
//   }

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
#include <ArduinoJson.h>
#include "LoRaBoards.h"

// --- LoRa通信設定 ---
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ               925.6
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER       17
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW                 500.0
#endif

#if !defined(USING_SX1276) && !defined(USING_SX1278)
#error "LoRa example is only allowed to run SX1276/78. For other RF models, please run examples/RadioLibExamples"
#endif

// !!! 修正点1: receivedTelemetryJson をグローバルスコープに宣言 !!!
StaticJsonDocument<4096> receivedTelemetryJson;

// コマンド送信で使用するタイマー値のデフォルト設定。
uint16_t command_separation1ProtectionTime = 8605;
uint16_t command_separation1ForceTime = 11605;
uint16_t command_separation2ProtectionTime = 1000;
uint16_t command_separation2ForceTime = 1000;
uint16_t command_landingTime = 30305;

// --- 受信データ処理用コールバック関数 ---
void onReceiveTelemetryData(
    char ident,
    uint32_t mcuMillis,
    uint16_t flightTime,
    uint8_t flightMode,
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
    int16_t height_raw,
    int16_t speed_raw,
    uint16_t accuracy_raw,
    int16_t motorTemperature_raw,
    int16_t mcuTemperature_raw,
    uint16_t inputVoltage_raw,
    int16_t current_raw,
    int16_t currentPosition_raw,
    int16_t currentDesiredPosition_raw,
    int16_t currentVelocity_raw,
    int16_t motorTemperature_SUPPLY_raw,
    int16_t mcuTemperature_SUPPLY_raw,
    int16_t inputVoltage_SUPPLY_raw,
    int16_t current_SUPPLY_raw,
    int16_t currentPosition_SUPPLY_raw,
    int16_t currentDesiredPosition_SUPPLY_raw,
    int16_t currentVelocity_SUPPLY_raw,
    uint16_t separation1ProtectionTime,
    uint16_t separation1ForceTime,
    uint16_t separation2ProtectionTime,
    uint16_t separation2ForceTime,
    uint16_t landingTime
) {
    receivedTelemetryJson.clear(); // 新しいデータのためにクリア

    receivedTelemetryJson["packet"]["rssi_dBm"] = LoRa.packetRssi();
    receivedTelemetryJson["packet"]["snr_dBm"] = LoRa.packetSnr();

    receivedTelemetryJson["rocket"]["ident"] = (String)ident;
    receivedTelemetryJson["rocket"]["mcuUptime_s"] = (float)mcuMillis / 1000.0;

    JsonObject logger = receivedTelemetryJson.createNestedObject("logger");
    logger["doLogging"] = doLogging;
    logger["usage"] = loggerUsage;
    logger["framNumber"] = framNumber;

    JsonObject flight = receivedTelemetryJson.createNestedObject("flight");
    flight["mode"] = flightMode;
    flight["time_s"] = (float)flightTime / 1000.0;

    JsonObject flightDetection = flight.createNestedObject("detection");
    flightDetection["isLaunchMode"] = isLaunchMode;
    flightDetection["flightPinIsOpen"] = flightPinIsOpen;
    flightDetection["isFalling"] = isFalling;

    JsonObject flightSeparation = flight.createNestedObject("separation");
    flightSeparation["sn3IsOn"] = sn3IsOn;
    flightSeparation["sn4IsOn"] = sn4IsOn;

    JsonObject gnss = receivedTelemetryJson.createNestedObject("gnss");
    gnss["unixEpoch"] = unixEpoch;
    gnss["fixType"] = fixType;
    gnss["satellites"] = satelliteCount;
    gnss["latitude_deg"] = latitude;
    gnss["longitude_deg"] = longitude;
    gnss["height_m"] = (float)height_raw / 10.0;
    gnss["speed_mps"] = (float)speed_raw / 10.0;
    gnss["accuracy_m"] = (float)accuracy_raw / 10.0;

    JsonObject valve = receivedTelemetryJson.createNestedObject("valve");
    valve["motorTemp_degC"] = (float)motorTemperature_raw / 100.0;
    valve["mcuTemp_degC"] = (float)mcuTemperature_raw / 100.0;
    valve["inputVoltage_V"] = (float)inputVoltage_raw / 100.0;
    valve["current_A"] = (float)current_raw / 100.0;
    valve["currentPosition_deg"] = (float)currentPosition_raw / 100.0;
    valve["desiredPosition_deg"] = (float)currentDesiredPosition_raw / 100.0;
    valve["velocity_degps"] = (float)currentVelocity_raw / 100.0;

    JsonObject supplyValve = receivedTelemetryJson.createNestedObject("supplyValve");
    supplyValve["motorTemp_degC"] = (float)motorTemperature_SUPPLY_raw / 100.0;
    supplyValve["mcuTemp_degC"] = (float)mcuTemperature_SUPPLY_raw / 100.0;
    supplyValve["inputVoltage_V"] = (float)inputVoltage_SUPPLY_raw / 100.0;
    supplyValve["current_A"] = (float)current_SUPPLY_raw / 100.0;
    supplyValve["currentPosition_deg"] = (float)currentPosition_SUPPLY_raw / 100.0;
    supplyValve["desiredPosition_deg"] = (float)currentDesiredPosition_SUPPLY_raw / 100.0;
    supplyValve["velocity_degps"] = (float)currentVelocity_SUPPLY_raw / 100.0;

    JsonObject timers = receivedTelemetryJson.createNestedObject("timers");
    timers["separation1Protection_s"] = (float)separation1ProtectionTime / 1000.0;
    timers["separation1Force_s"] = (float)separation1ForceTime / 1000.0;
    timers["separation2Protection_s"] = (float)separation2ProtectionTime / 1000.0;
    timers["separation2Force_s"] = (float)separation2ForceTime / 1000.0;
    timers["landing_s"] = (float)landingTime / 1000.0;

    Serial.print("Received: ");
    serializeJson(receivedTelemetryJson, Serial);
    Serial.println();
    Serial.flush();

    // --- OLEDディスプレイ表示 (オプション) ---
    // u8g2 初期化の例 (ご自身のボードに合わせて調整してください)
    // U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);
    /*
    if (u8g2) {
        u8g2->clearBuffer();
        u8g2->setFont(u8g2_font_ncenB08_tr);

        u8g2->drawStr(0, 10, "Packet RX!");

        char buf[64];
        snprintf(buf, sizeof(buf), "RSSI:%d dBm", LoRa.packetRssi());
        u8g2->drawStr(0, 22, buf);
        snprintf(buf, sizeof(buf), "SNR:%.1f dB", LoRa.packetSnr());
        u8g2->drawStr(0, 34, buf);

        snprintf(buf, sizeof(buf), "Lat:%.4f", latitude);
        u8g2->drawStr(0, 46, buf);
        snprintf(buf, sizeof(buf), "Lon:%.4f", longitude);
        u8g2->drawStr(0, 58, buf);
        snprintf(buf, sizeof(buf), "Alt:%.1fm", (float)height_raw / 10.0);
        u8g2->drawStr(65, 46, buf);
        snprintf(buf, sizeof(buf), "Mode:%d", flightMode);
        u8g2->drawStr(65, 58, buf);

        u8g2->sendBuffer();
    }
    */
}

void setup()
{
    setupBoards();
    Serial.begin(115200);

    Serial.println(F("--- TTGO T-BEAM LoRa Receiver & Commander ---"));
    Serial.println(F("Initializing LoRa module..."));

    delay(1500);

#ifdef RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);
    digitalWrite(RADIO_TCXO_ENABLE, HIGH);
    Serial.println(F("TCXO enabled."));
#endif

#ifdef RADIO_CTRL
    Serial.println(F("Enabling LoRa LAN control and setting to Rx mode."));
    digitalWrite(RADIO_CTRL, HIGH);
#endif /*RADIO_CTRL*/

    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);

    if (!LoRa.begin(CONFIG_RADIO_FREQ * 1000000)) {
        Serial.println(F("Error: Starting LoRa failed! Check wiring, frequency, and pin definitions."));
        while (1);
    }

    LoRa.setTxPower(CONFIG_RADIO_OUTPUT_POWER);
    LoRa.setSignalBandwidth(CONFIG_RADIO_BW * 1000);
    LoRa.setSpreadingFactor(10);
    LoRa.setPreambleLength(16);
    LoRa.setSyncWord(0xAB);
    LoRa.disableCrc();
    LoRa.disableInvertIQ();
    LoRa.setCodingRate4(7);

    Serial.println(F("LoRa module initialized successfully with the following settings:"));
    Serial.print(F("  Frequency: ")); Serial.print(CONFIG_RADIO_FREQ); Serial.println(F(" MHz"));
    Serial.print(F("  Bandwidth: ")); Serial.print(CONFIG_RADIO_BW); Serial.println(F(" kHz"));
    Serial.print(F("  Spreading Factor: ")); Serial.println(10);
    Serial.print(F("  Sync Word: 0x")); Serial.println(0xAB, HEX);
    Serial.println(F("Listening for telemetry packets (0x0A)..."));
    Serial.println(F("Commands via Serial: F (FlightModeOn), R (FlightModeReset), C (SetTimers)"));

    // !!! 修正点2: MsgPacketizer::subscribe のコールバック関数を匿名関数でラップ !!!
    MsgPacketizer::subscribe(LoRa, 0x0A,
        [](
            char ident,
            uint32_t mcuMillis,
            uint16_t flightTime,
            uint8_t flightMode,
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
            int16_t height_raw,
            int16_t speed_raw,
            uint16_t accuracy_raw,
            int16_t motorTemperature_raw,
            int16_t mcuTemperature_raw,
            uint16_t inputVoltage_raw,
            int16_t current_raw,
            int16_t currentPosition_raw,
            int16_t currentDesiredPosition_raw,
            int16_t currentVelocity_raw,
            int16_t motorTemperature_SUPPLY_raw,
            int16_t mcuTemperature_SUPPLY_raw,
            int16_t inputVoltage_SUPPLY_raw,
            int16_t current_SUPPLY_raw,
            int16_t currentPosition_SUPPLY_raw,
            int16_t currentDesiredPosition_SUPPLY_raw,
            int16_t currentVelocity_SUPPLY_raw,
            uint16_t separation1ProtectionTime,
            uint16_t separation1ForceTime,
            uint16_t separation2ProtectionTime,
            uint16_t separation2ForceTime,
            uint16_t landingTime) {
                // ラッパー内で実際の処理関数を呼び出す
                onReceiveTelemetryData(
                    ident, mcuMillis, flightTime, flightMode, loggerUsage, doLogging, framNumber,
                    flightPinIsOpen, sn3IsOn, sn4IsOn, isLaunchMode, isFalling, unixEpoch, fixType,
                    satelliteCount, latitude, longitude, height_raw, speed_raw, accuracy_raw,
                    motorTemperature_raw, mcuTemperature_raw, inputVoltage_raw, current_raw,
                    currentPosition_raw, currentDesiredPosition_raw, currentVelocity_raw,
                    motorTemperature_SUPPLY_raw, mcuTemperature_SUPPLY_raw, inputVoltage_SUPPLY_raw,
                    current_SUPPLY_raw, currentPosition_SUPPLY_raw, currentDesiredPosition_SUPPLY_raw,
                    currentVelocity_SUPPLY_raw, separation1ProtectionTime, separation1ForceTime,
                    separation2ProtectionTime, separation2ForceTime, landingTime
                );
            }
    );

    LoRa.receive();
}

void loop()
{
    Tasks.update();

    if (Serial.available())
    {
        char command = Serial.read();

        if (command == 'F')
        {
            Serial.println(F("Sending: FlightModeOn command (0xF1) to MKRWAN."));
            const auto &packetToSend = MsgPacketizer::encode(0xF1, (uint8_t)0);
            LoRa.beginPacket();
            LoRa.write(packetToSend.data.data(), packetToSend.data.size());
            LoRa.endPacket();
        }
        else if (command == 'R')
        {
            Serial.println(F("Sending: FlightModeReset command (0xF2) to MKRWAN."));
            const auto &packetToSend = MsgPacketizer::encode(0xF2, (uint8_t)0);
            LoRa.beginPacket();
            LoRa.write(packetToSend.data.data(), packetToSend.data.size());
            LoRa.endPacket();
        }
        else if (command == 'C')
        {
            Serial.println(F("Sending: SetTimers command (0xF3) to MKRWAN."));
            const auto &packetToSend = MsgPacketizer::encode(0xF3,
                                                            command_separation1ProtectionTime,
                                                            command_separation1ForceTime,
                                                            command_separation2ProtectionTime,
                                                            command_separation2ForceTime,
                                                            command_landingTime);
            LoRa.beginPacket();
            LoRa.write(packetToSend.data.data(), packetToSend.data.size());
            LoRa.endPacket();
        }
        else
        {
            Serial.print(F("Unknown command received via Serial: '"));
            Serial.print(command);
            Serial.println(F("'."));
        }
    }

    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        MsgPacketizer::parse();
    }
}
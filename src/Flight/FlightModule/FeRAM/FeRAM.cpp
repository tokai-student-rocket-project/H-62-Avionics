#include <Arduino.h> // Arduino スケッチの基本ヘッダ
#include <SPI.h>     // SPI 通信ライブラリ
#include <TaskManager.h> // タスク管理ライブラリ
#include <MsgPacketizer.h> // メッセージパケット化ライブラリ
#include "Lib_Logger3.hpp" // Logger クラスのヘッダ
#include "Lib_FRAM.hpp" // FRAM クラスのヘッダ


// FRAM のチップセレクト (CS) ピン番号を定義
const uint32_t FRAM0_CS_PIN = 7;
const uint32_t FRAM1_CS_PIN = A1;
const uint32_t FRAM2_CS_PIN = A2;

// Logger オブジェクトをグローバルに宣言
Logger logger(FRAM0_CS_PIN, FRAM1_CS_PIN, FRAM2_CS_PIN);

void loggingTask() {
    const auto &logPacket = MsgPacketizer::encode(0x0A, millis(), logger.getUsage(), logger.framNumber());
    logger.write(logPacket.data.data(), logPacket.data.size());

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  SPI.begin();

  while (!Serial);
  delay(5000);

  Serial.println("Logger sample start.");

  Serial.println("Initializing FRAM and clearing...");
  // FRAM の全データをクリアする
  // 注意：この処理は時間がかかる
  logger.clear();
  Serial.println("FRAM cleared.");

  // オフセットをリセット (clear() の後に呼ぶ必要は通常ありませんが、明示的に)
  logger.reset();
  Serial.print("Initial offset: ");
  Serial.println(logger.getOffset());

  Tasks.add(&loggingTask)->startFps(2000);
}

void loop() {
  Tasks.update(); // タスクの更新
}
#include <Arduino.h> // Arduino スケッチの基本ヘッダ
#include <SPI.h>     // SPI 通信ライブラリ
#include <TaskManager.h> // タスク管理ライブラリ
#include <MsgPacketizer.h> // メッセージパケット化ライブラリ
#include "Lib_Logger2.hpp" // Logger クラスのヘッダ
#include "Lib_FRAM.hpp" // FRAM クラスのヘッダ


// FRAM のチップセレクト (CS) ピン番号を定義
const uint32_t FRAM0_CS_PIN = 28; // 例としてピン10を使用
const uint32_t FRAM1_CS_PIN = 29; // 例としてピン10を使用

// Logger オブジェクトをグローバルに宣言
Logger logger(FRAM0_CS_PIN, FRAM1_CS_PIN);
FRAM fram0(FRAM0_CS_PIN);
FRAM fram1(FRAM1_CS_PIN);

void task500Hz() {
    const auto &logPacket = MsgPacketizer::encode(0x0A, millis());
    logger.write(logPacket.data.data(), logPacket.data.size());
}

void setup() {
  // シリアル通信の初期化 (デバッグ出力用)
  Serial.begin(115200);
  while (!Serial); // シリアルポートが開くまで待機
  delay(5000);

  Serial.println("Logger sample start.");

  SPI.begin();

  Serial.println("Initializing FRAM and clearing...");
  // FRAM の全データをクリアする
  // 注意：この処理は時間がかかる
  logger.clear();
  Serial.println("FRAM cleared.");

  // オフセットをリセット (clear() の後に呼ぶ必要は通常ありませんが、明示的に)
  logger.reset();
  Serial.print("Initial offset: ");
  Serial.println(logger.getOffset());

  Tasks.add(&task500Hz)->startFps(500); // 500Hz のタスクを追加
}

void loop() {
  Tasks.update(); // タスクの更新
}
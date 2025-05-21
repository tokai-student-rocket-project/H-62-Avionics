#include <Arduino.h> // Arduino スケッチの基本ヘッダ
#include <SPI.h>     // SPI 通信ライブラリ
#include "Lib_Logger2.hpp" // Logger クラスのヘッダ [2]

// FRAM のチップセレクト (CS) ピン番号を定義
// 使用するボードと配線に合わせて適切なピン番号に変更してください。
const uint32_t FRAM0_CS_PIN = 28; // 例としてピン10を使用
const uint32_t FRAM1_CS_PIN = 29; // 例としてピン10を使用

// Logger オブジェクトをグローバルに宣言
Logger logger(FRAM0_CS_PIN, FRAM1_CS_PIN);

void setup() {
  // シリアル通信の初期化 (dump() やデバッグ出力用)
  Serial.begin(115200);
  while (!Serial); // シリアルポートが開くまで待機 (一部のボードで必要)
  delay(5000);

  Serial.println("Logger sample start.");

  // SPI ライブラリの初期化 (FRAM クラス内部で SPISettings は設定されますが、
  // SPI.begin() 自体は一度行う必要があります)
  // FRAM コンストラクタ内で SPI.beginTransaction() が呼ばれる前に必要です。
  // 提供されたソースにはありませんが、SPI 通信には通常必須です。
  SPI.begin();

  Serial.println("Initializing FRAM and clearing...");
  // FRAM の全データをクリアする
  // 注意：この処理は時間がかかる場合があります (LENGTH * 1バイト書き込み)。 [4]
  logger.clear();
  Serial.println("FRAM cleared.");

  // オフセットをリセット (clear() の後に呼ぶ必要は通常ありませんが、明示的に)
  logger.reset();
  Serial.print("Initial offset: ");
  Serial.println(logger.getOffset());

  // データを書き込む
  uint8_t data_to_write1[] = { 0xAA, 0xBB, 0xCC, 0xDD };
  Serial.print("Writing first data block (");
  Serial.print(sizeof(data_to_write1));
  Serial.println(" bytes)...");
  uint32_t bytes_written1 = logger.write(data_to_write1, sizeof(data_to_write1));
  Serial.print("Bytes written: ");
  Serial.println(bytes_written1);
  Serial.print("Current offset: ");
  Serial.println(logger.getOffset());
  Serial.print("Usage: ");
  Serial.print(logger.getUsage());
  Serial.println("%");

  // 別のデータを追記する
  uint8_t data_to_write2[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
  Serial.print("Writing second data block (");
  Serial.print(sizeof(data_to_write2));
  Serial.println(" bytes)...");
  uint32_t bytes_written2 = logger.write(data_to_write2, sizeof(data_to_write2));
  Serial.print("Bytes written: ");
  Serial.println(bytes_written2);
  Serial.print("Current offset: ");
  Serial.println(logger.getOffset());
  Serial.print("Usage: ");
  Serial.print(logger.getUsage());
  Serial.println("%");

  // FRAM の内容をダンプする
  Serial.println("Dumping FRAM content...");
  logger.dump();
  Serial.println("\nDump complete.");

  // 現在の FRAM 番号 (満杯かどうかのチェック)
  int8_t fram_num = logger.framNumber(); // framNumberはuint8_tを返すが、-1を返すのでint8_tで受けると良いかもしれない
  Serial.print("FRAM number/status: ");
  Serial.println(fram_num); // 0ならOK, -1なら満杯 [6]

  Serial.println("Logger sample finished.");
}

void loop() {
  // このサンプルでは loop 関数には何も記述しません。
  // 必要に応じて、ここで定期的なロギング処理などを記述できます。
}
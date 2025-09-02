/**
 * @file WebLoRa.cpp
 * @brief ESP32でWebサーバーを立て、動的なプログレスバーでデータを表示するサンプル
 * @details
 * Wi-Fiに接続し、Webサーバーを起動します。
 * WebブラウザからESP32のIPアドレスにアクセスすると、
 * LoRaで受信した（という想定の）データと、ロガー使用率を示すプログレスバーを表示します。
 * プログレスバーはグラフィカルなものとアスキーアートの2種類を実装しています。
 * @author Gemini
 * @date 2025-08-20
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// --- Wi-Fi設定 ---
const char *ssid = "iPhone 16 Hiroki";
const char *password = "hiroki223";

// --- Webサーバーの準備 ---
AsyncWebServer server(80);

// --- 表示するデータの準備 ---
float lora_rssi = -85.5;
float lora_snr = 7.5;
String lora_message = "Hello from LoRa!";
int logger_usage = 0; // ★ロガー使用率のダミー変数を追加 (0-100)

/**
 * @brief パーセンテージからアスキーアートのプログレスバー文字列を生成する関数
 * @param percentage 0から100までのパーセンテージ
 * @return String アスキーアート文字列 (例: "[█████-----"] 50%")
 */
String createAsciiBar(int percentage)
{
  String bar = "[";
  int filled_length = map(percentage, 0, 100, 0, 20); // 20文字幅のバーに変換
  for (int i = 0; i < 20; ++i)
  {
    if (i < filled_length)
    {
      bar += "#";
    }
    else
    {
      bar += "-";
    }
  }
  bar += "] " + String(percentage) + "%";
  return bar;
}

/**
 * @brief Webページの内容を動的に生成し、ブラウザに送信する関数
 * @param request Webブラウザからのリクエスト情報を持つオブジェクト
 */
void handleRoot(AsyncWebServerRequest *request)
{
  // HTMLコンテンツを組み立てる
  String html = "<!DOCTYPE html><html><head><title>ESP32 LoRa Monitor</title>";
  html += "<meta http-equiv=\"refresh\" content=\"0.5\">"; // 2秒ごとに更新
  // ★プログレスバーのための簡単なCSSスタイルを追加
  html += "<style>";
  html += ".progress-bar { width: 90%; background-color: #f1f1f1; border: 1px solid #ccc; } ";
  html += ".progress-bar-inner { width: " + String(logger_usage) + "%; height: 20px; background-color: #4CAF50; text-align: center; line-height: 20px; color: white; } ";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>ESP32 LoRa Data Monitor</h1>";

  // --- ★グラフィカルなプログレスバーを追加 ---
  html += "<h2>Logger Usage (Graphical)</h2>";
  html += "<div class=\"progress-bar\">";
  html += "  <div class=\"progress-bar-inner\">" + String(logger_usage) + "%</div>";
  html += "</div>";

  // --- ★アスキーアートのプログレスバーを追加 ---
  html += "<h2>Logger Usage (ASCII Art)</h2>";
  html += "<pre>" + createAsciiBar(logger_usage) + "</pre>";

  html += "<h2>LoRa Data</h2>";
  html += "<p>RSSI: " + String(lora_rssi) + " dBm</p>";
  html += "<p>SNR: " + String(lora_snr) + " dB</p>";
  html += "<p>Message: " + lora_message + "</p>";
  html += "</body></html>";

  request->send(200, "text/html", html);
}

/**
 * @brief 初期設定を行う関数
 */
void setup()
{
  Serial.begin(115200);
  Serial.println("\nWebLoRa Server Starting...");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot);
  server.begin();
  Serial.println("HTTP server started");
}

/**
 * @brief メインループ
 */
void loop()
{
  delay(1000);

  // RSSIとメッセージを更新
  lora_rssi += 0.5;
  if (lora_rssi > -50)
    lora_rssi = -90;
  lora_message = "Update " + String(millis());

  // ★ロガー使用率を5%ずつ増やし、100%を超えたら0に戻す
  logger_usage += 5;
  if (logger_usage > 100)
  {
    logger_usage = 0;
  }
}
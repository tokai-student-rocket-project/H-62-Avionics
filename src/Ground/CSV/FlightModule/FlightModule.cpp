/**
 * @file FlightModule.cpp
 * @brief 地上局（退避所）用のフライトモジュールメインプログラム (AJAX対応Webサーバー版)
 * @details
 * T-Beamデバイス上で動作し、LoRa経由で受信したテレメトリーデータを処理する。
 * 機能一覧：
 * 1. OLEDディスプレイに受信データを表示する。
 * 2. シリアルポートにデータをCSV形式で出力する。
 * 3. Wi-Fiに接続し、受信データをモダンなUIのWebページで公開する。
 *    - Webページは初回ロード後、JavaScript(AJAX)で非同期にデータのみを更新するため、画面のちらつきやスクロールリセットが発生しない。
 * @author H-62 Avionics Team (modified by Gemini)
 * @date 2025-08-20
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <MsgPacketizer.h>
#include <TaskManager.h>
#include <ArduinoJson.h>
#include "LoRaBoards.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Lib_Var.hpp>
#include <TinyGPS++.h>

// --- Wi-Fi & Webサーバー設定 ---
const char *ssid = "SSID";
const char *password = "PASSWORD";
AsyncWebServer server(80);

// --- ハードウェア設定 ---
#define BUTTON_PIN 38
const unsigned long LONG_PRESS_TIME_MS = 1000;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
int currentPage = 0;
const int NUM_PAGES = 5;
TinyGPSPlus onboardGps;

// --- 分離時間設定 ---
uint32_t separation1ProtectionTime = 22390;
uint32_t separation1ForceTime = 25390;
uint32_t separation2ProtectionTime = 84290;
uint32_t separation2ForceTime = 85290;
uint32_t landingTime = 88890;

// --- テレメトリーデータ (グローバル変数) ---
volatile float telemetryRssi = 0.0;
volatile float telemetrySnr = 0.0;
volatile char telemetryIdent = ' ';
volatile uint32_t telemetryMillis = 0;
volatile uint8_t telemetryFlightMode = 0;
volatile uint32_t telemetryFlightTime = 0;
volatile uint8_t telemetryLoggerUsage = 0;
volatile bool telemetryDoLogging = false;
volatile uint8_t telemetryFramNumber = 0;
volatile bool telemetryFlightPinIsOpen = false;
volatile bool telemetrySn3IsOn = false;
volatile bool telemetrySn4IsOn = false;
volatile bool telemetryIsLaunchMode = false;
volatile bool telemetryIsFalling = false;
volatile uint32_t telemetryUnixEpoch = 0;
volatile uint8_t telemetryFixType = 0;
volatile uint8_t telemetrySatelliteCount = 0;
volatile float telemetryLatitude = 0.0;
volatile float telemetryLongitude = 0.0;
volatile int16_t telemetryHeight = 0;
volatile int16_t telemetrySpeed = 0;
volatile uint16_t telemetryAccuracy = 0;
volatile float telemetryMotorTemperature = 0.0;
volatile float telemetryMcuTemperature = 0.0;
volatile float telemetryInputVoltage = 0.0;
volatile float telemetryCurrent = 0.0;
volatile float telemetryCurrentPosition = 0.0;
volatile float telemetryCurrentDesiredPosition = 0.0;
volatile float telemetryCurrentVelocity = 0.0;
volatile float telemetryMotorTemperature_SUPPLY = 0.0;
volatile float telemetryMcuTemperature_SUPPLY = 0.0;
volatile float telemetryInputVoltage_SUPPLY = 0.0;
volatile float telemetryCurrent_SUPPLY = 0.0;
volatile float telemetryCurrentPosition_SUPPLY = 0.0;
volatile float telemetryCurrentDesiredPosition_SUPPLY = 0.0;
volatile float telemetryCurrentVelocity_SUPPLY = 0.0;
volatile float telemetrySeparation1ProtectionTime = 0;
volatile float telemetrySeparation1ForceTime = 0;
volatile float telemetrySeparation2ProtectionTime = 0;
volatile float telemetrySeparation2ForceTime = 0;
volatile float telemetryLandingTime = 0;

unsigned long buttonPressTime = 0;
bool longPressSent = false;

// --- 関数プロトタイプ宣言 ---
void updateDisplay();
void sendLoRaCommand();
const char *getFlightModeString(uint8_t mode);
void handleRoot(AsyncWebServerRequest *request);
void handleData(AsyncWebServerRequest *request);

// --- Webページ生成ヘルパー関数 ---
String createStatusIndicator(const char *id, const char *label, bool isActive)
{
    String color = isActive ? "#28a745" : "#dc3545";
    String text = isActive ? "ON" : "OFF";
    return "<div class=\"status-item\">" + String(label) + ": <span id=\"" + String(id) + "_indicator\" class=\"indicator\" style=\"background-color:" + color + ";\"></span> <span id=\"" + String(id) + "_text\">" + text + "</span></div>";
}

String createProgressBar(const char *id, const char *label, int percentage)
{
    String html = "<div>" + String(label) + "</div>";
    html += "<div class=\"progress-bar\">";
    html += "<div id=\"" + String(id) + "\" class=\"progress-bar-inner\" style=\"width:" + String(percentage) + "%;\">" + String(percentage) + "%</div>";
    html += "</div>";
    return html;
}

// --- Webサーバーリクエストハンドラ ---

/** @brief ルート("/")へのリクエスト。Webページの骨格(HTML)を返す */
void handleRoot(AsyncWebServerRequest *request)
{
    String html = "<!DOCTYPE html><html lang=\"ja\"><head>";
    html += "<meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=\"1.0\" > ";
    html += "<title>H-62 Avionics Dashboard</title>";
    html += "<style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif; background-color: #f0f2f5; color: #333; margin: 0; padding: 1rem; } ";
    html += "h1, h2 { color: #1a2b48; } h1 { text-align: center; } ";
    html += " .dashboard { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 1rem; } ";
    html += " .card { background-color: #fff; border-radius: 8px; padding: 1.5rem; box-shadow: 0 2px 4px rgba(0,0,0,0.1); } ";
    html += " .card h2 { margin-top: 0; border-bottom: 2px solid #eee; padding-bottom: 0.5rem; } ";
    html += " .data-item, .status-item { margin-bottom: 0.5rem; } .data-item strong { color: #555; } ";
    html += " .indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 5px; vertical-align: middle; } ";
    html += " .progress-bar { width: 100%; background-color: #e9ecef; border-radius: .25rem; overflow: hidden; } ";
    html += " .progress-bar-inner { height: 20px; background-color: #007bff; border-radius: .25rem; text-align: center; line-height: 20px; color: white; transition: width 0.5s ease-in-out; } ";
    html += "</style></head><body>";
    html += "<h1>H-62 Avionics Dashboard</h1>";
    html += "<div class=\"dashboard\">";
    html += "<div class=\"card\"><h2><span style='font-size:1.5em;'>📡</span> Status</h2>";
    html += "<div class=\"data-item\"><strong>Flight Mode:</strong> <span id=\"flightMode\">" + String(getFlightModeString(telemetryFlightMode)) + "</span></div>";
    html += "<div class=\"data-item\"><strong>Flight Time:</strong> <span id=\"flightTime\">" + String(telemetryFlightTime / 1000.0) + "</span> s</div>";
    html += "<div class=\"data-item\"><strong>RSSI:</strong> <span id=\"rssi\">" + String(telemetryRssi) + "</span> dBm</div>";
    html += "<div class=\"data-item\"><strong>SNR:</strong> <span id=\"snr\">" + String(telemetrySnr) + "</span> dB</div>";
    html += createStatusIndicator("logging", "Logging", telemetryDoLogging);
    html += createStatusIndicator("flightpin", "Flight Pin", telemetryFlightPinIsOpen);
    html += createStatusIndicator("sn3", "SN3", telemetrySn3IsOn);
    html += createStatusIndicator("sn4", "SN4", telemetrySn4IsOn);
    html += createProgressBar("loggerUsage", "Logger Usage", telemetryLoggerUsage);
    html += "</div>";
    html += "<div class=\"card\"><h2><span style='font-size:1.5em;'>🛰️</span> GPS</h2>";
    html += "<div class=\"data-item\"><strong>Latitude:</strong> <span id=\"lat\">" + String(telemetryLatitude, 6) + "</span></div>";
    html += "<div class=\"data-item\"><strong>Longitude:</strong> <span id=\"lon\">" + String(telemetryLongitude, 6) + "</span></div>";
    html += "<div class=\"data-item\"><strong>Height:</strong> <span id=\"hgt\">" + String(telemetryHeight) + "</span> m</div>";
    html += "<div class=\"data-item\"><strong>Satellites:</strong> <span id=\"sat\">" + String(telemetrySatelliteCount) + "</span></div>";
    html += "</div>";
    html += "<div class=\"card\"><h2><span style='font-size:1.5em;'>🔧</span> Main Valve</h2>";
    html += "<div class=\"data-item\"><strong>Temperature:</strong> <span id=\"temp_main\">" + String(telemetryMotorTemperature) + "</span> C</div>";
    html += "<div class=\"data-item\"><strong>Voltage:</strong> <span id=\"volt_main\">" + String(telemetryInputVoltage) + "</span> V</div>";
    html += "<div class=\"data-item\"><strong>Current:</strong> <span id=\"curr_main\">" + String(telemetryCurrent) + "</span> A</div>";
    html += "<div class=\"data-item\"><strong>Position:</strong> <span id=\"pos_main\">" + String(telemetryCurrentPosition) + "</span> deg</div>";
    html += "</div>";
    html += "<div class=\"card\"><h2><span style='font-size:1.5em;'>🔩</span> Supply Valve</h2>";
    html += "<div class=\"data-item\"><strong>Temperature:</strong> <span id=\"temp_supply\">" + String(telemetryMotorTemperature_SUPPLY) + "</span> C</div>";
    html += "<div class=\"data-item\"><strong>Voltage:</strong> <span id=\"volt_supply\">" + String(telemetryInputVoltage_SUPPLY) + "</span> V</div>";
    html += "<div class=\"data-item\"><strong>Current:</strong> <span id=\"curr_supply\">" + String(telemetryCurrent_SUPPLY) + "</span> A</div>";
    html += "<div class=\"data-item\"><strong>Position:</strong> <span id=\"pos_supply\">" + String(telemetryCurrentPosition_SUPPLY) + "</span> deg</div>";
    html += "</div>";
    html += "</div>";
    html += "<script>";
    html += "setInterval(function() { fetch('/data').then(response => response.json()).then(data => { ";
    html += "document.getElementById('flightMode').innerText = data.flightMode;";
    html += "document.getElementById('flightTime').innerText = data.flightTime;";
    html += "document.getElementById('rssi').innerText = data.rssi;";
    html += "document.getElementById('snr').innerText = data.snr;";
    html += "document.getElementById('lat').innerText = data.lat;";
    html += "document.getElementById('lon').innerText = data.lon;";
    html += "document.getElementById('hgt').innerText = data.hgt;";
    html += "document.getElementById('sat').innerText = data.sat;";
    html += "document.getElementById('temp_main').innerText = data.temp_main;";
    html += "document.getElementById('volt_main').innerText = data.volt_main;";
    html += "document.getElementById('curr_main').innerText = data.curr_main;";
    html += "document.getElementById('pos_main').innerText = data.pos_main;";
    html += "document.getElementById('temp_supply').innerText = data.temp_supply;";
    html += "document.getElementById('volt_supply').innerText = data.volt_supply;";
    html += "document.getElementById('curr_supply').innerText = data.curr_supply;";
    html += "document.getElementById('pos_supply').innerText = data.pos_supply;";
    html += "document.getElementById('loggerUsage').style.width = data.loggerUsage + '%';";
    html += "document.getElementById('loggerUsage').innerText = data.loggerUsage + '%';";
    html += "['logging', 'flightpin', 'sn3', 'sn4'].forEach(id => { let el_ind = document.getElementById(id + '_indicator'); let el_txt = document.getElementById(id + '_text'); if(data[id]){ el_ind.style.backgroundColor = '#28a745'; el_txt.innerText = 'ON'; } else { el_ind.style.backgroundColor = '#dc3545'; el_txt.innerText = 'OFF'; } });";
    html += "}); }, 2000);";
    html += "</script>";
    html += "</body></html>";
    request->send(200, "text/html", html);
}

/** @brief /data へのリクエスト。テレメトリーデータをJSON形式で返す */
void handleData(AsyncWebServerRequest *request)
{
    StaticJsonDocument<1024> doc;
    doc["rssi"] = telemetryRssi;
    doc["snr"] = telemetrySnr;
    doc["flightMode"] = getFlightModeString(telemetryFlightMode);
    doc["flightTime"] = String(telemetryFlightTime / 1000.0);
    doc["loggerUsage"] = telemetryLoggerUsage;
    doc["logging"] = telemetryDoLogging;
    doc["flightpin"] = telemetryFlightPinIsOpen;
    doc["sn3"] = telemetrySn3IsOn;
    doc["sn4"] = telemetrySn4IsOn;
    doc["lat"] = String(telemetryLatitude, 6);
    doc["lon"] = String(telemetryLongitude, 6);
    doc["hgt"] = telemetryHeight;
    doc["sat"] = telemetrySatelliteCount;
    doc["temp_main"] = telemetryMotorTemperature;
    doc["volt_main"] = telemetryInputVoltage;
    doc["curr_main"] = telemetryCurrent;
    doc["pos_main"] = telemetryCurrentPosition;
    doc["temp_supply"] = telemetryMotorTemperature_SUPPLY;
    doc["volt_supply"] = telemetryInputVoltage_SUPPLY;
    doc["curr_supply"] = telemetryCurrent_SUPPLY;
    doc["pos_supply"] = telemetryCurrentPosition_SUPPLY;
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
}

// --- ここから下の関数は、前回のコードから変更ありません ---

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
    int barWidth = 5, barSpacing = 1, barHeight = 8;
    int startX = 128 - (5 * (barWidth + barSpacing));
    int startY = 64 - barHeight;
    for (int i = 0; i < 5; i++)
    {
        int currentBarHeight = barHeight * (i + 1) / 5;
        if (i < bars)
        {
            display.fillRect(startX + i * (barWidth + barSpacing), startY + (barHeight - currentBarHeight), barWidth, currentBarHeight, SSD1306_WHITE);
        }
    }
}

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
    display.print(F("DST: "));
    if (onboardGps.location.isValid() && telemetryLatitude != 0.0)
    {
        double distance = TinyGPSPlus::distanceBetween(onboardGps.location.lat(), onboardGps.location.lng(), telemetryLatitude, telemetryLongitude);
        if (distance < 1000)
        {
            display.print(distance, 0);
            display.println(F("m"));
        }
        else
        {
            display.print(distance / 1000.0, 1);
            display.println(F("km"));
        }
    }
    else
    {
        display.print(F("No Data..."));
    }
    display.display();
}

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

void sendLoRaCommand()
{
    const auto &packet = MsgPacketizer::encode(0xF3, separation1ProtectionTime, separation1ForceTime, separation2ProtectionTime, separation2ForceTime, landingTime);
    LoRa.beginPacket();
    LoRa.write(packet.data.data(), packet.data.size());
    LoRa.endPacket();
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
            updateDisplay();
        }
        buttonPressTime = 0;
    }
}

void setup()
{
    setupBoards();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.begin(115200);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Starting WiFi...");
    display.display();

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

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Connected!");
    display.println("IP Address:");
    display.println(WiFi.localIP());
    display.display();
    delay(2000);

    Serial.println("loraRssi,loraSnr,millis,flightMode,flightTime,loggerUsage,doLogging,framNumber,flightPinIsOpen,sn3IsOn,sn4IsOn,isLaunchMode,isFalling,unixEpoch,fixType,satelliteCount,latitude,longitude,height,speed,accuracy,motorTemperature,mcuTemperature,inputVoltage,current,currentPosition,currentDesiredPosition,currentVelocity,motorTemperature_SUPPLY,mcuTemperature_SUPPLY,inputVoltage_SUPPLY,current_SUPPLY,currentPosition_SUPPLY,currentDesiredPosition_SUPPLY,currentVelocity_SUPPLY,separation1ProtectionTime,separation1ForceTime,separation2ProtectionTime,separation2ForceTime,landingTime");

    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(924.6E6))
    {
        Serial.println("Starting LoRa failed!");
        display.clearDisplay();
        display.println("LoRa Failed!");
        display.display();
        while (1)
            ;
    }
    LoRa.setSignalBandwidth(500E3);

    // Webサーバーのハンドラを登録
    server.on("/", HTTP_GET, handleRoot);     // HTMLページ本体
    server.on("/data", HTTP_GET, handleData); // JSONデータ
    server.begin();
    Serial.println("HTTP server started");

    MsgPacketizer::subscribe(LoRa, 0x0A,
                             [](
                                 char ident, uint32_t millis, uint8_t flightMode, uint32_t flightTime, uint8_t loggerUsage,
                                 bool doLogging, uint8_t framNumber, bool flightPinIsOpen, bool sn3IsOn, bool sn4IsOn,
                                 bool isLaunchMode, bool isFalling, uint32_t unixEpoch, uint8_t fixType, uint8_t satelliteCount,
                                 float latitude, float longitude, int16_t height, int16_t speed, uint16_t accuracy,
                                 int16_t motorTemperature, int16_t mcuTemperature, uint16_t inputVoltage, int16_t current,
                                 int16_t currentPosition, int16_t currentDesiredPosition, int16_t currentVelocity,
                                 int16_t motorTemperature_SUPPLY, int16_t mcuTemperature_SUPPLY, uint16_t inputVoltage_SUPPLY,
                                 int16_t current_SUPPLY, int16_t currentPosition_SUPPLY, int16_t currentDesiredPosition_SUPPLY,
                                 int16_t currentVelocity_SUPPLY, uint32_t separation1ProtectionTime, uint32_t separation1ForceTime,
                                 uint32_t separation2ProtectionTime, uint32_t separation2ForceTime, uint32_t landingTime)
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

// --- デモモード設定 ---
#define ENABLE_DEMO_MODE // この行のコメントを外すとデモモードが有効

#ifdef ENABLE_DEMO_MODE
void run_demo_code();
#endif

void loop()
{
    Tasks.update();
    if (Serial.available())
    {
        char command = Serial.read();
        if (command == 'F')
        {
            const auto &packet = MsgPacketizer::encode(0xF1, (uint8_t)0);
            LoRa.beginPacket();
            LoRa.write(packet.data.data(), packet.data.size());
            LoRa.endPacket();
        }
        if (command == 'R')
        {
            const auto &packet = MsgPacketizer::encode(0xF2, (uint8_t)0);
            LoRa.beginPacket();
            LoRa.write(packet.data.data(), packet.data.size());
            LoRa.endPacket();
        }
        if (command == 'C')
        {
            const auto &packet = MsgPacketizer::encode(0xF3, separation1ProtectionTime, separation1ForceTime, separation2ProtectionTime, separation2ForceTime, landingTime);
            LoRa.beginPacket();
            LoRa.write(packet.data.data(), packet.data.size());
            LoRa.endPacket();
        }
    }

#ifdef ENABLE_DEMO_MODE
    run_demo_code();
#else
    if (LoRa.parsePacket())
    {
        MsgPacketizer::parse();
    }
#endif
}

#ifdef ENABLE_DEMO_MODE
/**
 * @brief デモ用のダミーデータを生成し、表示を更新する関数
 */
void run_demo_code()
{
    static unsigned long lastDemoTime = 0;
    if (millis() - lastDemoTime > 500) // 0.5秒ごとにデータを更新
    {
        lastDemoTime = millis();

        // --- ダミーデータの生成 ---
        telemetryRssi = -50 + random(0, 20);
        telemetrySnr = 8.5 + (random(0, 40) / 10.0);
        telemetryFlightMode = (telemetryFlightMode + 1) % 10; // 10種類のフライトモードを巡回
        telemetryFlightTime = millis();
        telemetryLoggerUsage = (telemetryLoggerUsage + 3) % 101;
        telemetryDoLogging = true;
        telemetryFlightPinIsOpen = (millis() / 5000) % 2 == 0;
        telemetrySn3IsOn = (millis() / 8000) % 2 == 1;
        telemetrySn4IsOn = (millis() / 12000) % 2 == 0;
        telemetryIsLaunchMode = telemetryFlightMode > 1 && telemetryFlightMode < 5;
        telemetryIsFalling = telemetryFlightMode > 5;
        telemetryUnixEpoch = 1672531200 + (millis() / 1000);
        telemetrySatelliteCount = 8 + random(0, 5);
        telemetryLatitude += 0.0001;
        telemetryLongitude += 0.0001;
        telemetryHeight = 1500 + sin(millis() / 10000.0) * 500;
        telemetrySpeed = 300 + cos(millis() / 5000.0) * 100;
        telemetryMotorTemperature = 45.5 + sin(millis() / 3000.0) * 5;
        telemetryInputVoltage = 12.1 + cos(millis() / 8000.0) * 0.5;
        telemetryCurrent = 2.3 + sin(millis() / 2000.0);
        telemetryCurrentPosition = (float)(((int)telemetryCurrentPosition + 5) % 360);
        telemetryMotorTemperature_SUPPLY = 35.2 + cos(millis() / 4000.0) * 3;
        telemetryInputVoltage_SUPPLY = 5.0 - sin(millis() / 10000.0) * 0.2;
        telemetryCurrent_SUPPLY = 0.8 + cos(millis() / 1500.0) * 0.2;
        telemetryCurrentPosition_SUPPLY = (float)(((int)telemetryCurrentPosition_SUPPLY + 10) % 360);

        // --- 表示とシリアル出力の更新 ---
        updateDisplay();

        // シリアルへのCSV出力 (subscribeコールバック内の処理を模倣)
        Serial.print(telemetryFlightTime / 1000.0);
        Serial.print(",");
        Serial.print('D');
        Serial.print(","); // Identを'D' for Demoに
        Serial.print(telemetryLoggerUsage);
        Serial.print(",");
        Serial.print(telemetryDoLogging);
        Serial.print(",");
        Serial.print(telemetryFramNumber);
        Serial.print(",");
        Serial.print(telemetryFlightMode);
        Serial.print(",");
        Serial.print(telemetryFlightTime / 1000.0);
        Serial.print(",");
        Serial.print(telemetryRssi);
        Serial.print(",");
        Serial.print(telemetrySnr);
        Serial.print(",");
        Serial.print(telemetryFlightPinIsOpen);
        Serial.print(",");
        Serial.print(telemetrySn3IsOn);
        Serial.print(",");
        Serial.print(telemetrySn4IsOn);
        Serial.print(",");
        Serial.print(telemetryIsLaunchMode);
        Serial.print(",");
        Serial.print(telemetryIsFalling);
        Serial.print(",");
        Serial.print(telemetryUnixEpoch);
        Serial.print(",");
        Serial.print(telemetryFixType);
        Serial.print(",");
        Serial.print(telemetrySatelliteCount);
        Serial.print(",");
        Serial.print(telemetryLatitude, 6);
        Serial.print(",");
        Serial.print(telemetryLongitude, 6);
        Serial.print(",");
        Serial.print(telemetryHeight);
        Serial.print(",");
        Serial.print(telemetrySpeed);
        Serial.print(",");
        Serial.print(telemetryAccuracy);
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
        Serial.print(telemetrySeparation1ProtectionTime);
        Serial.print(",");
        Serial.print(telemetrySeparation1ForceTime);
        Serial.print(",");
        Serial.print(telemetrySeparation2ProtectionTime);
        Serial.print(",");
        Serial.print(telemetrySeparation2ForceTime);
        Serial.print(",");
        Serial.println(telemetryLandingTime);
        Serial.flush();
    }
}
#endif

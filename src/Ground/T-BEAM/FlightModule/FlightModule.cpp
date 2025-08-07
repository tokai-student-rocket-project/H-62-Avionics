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

// テレメトリー
float telemetryRssi = 0.0;
float telemetrySnr = 0.0;
float telemetryLatitude = 0.0;
float telemetryLongitude = 0.0;
float telemetryMainPosition = 0.0;
float telemetrySupplyPosition = 0.0;
float telemetryFlightTime = 0.0;

unsigned long buttonPressTime = 0;
bool longPressSent = false;

void updateDisplay();
void sendLoRaCommand();
void loraRssiBar();

void displayPage0()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("--- GPS ---"));

  display.setCursor(70, 0);
  display.println();
  display.println();

  loraRssiBar();

  display.print(" H-62 Lat: ");
  display.println(telemetryLatitude, 6);
  display.print("      Lng: ");
  display.println(telemetryLongitude, 6);
  display.println("=====================");

  if (onbordGps.location.isValid())
  {
    display.print(F(" Now Lat: "));
    display.println(onbordGps.location.lat(), 6);
    display.print(F("     Lng: "));
    display.println(onbordGps.location.lng(), 6);
    display.print(F("      Sat: "));
    display.println(onbordGps.satellites.value());
  }
  else
  {
    display.println(F(" Now Lat: --.------"));
    display.println(F("     Lng: --.------"));
    display.println(F("     Sat: --"));
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
  display.println();

  loraRssiBar();

  display.print(F("  RSSI: "));
  display.print(telemetryRssi);
  display.println(F(" dBm"));

  display.print(F("   SNR: "));
  display.print(telemetrySnr);
  display.println(F(" dB"));

  display.print(F("  MAIN: "));
  display.print(telemetryMainPosition);
  display.println(F(" deg"));

  display.print(F("SUPPLY: "));
  display.print(telemetrySupplyPosition);
  display.println(F(" deg"));

  display.display();
}

void loraRssiBar()
{
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

  Tasks["update-display"]->startOnceAfterMsec(3000);
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
  if (!LoRa.begin(925.6E6))
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
                               uint32_t separation1ProtectionTime,
                               uint32_t separation1ForceTime,
                               uint32_t separation2ProtectionTime,
                               uint32_t separation2ForceTime,
                               uint32_t landingTime)
                           {
                             Serial.print(">LoRa_RSSI_dBm: ");
                             telemetryRssi = LoRa.packetRssi();
                             Serial.println(telemetryRssi);

                             telemetrySnr = LoRa.packetSnr();
                             Serial.print(">LoRa_SNR_dBm: ");
                             Serial.println(telemetrySnr);

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
                             telemetryFlightTime = (float)flightTime / 1000.0;

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
                             telemetryMainPosition = (float)currentPosition / 100.0;
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
                             telemetrySupplyPosition = (float)currentDesiredPosition_SUPPLY / 100.0;
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
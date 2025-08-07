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
float flightTime = 0.0;

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
    display.println(F("--- MAIN ---"));

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

    display.display();

    // int rssi = LoRa.packetRssi();
    // int bars = 0;
    // if (rssi > -95.8) {
    //     bars = 5;
    // } else if (rssi > -101.6) {
    //     bars = 4;
    // } else if (rssi > -107.4) {
    //     bars = 3;
    // } else if (rssi > -113.2) {
    //     bars = 2;
    // } else {
    //     bars = 1;
    // }

    // int barWidth = 5;
    // int barSpacing = 1;
    // int startX = 100;
    // int startY = 0;
    // int barHeight = 8;

    // for (int i = 0; i < 5; i++) {
    //     int currentBarHeight = barHeight * (i + 1) / 5;
    //     if (i < bars) {
    //         display.fillRect(startX + i * (barWidth + barSpacing), startY + (barHeight - currentBarHeight), barWidth, currentBarHeight, SSD1306_WHITE);
    //     }
    // }

    //     display.print("Flight Time:");
    //     display.println((float)flightTime / 1000.0);

    // if (onbordGps.location.isValid())
    // {
    //     display.print(" H-62 Lat:");
    //     display.println(receiveLatitude, 6);
    //     display.print(" H-62 Lng:");
    //     display.println(receiveLongtitude, 6);
    //     display.print(F("TBEAM Lat: "));
    //     display.println(onbordGps.location.lat(), 6);
    //     display.print(F("TBEAM Lng: "));
    //     display.println(onbordGps.location.lng(), 6);
    //     display.print(F("SAT: "));
    //     display.println(onbordGps.satellites.value());
    // }
    // else
    // {
    //     display.println(F("No valid GPS data."));
    // }
    // display.display();
}

void displayPage1()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("--- Sub ---"));
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

    display.print("Flight Time:");
    display.println((float)flightTime / 1000.0);
    display.print(" H-62 Lat:");
    display.println(receiveLatitude, 6);
    display.print(" H-62 Lng:");
    display.println(receiveLongtitude, 6);
    // display.println("Need to change the frequency band");

    if (onbordGps.location.isValid())
    {
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

    // display.setCursor(70, 0);
    // display.println();

    // int rssi = LoRa.packetRssi();
    // int bars = 0;
    // if (rssi > -95.8) {
    //     bars = 5;
    // } else if (rssi > -101.6) {
    //     bars = 4;
    // } else if (rssi > -107.4) {
    //     bars = 3;
    // } else if (rssi > -113.2) {
    //     bars = 2;
    // } else {
    //     bars = 1;
    // }

    // int barWidth = 5;
    // int barSpacing = 1;
    // int startX = 100;
    // int startY = 0;
    // int barHeight = 8;

    // for (int i = 0; i < 5; i++) {
    //     int currentBarHeight = barHeight * (i + 1) / 5;
    //     if (i < bars) {
    //         display.fillRect(startX + i * (barWidth + barSpacing), startY + (barHeight - currentBarHeight), barWidth, currentBarHeight, SSD1306_WHITE);
    //     }
    // }
    // display.print(F("RSSI: "));
    // display.print(lastRssi);
    // display.println(F(" dBm"));
    // display.print(F("SNR:  "));
    // display.print(lastSnr);
    // display.println(F(" dB"));
    // display.print(F("ALT: "));
    // display.print(altitude);
    // display.println(F(" m"));
    // display.print(F("BAT: "));
    // display.print(batteryVoltage);
    // display.println(F(" V"));
    // display.print(F("EXT: "));
    // display.print(externalVoltage);
    // display.println(F(" V"));

    // display.display();
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

    Tasks["update-display"]->startOnceAfterMsec(2000);
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
            // 長押しした際の動作はここに実装
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
    if (!LoRa.begin(924.2E6)) // 一時的にFlightModuleの周波数
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
                                 uint32_t millis,
                                 char ident,
                                 uint8_t loggerUsage,
                                 bool doLogging,
                                 uint8_t framNumber,
                                 int16_t accelerationX_mps2,
                                 int16_t accelerationY_mps2,
                                 int16_t accelerationZ_mps2,
                                 int16_t accelerationNorm_mps2,
                                 int16_t roll_deg,
                                 int16_t pitch_deg,
                                 int16_t yaw_deg,
                                 int16_t forceX_N,
                                 int16_t jerkX_mps3,
                                 int16_t altitude_m,
                                 int16_t verticalSpeed_mps,
                                 int16_t estimated,
                                 int16_t apogee,
                                 int16_t externalVoltage_V,
                                 int16_t batteryVoltage_V,
                                 int16_t busVoltage_V,
                                 int16_t externalCurrent_mA,
                                 int16_t batteryCurrent_mA,
                                 int16_t busCurrent_mA,
                                 int16_t externalPower_W,
                                 int16_t batteryPower_W,
                                 int16_t busPower_W,
                                 int16_t externalDieTemperature_C,
                                 int16_t batteryDieTemperature_C,
                                 int16_t busDieTemperature_C)
                             {
                                 lastRssi = LoRa.packetRssi();
                                 lastSnr = LoRa.packetSnr();

                                 if (currentPage == 1)
                                 {
                                     updateDisplay();
                                 }

                                 Serial.print(">LoRa_RSSI_dBm: ");
                                 Serial.println(lastRssi);
                                 Serial.print(">LoRa_SNR_dBm: ");
                                 Serial.println(lastSnr);
                                 Serial.print(">upTime_sec: ");
                                 Serial.println((float)millis / 1000);
                                 Serial.print(">doLogging_bool: ");
                                 Serial.println(doLogging);
                                 Serial.print(">loggerUsage_%: ");
                                 Serial.println(loggerUsage);
                                 Serial.print(">framNumber: ");
                                 Serial.println(framNumber);
                                 Serial.print(">acceleration_mps2_norm: ");
                                 Serial.println((float)accelerationNorm_mps2 / 10.0);
                                 Serial.print(">acceleration_mps2_x: ");
                                 Serial.println((float)accelerationX_mps2 / 10.0);
                                 Serial.print(">acceleration_mps2_y: ");
                                 Serial.println((float)accelerationY_mps2 / 10.0);
                                 Serial.print(">acceleration_mps2_z: ");
                                 Serial.println((float)accelerationZ_mps2 / 10.0);
                                 Serial.print(">orientation_deg_roll: ");
                                 Serial.println((float)roll_deg / 10.0);
                                 Serial.print(">orientation_deg_pitch: ");
                                 Serial.println((float)pitch_deg / 10.0);
                                 Serial.print(">orientation_deg_yaw: ");
                                 Serial.println((float)yaw_deg / 10.0);
                                 Serial.print(">forceX_N: ");
                                 Serial.println((float)forceX_N / 10.0);
                                 Serial.print(">jerkX_mps3: ");
                                 Serial.println((float)jerkX_mps3 / 10.0);
                                 Serial.print(">altitude_m: ");
                                 Serial.println((float)altitude_m / 10.0);
                                 altitude = (float)altitude_m / 10.0;
                                 Serial.print(">vertiaclSpeed_mps: ");
                                 Serial.println((float)verticalSpeed_mps / 10.0);
                                 Serial.print(">apogee_m: ");
                                 Serial.println((float)apogee / 10.0);
                                 Serial.print(">estimated_s: ");
                                 Serial.println((float)estimated / 10.0);
                                 Serial.print(">externalVoltage_V: ");
                                 Serial.println((float)externalVoltage_V / 100.0);
                                 externalVoltage = (float)externalVoltage_V / 100.0;
                                 Serial.print(">batteryVoltage_V: ");
                                 Serial.println((float)batteryVoltage_V / 100.0);
                                 batteryVoltage = (float)batteryVoltage_V / 100.0;
                                 Serial.print(">busVoltage_V: ");
                                 Serial.println((float)busVoltage_V / 100.0);
                                 Serial.print(">externalCurrent_mA: ");
                                 Serial.println((float)externalCurrent_mA / 100.0);
                                 Serial.print(">batteryCurrent_mA: ");
                                 Serial.println((float)batteryCurrent_mA / 100.0);
                                 Serial.print(">busCurrent_mA: ");
                                 Serial.println((float)busCurrent_mA / 100.0);
                                 Serial.print(">externalPower_W: ");
                                 Serial.println((float)externalPower_W / 10.0);
                                 Serial.print(">batteryPower_W");
                                 Serial.println((float)batteryPower_W / 10.0);
                                 Serial.print(">busPower_W: ");
                                 Serial.println((float)busPower_W / 10.0);
                                 Serial.print(">groundTemperature_degC: ");
                                 Serial.println((float)externalDieTemperature_C / 10.0);
                                 Serial.print(">batteryDieTemperature_degC: ");
                                 Serial.println((float)batteryDieTemperature_C / 10.0);
                                 Serial.print(">busDieTemperature_degC: ");
                                 Serial.println((float)busDieTemperature_C / 10.0);
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
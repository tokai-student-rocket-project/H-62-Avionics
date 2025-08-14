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
const int NUM_PAGES = 9;

TinyGPSPlus onbordGps;

// --- テレメトリーデータ ---
float telemetryRssi = 0.0;
float telemetrySnr = 0.0;
float telemetryUptime = 0.0;
char telemetryIdent = ' ';
uint8_t telemetryLoggerUsage = 0;
bool telemetryDoLogging = false;
uint8_t telemetryFramNumber = 0;
float telemetryAccelerationX = 0.0;
float telemetryAccelerationY = 0.0;
float telemetryAccelerationZ = 0.0;
float telemetryAccelerationNorm = 0.0;
float telemetryRoll = 0.0;
float telemetryPitch = 0.0;
float telemetryYaw = 0.0;
float telemetryForceX = 0.0;
float telemetryJerkX = 0.0;
float telemetryAltitude = 0.0;
float telemetryVerticalSpeed = 0.0;
float telemetryEstimated = 0.0;
float telemetryApogee = 0.0;
float telemetryExternalVoltage = 0.0;
float telemetryBatteryVoltage = 0.0;
float telemetryBusVoltage = 0.0;
float telemetryExternalCurrent = 0.0;
float telemetryBatteryCurrent = 0.0;
float telemetryBusCurrent = 0.0;
float telemetryExternalPower = 0.0;
float telemetryBatteryPower = 0.0;
float telemetryBusPower = 0.0;
float telemetryExternalDieTemperature = 0.0;
float telemetryBatteryDieTemperature = 0.0;
float telemetryBusDieTemperature = 0.0;

unsigned long buttonPressTime = 0;
bool longPressSent = false;

void updateDisplay();
void sendLoRaCommand();
void loraRssiBar();

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

    int barWidth = 5;
    int barSpacing = 1;
    int barHeight = 8;
    int startX = 128 - (5 * (barWidth + barSpacing)); // Position to the far right
    int startY = 64 - barHeight;                      // Position to the bottom

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
    display.print(F("RSSI   : "));
    display.print(telemetryRssi);
    display.println(F("dBm"));
    display.print(F("SNR    : "));
    display.print(telemetrySnr);
    display.println(F("dB"));
    display.print(F("UPTIME : "));
    display.print(telemetryUptime);
    display.println(F("s"));
    display.print(F("LOG    : "));
    display.println(telemetryDoLogging ? "ON" : "OFF");
    display.print(F("MEMORY : "));
    display.print(telemetryLoggerUsage);
    display.println(F("%"));
    display.print(F("FRAM: "));
    display.println(telemetryFramNumber);
    display.display();
}

void displayPage1()
{
    displayHeader(" Altitude -");
    display.setCursor(0, 10);
    display.print(F("ALT: "));
    display.print(telemetryAltitude);
    display.println(F("m"));
    display.print(F("V/S: "));
    display.print(telemetryVerticalSpeed);
    display.println(F("m/s"));
    display.print(F("APO: "));
    display.print(telemetryApogee);
    display.println(F("m"));
    display.print(F("EST: "));
    display.print(telemetryEstimated);
    display.println(F("s"));
    display.display();
}

void displayPage2()
{
    displayHeader(" Accel [m/s2] -");
    display.setCursor(0, 10);
    display.print(F("X: "));
    display.println(telemetryAccelerationX);
    display.print(F("Y: "));
    display.println(telemetryAccelerationY);
    display.print(F("Z: "));
    display.println(telemetryAccelerationZ);
    display.print(F("N: "));
    display.println(telemetryAccelerationNorm);
    display.display();
}

void displayPage3()
{
    displayHeader(" Orientation [deg] -");
    display.setCursor(0, 10);
    display.print(F("ROLL : "));
    display.println(telemetryRoll);
    display.print(F("PITCH: "));
    display.println(telemetryPitch);
    display.print(F("YAW  : "));
    display.println(telemetryYaw);
    display.display();
}

void displayPage4()
{
    displayHeader(" Bus Power -");
    display.setCursor(0, 10);
    display.print(F("V: "));
    display.print(telemetryBusVoltage);
    display.println(F(" V"));
    display.print(F("I: "));
    display.print(telemetryBusCurrent);
    display.println(F(" mA"));
    display.print(F("P: "));
    display.print(telemetryBusPower);
    display.println(F(" W"));
    display.display();
}

void displayPage5()
{
    displayHeader(" Battery Power -");
    display.setCursor(0, 10);
    display.print(F("V: "));
    display.print(telemetryBatteryVoltage);
    display.println(F(" V"));
    display.print(F("I: "));
    display.print(telemetryBatteryCurrent);
    display.println(F(" mA"));
    display.print(F("P: "));
    display.print(telemetryBatteryPower);
    display.println(F(" W"));
    display.display();
}

void displayPage6()
{
    displayHeader(" External Power -");
    display.setCursor(0, 10);
    display.print(F("V: "));
    display.print(telemetryExternalVoltage);
    display.println(F(" V"));
    display.print(F("I: "));
    display.print(telemetryExternalCurrent);
    display.println(F(" mA"));
    display.print(F("P: "));
    display.print(telemetryExternalPower);
    display.println(F(" W"));
    display.display();
}

void displayPage7()
{
    displayHeader(" Temperature [C] -");
    display.setCursor(0, 10);
    display.print(F("BUS: "));
    display.println(telemetryBusDieTemperature);
    display.print(F("BAT: "));
    display.println(telemetryBatteryDieTemperature);
    display.print(F("EXT: "));
    display.println(telemetryExternalDieTemperature);
    display.display();
}

void displayPage8()
{
    displayHeader(" Onboard GPS -");
    display.setCursor(0, 10);
    if (onbordGps.location.isValid())
    {
        display.print(F("LAT: "));
        display.println(onbordGps.location.lat(), 6);
        display.print(F("LNG: "));
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
    case 5:
        displayPage5();
        break;
    case 6:
        displayPage6();
        break;
    case 7:
        displayPage7();
        break;
    case 8:
        displayPage8();
        break;
    }
}

void sendLoRaCommand()
{
    Serial.println("Sending LoRa Command...");
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

    if (currentPage == 8)
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
            // 長押し時の動作
            longPressSent = true;
        }
    }
    else
    {
        if (buttonPressTime != 0 && !longPressSent)
        {
            // 短押し時の動作
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
    if (!LoRa.begin(924.2E6))
    {
        Serial.println("Starting LoRa failed!");
        display.clearDisplay();
        display.println("LoRa Failed!");
        display.display();
        while (1)
            ;
    }
    LoRa.setSignalBandwidth(500E3);

    // Print CSV header
    Serial.println("millis,ident,loggerUsage,doLogging,framNumber,accelerationX_mps2,accelerationY_mps2,accelerationZ_mps2,accelerationNorm_mps2,roll_deg,pitch_deg,yaw_deg,forceX_N,jerkX_mps3,altitude_m,verticalSpeed_mps,estimated,apogee,externalVoltage_V,batteryVoltage_V,busVoltage_V,externalCurrent_A,batteryCurrent_A,busCurrent_A,externalPower_W,batteryPower_W,busPower_W,externalDieTemperature_C,batteryDieTemperature_C,busDieTemperature_C,rssi,snr");

    MsgPacketizer::subscribe(LoRa, 0x0A,
                             [](
                                 uint32_t millis, char ident, uint8_t loggerUsage, bool doLogging, uint8_t framNumber,
                                 int16_t accelerationX_mps2, int16_t accelerationY_mps2, int16_t accelerationZ_mps2, int16_t accelerationNorm_mps2,
                                 int16_t roll_deg, int16_t pitch_deg, int16_t yaw_deg,
                                 int16_t forceX_N, int16_t jerkX_mps3,
                                 int16_t altitude_m, int16_t verticalSpeed_mps, int16_t estimated, int16_t apogee,
                                 int16_t externalVoltage_V, int16_t batteryVoltage_V, int16_t busVoltage_V,
                                 int16_t externalCurrent_mA, int16_t batteryCurrent_mA, int16_t busCurrent_mA,
                                 int16_t externalPower_W, int16_t batteryPower_W, int16_t busPower_W,
                                 int16_t externalDieTemperature_C, int16_t batteryDieTemperature_C, int16_t busDieTemperature_C)
                             {
                                 telemetryRssi = LoRa.packetRssi();
                                 telemetrySnr = LoRa.packetSnr();
                                 telemetryUptime = (float)millis / 1000.0;
                                 telemetryIdent = ident;
                                 telemetryLoggerUsage = loggerUsage;
                                 telemetryDoLogging = doLogging;
                                 telemetryFramNumber = framNumber;
                                 telemetryAccelerationX = (float)accelerationX_mps2 / 10.0;
                                 telemetryAccelerationY = (float)accelerationY_mps2 / 10.0;
                                 telemetryAccelerationZ = (float)accelerationZ_mps2 / 10.0;
                                 telemetryAccelerationNorm = (float)accelerationNorm_mps2 / 10.0;
                                 telemetryRoll = (float)roll_deg / 10.0;
                                 telemetryPitch = (float)pitch_deg / 10.0;
                                 telemetryYaw = (float)yaw_deg / 10.0;
                                 telemetryForceX = (float)forceX_N / 10.0;
                                 telemetryJerkX = (float)jerkX_mps3 / 10.0;
                                 telemetryAltitude = (float)altitude_m / 10.0;
                                 telemetryVerticalSpeed = (float)verticalSpeed_mps / 10.0;
                                 telemetryEstimated = (float)estimated / 10.0;
                                 telemetryApogee = (float)apogee / 10.0;
                                 telemetryExternalVoltage = (float)externalVoltage_V / 100.0;
                                 telemetryBatteryVoltage = (float)batteryVoltage_V / 100.0;
                                 telemetryBusVoltage = (float)busVoltage_V / 100.0;
                                 telemetryExternalCurrent = (float)externalCurrent_mA / 100.0;
                                 telemetryBatteryCurrent = (float)batteryCurrent_mA / 100.0;
                                 telemetryBusCurrent = (float)busCurrent_mA / 100.0;
                                 telemetryExternalPower = (float)externalPower_W / 10.0;
                                 telemetryBatteryPower = (float)batteryPower_W / 10.0;
                                 telemetryBusPower = (float)busPower_W / 10.0;
                                 telemetryExternalDieTemperature = (float)externalDieTemperature_C / 10.0;
                                 telemetryBatteryDieTemperature = (float)batteryDieTemperature_C / 10.0;
                                 telemetryBusDieTemperature = (float)busDieTemperature_C / 10.0;

                                 updateDisplay();

                                 // CSV output for Teraterm
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
                                 Serial.print(telemetryAccelerationX);
                                 Serial.print(",");
                                 Serial.print(telemetryAccelerationY);
                                 Serial.print(",");
                                 Serial.print(telemetryAccelerationZ);
                                 Serial.print(",");
                                 Serial.print(telemetryAccelerationNorm);
                                 Serial.print(",");
                                 Serial.print(telemetryRoll);
                                 Serial.print(",");
                                 Serial.print(telemetryPitch);
                                 Serial.print(",");
                                 Serial.print(telemetryYaw);
                                 Serial.print(",");
                                 Serial.print(telemetryForceX);
                                 Serial.print(",");
                                 Serial.print(telemetryJerkX);
                                 Serial.print(",");
                                 Serial.print(telemetryAltitude);
                                 Serial.print(",");
                                 Serial.print(telemetryVerticalSpeed);
                                 Serial.print(",");
                                 Serial.print(telemetryEstimated);
                                 Serial.print(",");
                                 Serial.print(telemetryApogee);
                                 Serial.print(",");
                                 Serial.print(telemetryExternalVoltage);
                                 Serial.print(",");
                                 Serial.print(telemetryBatteryVoltage);
                                 Serial.print(",");
                                 Serial.print(telemetryBusVoltage);
                                 Serial.print(",");
                                 Serial.print(telemetryExternalCurrent);
                                 Serial.print(",");
                                 Serial.print(telemetryBatteryCurrent);
                                 Serial.print(",");
                                 Serial.print(telemetryBusCurrent);
                                 Serial.print(",");
                                 Serial.print(telemetryExternalPower);
                                 Serial.print(",");
                                 Serial.print(telemetryBatteryPower);
                                 Serial.print(",");
                                 Serial.print(telemetryBusPower);
                                 Serial.print(",");
                                 Serial.print(telemetryExternalDieTemperature);
                                 Serial.print(",");
                                 Serial.print(telemetryBatteryDieTemperature);
                                 Serial.print(",");
                                 Serial.print(telemetryBusDieTemperature);
                                 Serial.print(",");
                                 Serial.print(telemetryRssi);
                                 Serial.print(",");
                                 Serial.println(telemetrySnr);

                                 //  // Teleplot用にシリアル出力
                                 //  Serial.print(">LoRa_RSSI_dBm:");
                                 //  Serial.println(telemetryRssi);
                                 //  Serial.print(">LoRa_SNR_dBm:");
                                 //  Serial.println(telemetrySnr);
                                 //  Serial.print(">upTime_sec:");
                                 //  Serial.println(telemetryUptime);
                                 //  Serial.print(">ident:");
                                 //  Serial.println(telemetryIdent);
                                 //  Serial.print(">doLogging_bool:");
                                 //  Serial.println(telemetryDoLogging);
                                 //  Serial.print(">loggerUsage_%:");
                                 //  Serial.println(telemetryLoggerUsage);
                                 //  Serial.print(">framNumber:");
                                 //  Serial.println(telemetryFramNumber);
                                 //  Serial.print(">acceleration_mps2_x:");
                                 //  Serial.println(telemetryAccelerationX);
                                 //  Serial.print(">acceleration_mps2_y:");
                                 //  Serial.println(telemetryAccelerationY);
                                 //  Serial.print(">acceleration_mps2_z:");
                                 //  Serial.println(telemetryAccelerationZ);
                                 //  Serial.print(">acceleration_mps2_norm:");
                                 //  Serial.println(telemetryAccelerationNorm);
                                 //  Serial.print(">orientation_deg_roll:");
                                 //  Serial.println(telemetryRoll);
                                 //  Serial.print(">orientation_deg_pitch:");
                                 //  Serial.println(telemetryPitch);
                                 //  Serial.print(">orientation_deg_yaw:");
                                 //  Serial.println(telemetryYaw);
                                 //  Serial.print(">forceX_N:");
                                 //  Serial.println(telemetryForceX);
                                 //  Serial.print(">jerkX_mps3:");
                                 //  Serial.println(telemetryJerkX);
                                 //  Serial.print(">altitude_m:");
                                 //  Serial.println(telemetryAltitude);
                                 //  Serial.print(">verticalSpeed_mps:");
                                 //  Serial.println(telemetryVerticalSpeed);
                                 //  Serial.print(">apogee_m:");
                                 //  Serial.println(telemetryApogee);
                                 //  Serial.print(">estimated_s:");
                                 //  Serial.println(telemetryEstimated);
                                 //  Serial.print(">externalVoltage_V:");
                                 //  Serial.println(telemetryExternalVoltage);
                                 //  Serial.print(">batteryVoltage_V:");
                                 //  Serial.println(telemetryBatteryVoltage);
                                 //  Serial.print(">busVoltage_V:");
                                 //  Serial.println(telemetryBusVoltage);
                                 //  Serial.print(">externalCurrent_A:");
                                 //  Serial.println(telemetryExternalCurrent);
                                 //  Serial.print(">batteryCurrent_A:");
                                 //  Serial.println(telemetryBatteryCurrent);
                                 //  Serial.print(">busCurrent_A:");
                                 //  Serial.println(telemetryBusCurrent);
                                 //  Serial.print(">externalPower_W:");
                                 //  Serial.println(telemetryExternalPower);
                                 //  Serial.print(">batteryPower_W:");
                                 //  Serial.println(telemetryBatteryPower);
                                 //  Serial.print(">busPower_W:");
                                 //  Serial.println(telemetryBusPower);
                                 //  Serial.print(">externalDieTemperature_C:");
                                 //  Serial.println(telemetryExternalDieTemperature);
                                 //  Serial.print(">batteryDieTemperature_C:");
                                 //  Serial.println(telemetryBatteryDieTemperature);
                                 //  Serial.print(">busDieTemperature_C:");
                                 //  Serial.println(telemetryBusDieTemperature);
                                 //  Serial.println();
                                 //  Serial.flush();
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

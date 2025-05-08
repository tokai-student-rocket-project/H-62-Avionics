#include <Arduino.h>
#include "Lib_B3msc1170a.hpp"
#include "TaskManager.h"

// Seeed RP2040 の2番ピン（EN_PIN）はType-C端子を上部にした場合，右側，一番下から2番目のピン 
// ライブラリによってピン番号が変化


B3MSC1170A mainValve;
void openMainValve() {
    mainValve.setPosition(0x01, 20000, 20000); // 20秒かけて200度へ
}

void closeMainValve() {
    mainValve.setPosition(0x01, 0, 0); // 最速で0度へ
}

void readMainValveData() {
    Serial.println("Main");
    Serial.print("CurrentPosition: ");
    Serial.println(mainValve.readCurrentPosition(0x01));
    Serial.print("MCUTemperature: ");
    Serial.println(mainValve.readMcuTemperature(0x01));
    Serial.print("MotorTemperature: ");
    Serial.println(mainValve.readMotorTemperature(0x01));
    Serial.print("Voltage: ");
    Serial.println(mainValve.readVoltage(0x01));
    Serial.print("Current: ");
    Serial.println(mainValve.readCurrent(0x01));
    Serial.println("=====");
}

B3MSC1170A supplyValve;
void openSupplyValve() {
    supplyValve.setPosition(0x02, -20000, 20000); // 20秒かけて-200度へ
}

void closeSupplyValve() {
    supplyValve.setPosition(0x02, 0, 0); // 最速で0度へ
}

void readSupplyValve() {
    Serial.println("Supply");
    Serial.print("CurrentPosition: ");
    Serial.println(mainValve.readCurrentPosition(0x02));
    Serial.print("MCUTemperature: ");
    Serial.println(mainValve.readMcuTemperature(0x02));
    Serial.print("MotorTemperature: ");
    Serial.println(mainValve.readMotorTemperature(0x02));
    Serial.print("Voltage: ");
    Serial.println(mainValve.readVoltage(0x02));
    Serial.print("Current: ");
    Serial.println(mainValve.readCurrent(0x02));
    Serial.println("=====");
}



void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1);

    delay(100);
    mainValve.initialize(0x01);
    supplyValve.initialize(0x02);

    closeMainValve(); // メイン（ID:0x01）の角度を0度に
    closeSupplyValve(); // サプライ（ID:0x02）の角度を0度に

    Tasks.add(&readMainValveData)->startFps(2); // メイン（ID:0x01）の情報を取得
    Tasks.add(&openMainValve)->startFps(1); // メイン（ID:0x01）の角度を更新
    Tasks.add(&readSupplyValve)->startFps(2); // サプライ（ID:0x02）の情報を取得
    Tasks.add(&openSupplyValve)->startFps(1); // サプライ（ID:0x02）の角度を更新
}


void loop() {
    Tasks.update();
}
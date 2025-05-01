#include <Arduino.h>
#include "Lib_B3msc1170a.hpp"

B3MSC1170A mainValve;
void openMainValve(){
    mainValve.setPosition(0x01, 9000, 0);
}

void closeMainValve(){
    mainValve.setPosition(0x01, 0, 0);
}

B3MSC1170A supplyValve;
void openSupplyValve(){
    supplyValve.setPosition(0x02, 9000, 0);
}

void closeSupplyValve(){
    supplyValve.setPosition(0x02, 0, 0);
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1);

    delay(100);
    mainValve.initialize(0x01);
    supplyValve.initialize(0x02);
}


void loop() {
    openMainValve(); // ID: 0x01 のサーボモータが90度方向へ回転
    delay(500);
    closeMainValve(); // ID: 0x01 のサーボモータが0度方向へ回転
    delay(500);
    openSupplyValve(); // ID: 0x02 のサーボモータが90度方向へ回転
    delay(500);
    closeSupplyValve(); // ID: 0x02 のサーボモータが0度方向へ回転
    delay(500);
}
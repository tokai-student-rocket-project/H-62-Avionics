#include <Arduino.h>
#include <Lib_ICM42688.hpp>

// I2C example
ICM42688 imuI2C(Wire);

// SPI example (CS=10)
ICM42688 imuSPI(10, SPI);

void setup() {
    Serial.begin(115200);
    
    Serial.println("ICM42688 Test");

    if (!imuI2C.begin(ICM42688_ADDR_GND)) {
        Serial.println("I2C Init Failed");
    }

    // if (!imuSPI.begin()) {
    //     Serial.println("SPI Init Failed");
    // }

    imuI2C.setAccelFS(ICM42688::ACCEL_FS_8G);
    imuI2C.setGyroFS(ICM42688::GYRO_FS_1000DPS);
}

void loop() {
    imuI2C.readSensor();

    Serial.print("A [g]: ");
    Serial.print(imuI2C.getAccelX()); Serial.print(", ");
    Serial.print(imuI2C.getAccelY()); Serial.print(", ");
    Serial.println(imuI2C.getAccelZ());

    Serial.print("G [dps]: ");
    Serial.print(imuI2C.getGyroX()); Serial.print(", ");
    Serial.print(imuI2C.getGyroY()); Serial.print(", ");
    Serial.println(imuI2C.getGyroZ());

    Serial.print("T [C]: ");
    Serial.println(imuI2C.getTemp());

    delay(500);
}

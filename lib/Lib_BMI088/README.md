# Lib_BMI088

Bosch BMI088 6軸IMU (加速度 + ジャイロ) を制御するためのArduinoライブラリです。
I2CおよびSPI通信に対応しています。

## 特徴

- **デュアルインターフェース**: I2CとSPIの両方をサポート
- **シンプルAPI**: `beginI2C` / `beginSPI`, `getAcceleration`, `getGyroscope`
- **設定変更**: レンジ (G, DPS) や ODR (Output Data Rate) を簡単に変更可能
- **軸リマップ**: センサの取り付け向きに合わせて出力軸を変更可能

## 接続

### I2C接続
| BMI088 Pin | MCU Pin | 備考 |
|------------|---------|------|
| VCC        | 3.3V    | |
| GND        | GND     | |
| SDA        | SDA     | |
| SCL        | SCL     | |
| CS (Accel) | VCC     | I2Cモード選択のためHigh |
| CS (Gyro)  | VCC     | I2Cモード選択のためHigh |
| SDO (Accel)| GND/VCC | I2Cアドレス設定 (GND: 0x18, VCC: 0x19) |
| SDO (Gyro) | GND/VCC | I2Cアドレス設定 (GND: 0x68, VCC: 0x69) |

### SPI接続
| BMI088 Pin | MCU Pin | 備考 |
|------------|---------|------|
| VCC        | 3.3V    | |
| GND        | GND     | |
| SDA (MOSI) | MOSI    | |
| SDO (MISO) | MISO    | |
| SCL (SCK)  | SCK     | |
| CS (Accel) | D10 etc | 加速度用CSピン |
| CS (Gyro)  | D9 etc  | ジャイロ用CSピン |

## 使用方法

### 1. インスタンス作成
```cpp
#include "Lib_BMI088.hpp"

BMI088 bmi;
```

### 2. 初期化

#### I2Cの場合
```cpp
void setup() {
    Wire.begin();
    // デフォルトアドレス (Accel: 0x18, Gyro: 0x68)
    if (!bmi.beginI2C()) {
        Serial.println("BMI088 Init Failed!");
        while(1);
    }
}
```

#### SPIの場合
```cpp
void setup() {
    // CSピンの設定
    // beginSPI内でOUTPUT/HIGHに設定されますが、念のため
    
    // SPI, Accel_CS=10, Gyro_CS=9
    if (!bmi.beginSPI(&SPI, 10, 9)) {
        Serial.println("BMI088 Init Failed!");
        while(1);
    }
}
```

### 3. データの取得
```cpp
void loop() {
    float ax, ay, az;
    float gx, gy, gz;

    bmi.getAcceleration(&ax, &ay, &az);
    bmi.getGyroscope(&gx, &gy, &gz);

    Serial.print("Accel: "); Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.println(az);
    Serial.print("Gyro: "); Serial.print(gx); Serial.print(", "); Serial.print(gy); Serial.print(", "); Serial.println(gz);
    
    delay(100);
}
```

### 4. 設定変更
```cpp
// レンジとODRの設定
bmi.setAccelConfig(BMI088::ACCEL_6G, BMI088::ACCEL_ODR_100HZ);
bmi.setGyroConfig(BMI088::GYRO_500DPS, BMI088::GYRO_ODR_100HZ_32HZ);

// 軸のリマップ (例: センサZ軸を機体X軸へ)
bmi.setAxisRemap(2, 1, 0, 1.0, 1.0, 1.0);
```

## API リファレンス

### 初期化
- `bool beginI2C(TwoWire* wire = &Wire, uint8_t accelAddr = 0x18, uint8_t gyroAddr = 0x68)`
- `bool beginSPI(SPIClass* spi, uint8_t accelCS, uint8_t gyroCS)`

### データ取得
- `void getAcceleration(float* x, float* y, float* z)`: 単位 m/s^2
- `void getGyroscope(float* x, float* y, float* z)`: 単位 dps
- `float getTemperature()`: 単位 ℃ (Accelチップ温度)

### 設定
- `void setAccelConfig(AccelRange range, AccelODR odr)`
- `void setGyroConfig(GyroRange range, GyroODR odr)`
- `void setAxisRemap(uint8_t xMap, uint8_t yMap, uint8_t zMap, float xSign, float ySign, float zSign)`

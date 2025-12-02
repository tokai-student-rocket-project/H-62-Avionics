# Lib_MPU9250

MPU9250 9軸センサ (加速度、ジャイロ、磁気) を制御するためのArduinoライブラリです。

## 概要

このライブラリは、MPU9250から加速度、角速度、磁束密度のデータを取得するためのシンプルなインターフェースを提供します。
Avionicsプロジェクトのコーディングスタイルに合わせて設計されています。

## 特徴

- シンプルなAPI (`begin`, `getAcceleration`, `getGyroscope`, `getMagnetometer`)
- AK8963 (磁気センサ) の自動初期化とキャリブレーション読み込み
- 標準的な単位系 (m/s^2, dps, uT) での出力

## 使用方法

### 1. インスタンスの作成

```cpp
#include "Lib_MPU9250.hpp"

MPU9250 mpu;
```

### 2. 初期化

`setup()` 関数内で `Wire.begin()` を呼び出し、その後に `mpu.begin()` を呼び出します。

```cpp
void setup() {
    Serial.begin(115200);
    
    // I2Cバスの初期化 (必須)
    Wire.begin();
    
    // デフォルト設定 (Accel: 16G, Gyro: 2000dps, DLPF: 41Hz, Wire: &Wire)
    if (!mpu.begin()) {
    // カスタム設定例: Wire1を使用する場合
    // if (!mpu.begin(MPU9250::AFS_16G, MPU9250::GFS_2000DPS, MPU9250::DLPF_41HZ, &Wire1)) {
        Serial.println("MPU9250 initialization failed!");
        while (1);
    }
}
```

### 3. 設定の変更

初期化後も以下の関数で設定を変更できます。

```cpp
// 加速度レンジ設定: AFS_2G, AFS_4G, AFS_8G, AFS_16G
mpu.setAccelRange(MPU9250::AFS_4G);

// ジャイロレンジ設定: GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
mpu.setGyroRange(MPU9250::GFS_500DPS);

// DLPF帯域幅設定: DLPF_250HZ, DLPF_184HZ, DLPF_92HZ, DLPF_41HZ, DLPF_20HZ, DLPF_10HZ, DLPF_5HZ
mpu.setDLPFBandwidth(MPU9250::DLPF_92HZ);

// 加速度キャリブレーション設定 (Bias X, Y, Z, Scale X, Y, Z)
// calibrateAccel() で取得した値を設定します
mpu.setAccelCalibration(0.45, 0.12, -0.30, 1.002, 0.998, 1.005);

// ジャイロキャリブレーション設定 (Bias X, Y, Z)
// calibrateGyro() で取得した値を設定します
mpu.setGyroBias(0.12, -0.05, 0.08);

// 磁気キャリブレーション設定 (Bias X, Y, Z, Scale X, Y, Z)
// calibrateMag() で取得した値を設定します
mpu.setMagCalibration(20.5, -15.2, 5.0, 1.02, 0.98, 1.01);

// 軸のリマップ (ロケットの機体軸に合わせる場合など)
// 例: センサのY軸を機体のX軸、センサのZ軸を機体のY軸、センサのX軸を機体のZ軸にする
// 引数: (出力Xへの入力軸, 出力Yへの入力軸, 出力Zへの入力軸, X符号, Y符号, Z符号)
// 入力軸: 0=X, 1=Y, 2=Z
mpu.setAxisRemap(1, 2, 0, 1.0, 1.0, 1.0);
```

### 4. データの取得

`loop()` 関数などで各データを取得します。

```cpp
void loop() {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    // 加速度 (m/s^2)
    mpu.getAcceleration(&ax, &ay, &az);
    
    // 角速度 (dps)
    mpu.getGyroscope(&gx, &gy, &gz);
    
    // 磁束密度 (uT)
    mpu.getMagnetometer(&mx, &my, &mz);
    
    // データの表示
    Serial.print("Accel: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az); Serial.println(" m/s^2");

    Serial.print("Gyro: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.print(gz); Serial.println(" dps");

    Serial.print("Mag: ");
    Serial.print(mx); Serial.print(", ");
    Serial.print(my); Serial.print(", ");
    Serial.print(mz); Serial.println(" uT");
    
    delay(100);
}
```

### 5. キャリブレーション

#### 加速度
6面キャリブレーションを行うための対話的モードを実行できます。
`setup()` 内で一度だけ呼び出し、シリアルモニタの指示に従ってください。

```cpp
void setup() {
    // ... 初期化 ...
    if (!mpu.begin()) { ... }

    // 加速度キャリブレーション実行
    // mpu.calibrateAccel(); 
}
```

#### ジャイロ
静止状態でのバイアス補正を行います。センサを平らな場所に置き、動かさない状態で実行してください。

```cpp
void setup() {
    // ... 初期化 ...
    if (!mpu.begin()) { ... }

    // ジャイロキャリブレーション実行
    // mpu.calibrateGyro();
}
```

#### 磁気
8の字補正（Hard Iron / Soft Iron）を行います。実行後、センサをあらゆる方向に8の字を描くように動かし続けてください（約15秒間）。

```cpp
void setup() {
    // ... 初期化 ...
    if (!mpu.begin()) { ... }

    // 磁気キャリブレーション実行
    // mpu.calibrateMag();
}
```

手順:
1. `calibrateAccel()`、`calibrateGyro()`、または `calibrateMag()` を呼び出すスケッチを書き込む。
2. シリアルモニタを開く。
3. 指示に従って操作する。
4. 最後に表示される設定コードをコピーし、本番用のスケッチに貼り付ける。

### 6. CSV形式での出力サンプル

データロギングやグラフ化のために、CSV形式で出力する例です。
`Serial Plotter` (Arduino IDE) や `Teleplot` (VSCode) での可視化にも便利です。

```cpp
#include <Arduino.h>
#include "Lib_MPU9250.hpp"

MPU9250 mpu;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Wire.begin();
    if (!mpu.begin()) {
        Serial.println("Initialization failed!");
        while (1);
    }

    // ヘッダー行の出力
    Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz");
}

void loop() {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getGyroscope(&gx, &gy, &gz);
    mpu.getMagnetometer(&mx, &my, &mz);

    // CSV形式で出力 (カンマ区切り)
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(",");
    
    Serial.print(mx); Serial.print(",");
    Serial.print(my); Serial.print(",");
    Serial.println(mz); // 最後は改行

    delay(20); // 50Hz
}
```

## API リファレンス

### `bool begin(Ascale ascale, Gscale gscale, DLPF dlpf, TwoWire* wire)`
センサを初期化します。
- **引数**:
    - `ascale`: 加速度レンジ (デフォルト: `AFS_16G`)
    - `gscale`: ジャイロレンジ (デフォルト: `GFS_2000DPS`)
    - `dlpf`: DLPF帯域幅 (デフォルト: `DLPF_41HZ`)
    - `wire`: I2Cインターフェース (デフォルト: `&Wire`)
- **戻り値**: 成功時は `true`、失敗時は `false`。
- **注意**: 事前に `Wire.begin()` (または `wire->begin()`) を呼び出す必要があります。

### 設定関数
- `void setAccelRange(Ascale ascale)`
- `void setGyroRange(Gscale gscale)`
- `void setDLPFBandwidth(DLPF dlpf)`
- `void setAxisRemap(uint8_t xMap, uint8_t yMap, uint8_t zMap, float xSign, float ySign, float zSign)`

### I2C直接アクセス
- `void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)`
- `uint8_t readByte(uint8_t address, uint8_t subAddress)`
- `void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)`

### `void getAcceleration(float* x, float* y, float* z)`
加速度を取得します。
- **単位**: m/s^2 (メートル毎秒毎秒)
- **範囲**: +/- 16G

### `void getGyroscope(float* x, float* y, float* z)`
角速度を取得します。
- **単位**: dps (degree per second)
- **範囲**: +/- 2000 dps

### `void getMagnetometer(float* x, float* y, float* z)`
磁束密度を取得します。
- **単位**: uT (マイクロテスラ)

## 接続

| MPU9250 Pin | MCU Pin |
|-------------|---------|
| VCC         | 3.3V    |
| GND         | GND     |
| SDA         | SDA     |
| SCL         | SCL     |

## 注意事項

- I2Cアドレスは `0x69` (MPU9250, AD0=High) と `0x0C` (AK8963) に設定されています。
- 磁気センサの軸はチップの物理的な配置に依存します。必要に応じて座標変換を行ってください。

## トラブルシューティング

### AK8963 (磁気センサ) が見つからない場合
`begin()` 内で `AK8963 WHO_AM_I: 0xFF` となる場合、MPU9250の内部I2Cマスター機能が有効になっている可能性があります。
本ライブラリは `begin()` 内で自動的に以下の処理を行い、AK8963へのアクセスを確立します。
1. `USER_CTRL` レジスタでI2Cマスターを無効化
2. `INT_PIN_CFG` レジスタでBypass Modeを有効化

### デバッグモード
現在、ライブラリはデバッグモードで動作しており、シリアルモニタに以下の情報を出力します。
- `WHO_AM_I` の確認結果
- 各設定レジスタへの書き込み値と読み出し値
- 磁気センサのキャリブレーション値
- 磁気センサのオーバーフロー検知時 (`ST1`, `ST2`, Rawデータ)

不要な場合は `Lib_MPU9250.cpp` 内の `Serial.print` をコメントアウトしてください。

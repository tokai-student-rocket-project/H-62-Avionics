# Lib_BNO055

Bosch BNO055 9軸フュージョンセンサ用のArduinoライブラリです。
生データ取得（AMGモード）から、内部フュージョン済みデータ（NDOFモード）まで幅広くサポートします。

## 特徴

*   **マルチモード対応**: 生データ取得用の **AMGモード** や、内部フュージョンを利用する **NDOFモード** など、全動作モードをサポート。
*   **軸リマップ機能**: センサの取り付け向きに合わせて、ハードウェアレベルで軸の入れ替えと符号反転を設定可能。
*   **キャリブレーション**: 内部キャリブレーション状態の取得に対応。
*   **外部水晶対応**: 精度の高い外部32kHz水晶発振子の使用設定が可能。

## 使用方法

### 1. インスタンス作成と初期化

```cpp
#include "Lib_BNO055.hpp"

BNO055 bno;

void setup() {
    Wire.begin();
    
    // NDOFモード（フュージョン済み）で開始
    if (!bno.begin(BNO055::OPERATION_MODE_NDOF)) {
        Serial.println("BNO055 init failed");
        while(1);
    }
    
    // 必要に応じて軸リマップを設定 (例: P1設定)
    bno.setAxisRemap(BNO055::REMAP_CONFIG_P1, BNO055::REMAP_SIGN_P1);
    
    // 外部水晶を使用 (推奨)
    bno.setExtCrystalUse(true);
}
```

### 2. データの取得

#### フュージョン済みデータ (NDOFモード等)

```cpp
float roll, pitch, yaw;
float q0, q1, q2, q3;
float lin_ax, lin_ay, lin_az;
float grav_x, grav_y, grav_z;

// オイラー角 (度)
bno.getEuler(&roll, &pitch, &yaw);

// クォータニオン
bno.getQuaternion(&q0, &q1, &q2, &q3);

// 線形加速度 (重力除去済み, m/s^2)
bno.getLinearAcceleration(&lin_ax, &lin_ay, &lin_az);

// 重力ベクトル (m/s^2)
bno.getGravity(&grav_x, &grav_y, &grav_z);
```

#### 生データ (AMGモード等)

```cpp
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

// 加速度 (m/s^2)
bno.getAcceleration(&ax, &ay, &az);

// ジャイロ (dps)
bno.getGyroscope(&gx, &gy, &gz);

// 磁気 (uT)
bno.getMagnetometer(&mx, &my, &mz);
```

### 3. キャリブレーション状態の確認

```cpp
uint8_t sys, gyro, accel, mag;
bno.getCalibration(&sys, &gyro, &accel, &mag);

Serial.print("Calib: Sys="); Serial.print(sys);
Serial.print(" Gyro="); Serial.print(gyro);
Serial.print(" Accel="); Serial.print(accel);
Serial.print(" Mag="); Serial.println(mag);
// 値は 0 (未補正) ～ 3 (完全補正)
```

### 4. キャリブレーション値の保存と復元

一度キャリブレーションを行った値を保存し、次回起動時に復元することで、毎回キャリブレーションを行う手間を省けます。

```cpp
// キャリブレーション完了後、値を保存
if (bno.isFullyCalibrated()) {
    BNO055::bno055_offsets_t offsets;
    bno.getSensorOffsets(offsets);
    // offsets を EEPROM や Flash に保存
}

// 起動時、値を復元
BNO055::bno055_offsets_t offsets;
// EEPROM や Flash から offsets を読み込み
bno.setSensorOffsets(offsets);
```

### 5. キャリブレーションプロファイルの使用 (推奨)

`BNO055_Calibration.hpp` を使用して、ソースコードとは別にキャリブレーション値を管理できます。

1. キャリブレーションを実行し、表示された値を `lib/Lib_BNO055/BNO055_Calibration.hpp` に書き写します。
2. メインコードで以下のように読み込みます。

```cpp
#include "Lib_BNO055.hpp"
#include "BNO055_Calibration.hpp" // 追加

// ... setup() 内 ...

// 初期化後
bno.setSensorOffsets(BNO055_CALIBRATION_PROFILE);
```

## API リファレンス

### `bool begin(OperationMode mode)`
センサを初期化し、指定したモードに設定します。
- `mode`: 初期動作モード (デフォルト: `OPERATION_MODE_NDOF`)
- 戻り値: 初期化成功で `true`

### `void setMode(OperationMode mode)`
動作モードを変更します。
- `OPERATION_MODE_CONFIG`: 設定モード
- `OPERATION_MODE_AMG`: 加速度・磁気・ジャイロの生データ出力 (フュージョンなし)
- `OPERATION_MODE_NDOF`: 9軸フュージョン (絶対方位)
- `OPERATION_MODE_IMUPLUS`: 6軸フュージョン (相対方位, 磁気不使用)
- その他: データシート参照

### `void setAxisRemap(RemapConfig config, RemapSign sign)`
軸の割り当てと符号を設定します。
- `config`: 軸の入れ替え設定 (`REMAP_CONFIG_P0` ～ `P7`)
- `sign`: 軸の符号設定 (`REMAP_SIGN_P0` ～ `P7`)
- デフォルトは `P1` (変化なし) です。

### `void setExtCrystalUse(bool usextal)`
外部32kHz水晶発振子の使用を切り替えます。
- `true`: 外部水晶を使用 (精度向上, 推奨)
- `false`: 内部発振回路を使用

### `void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)`
現在のキャリブレーションステータス (0-3) を取得します。3が最高精度です。

### `bool isFullyCalibrated()`
システム、ジャイロ、加速度、磁気の全てのキャリブレーションステータスが3（最大）であるかを確認します。
- 戻り値: 全て完了していれば `true`

### `bool getSensorOffsets(bno055_offsets_t &offsets_type)`
現在のキャリブレーションオフセット値を取得します。
- `offsets_type`: 取得した値を格納する構造体
- 戻り値: 成功時 `true`

### `void setSensorOffsets(const bno055_offsets_t &offsets_type)`
キャリブレーションオフセット値をセンサに設定します。
- `offsets_type`: 設定する値が格納された構造体

## I2Cアドレス
デフォルトで `0x28` を使用します。変更が必要な場合はコンストラクタで指定してください。
```cpp
BNO055 bno(0x29);
```

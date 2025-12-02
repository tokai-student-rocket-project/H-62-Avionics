#pragma once

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief MPU9250 9軸センサライブラリ
 */
class MPU9250 {
public:
    enum Ascale {
      AFS_2G = 0,
      AFS_4G,
      AFS_8G,
      AFS_16G
    };

    enum Gscale {
      GFS_250DPS = 0,
      GFS_500DPS,
      GFS_1000DPS,
      GFS_2000DPS
    };

    enum DLPF {
      DLPF_250HZ = 0,
      DLPF_184HZ,
      DLPF_92HZ,
      DLPF_41HZ,
      DLPF_20HZ,
      DLPF_10HZ,
      DLPF_5HZ
    };

    MPU9250();
    
    /**
     * @brief センサの初期化を行います。
     * 
     * @param ascale 加速度センサのレンジ (デフォルト: AFS_16G)
     * @param gscale ジャイロセンサのレンジ (デフォルト: GFS_2000DPS)
     * @param dlpf DLPFの帯域幅 (デフォルト: DLPF_41HZ)
     * @param wire I2Cインターフェース (デフォルト: &Wire)
     * @return true 初期化成功
     * @return false 初期化失敗
     */
    bool begin(Ascale ascale = AFS_16G, Gscale gscale = GFS_2000DPS, DLPF dlpf = DLPF_41HZ, TwoWire* wire = &Wire);
    
    /**
     * @brief 加速度センサのレンジを設定します。
     * @param ascale レンジ (AFS_2G, AFS_4G, AFS_8G, AFS_16G)
     */
    void setAccelRange(Ascale ascale);

    /**
     * @brief ジャイロセンサのレンジを設定します。
     * @param gscale レンジ (GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS)
     */
    void setGyroRange(Gscale gscale);

    /**
     * @brief DLPF (Digital Low Pass Filter) の帯域幅を設定します。
     * @param dlpf 帯域幅 (DLPF_250HZ ... DLPF_5HZ)
     */
    void setDLPFBandwidth(DLPF dlpf);
    
    /**
     * @brief 加速度を取得します。
     * 
     * @param x X軸加速度 (m/s^2)
     * @param y Y軸加速度 (m/s^2)
     * @param z Z軸加速度 (m/s^2)
     */
    void getAcceleration(float* x, float* y, float* z);
    
    /**
     * @brief 角速度(ジャイロ)を取得します。
     * 
     * @param x X軸角速度 (dps: degrees per second)
     * @param y Y軸角速度 (dps: degrees per second)
     * @param z Z軸角速度 (dps: degrees per second)
     */
    void getGyroscope(float* x, float* y, float* z);
    
    /**
     * @brief 磁束密度を取得します。
     * 
     * @param x X軸磁束密度 (uT: micro Tesla)
     * @param y Y軸磁束密度 (uT: micro Tesla)
     * @param z Z軸磁束密度 (uT: micro Tesla)
     */
    void getMagnetometer(float* x, float* y, float* z);

    // I2C Read/Write functions
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t readByte(uint8_t address, uint8_t subAddress);
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

    // Calibration
    void setAccelCalibration(float biasX, float biasY, float biasZ, float scaleX, float scaleY, float scaleZ);
    void calibrateAccel(); // Interactive calibration procedure
    
    void setGyroBias(float biasX, float biasY, float biasZ);
    void calibrateGyro(); // Interactive calibration procedure

    void setMagCalibration(float biasX, float biasY, float biasZ, float scaleX, float scaleY, float scaleZ);
    void calibrateMag(); // Interactive calibration procedure

    /**
     * @brief センサの軸をリマップ（入れ替え）します。
     * ロケットの機体軸に合わせてセンサの出力軸を変更する場合に使用します。
     * 
     * @param xMap 出力X軸に割り当てるセンサ軸 (0:X, 1:Y, 2:Z)
     * @param yMap 出力Y軸に割り当てるセンサ軸 (0:X, 1:Y, 2:Z)
     * @param zMap 出力Z軸に割り当てるセンサ軸 (0:X, 1:Y, 2:Z)
     * @param xSign 出力X軸の符号 (1:正, -1:負)
     * @param ySign 出力Y軸の符号 (1:正, -1:負)
     * @param zSign 出力Z軸の符号 (1:正, -1:負)
     */
    void setAxisRemap(uint8_t xMap, uint8_t yMap, uint8_t zMap, float xSign, float ySign, float zSign);

    /**
     * @brief 地磁気センサの軸をリマップ（入れ替え）します。
     * 加速度・ジャイロとは独立して地磁気の軸を設定する場合に使用します。
     * 
     * @param xMap 出力X軸に割り当てるセンサ軸 (0:X, 1:Y, 2:Z)
     * @param yMap 出力Y軸に割り当てるセンサ軸 (0:X, 1:Y, 2:Z)
     * @param zMap 出力Z軸に割り当てるセンサ軸 (0:X, 1:Y, 2:Z)
     * @param xSign 出力X軸の符号 (1:正, -1:負)
     * @param ySign 出力Y軸の符号 (1:正, -1:負)
     * @param zSign 出力Z軸の符号 (1:正, -1:負)
     */
    void setMagRemap(uint8_t xMap, uint8_t yMap, uint8_t zMap, float xSign, float ySign, float zSign);

private:

    // MPU9250 I2C Address
    const uint8_t MPU9250_ADDRESS = 0x69;
    // AK8963 I2C Address
    const uint8_t AK8963_ADDRESS = 0x0C;

    // Scale factors
    float aRes; // Scale for Accl
    float gRes; // Scale for Gyro
    float mRes; // Scale for Mag

    // Magnetometer adjustment values
    float magCalibration[3] = {0, 0, 0};
    
    // Accel Calibration
    float _accelBias[3];
    float _accelScale[3];

    // Gyro Calibration
    float _gyroBias[3];

    // Mag Calibration
    float _magBias[3];
    float _magScale[3];
    
    Ascale _ascale;
    Gscale _gscale;
    DLPF _dlpf;
    TwoWire* _wire;

    // Axis Remapping
    uint8_t _axisMap[3];
    float _axisSign[3];

    // Mag Axis Remapping
    uint8_t _magMap[3];
    float _magSign[3];
    
    void updateAccelRes();
    void updateGyroRes();
};

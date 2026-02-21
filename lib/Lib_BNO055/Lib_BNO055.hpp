#pragma once


#include <Arduino.h>
#include <Wire.h>


class BNO055 {
public:
    enum OperationMode {
        OPERATION_MODE_CONFIG = 0x00,
        OPERATION_MODE_ACCONLY = 0x01,
        OPERATION_MODE_MAGONLY = 0x02,
        OPERATION_MODE_GYRONLY = 0x03,
        OPERATION_MODE_ACCMAG = 0x04,
        OPERATION_MODE_ACCGYRO = 0x05,
        OPERATION_MODE_MAGGYRO = 0x06,
        OPERATION_MODE_AMG = 0x07,
        OPERATION_MODE_IMUPLUS = 0x08,
        OPERATION_MODE_COMPASS = 0x09,
        OPERATION_MODE_M4G = 0x0A,
        OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
        OPERATION_MODE_NDOF = 0x0C
    };

    enum PowerMode {
        POWER_MODE_NORMAL = 0x00,
        POWER_MODE_LOWPOWER = 0x01,
        POWER_MODE_SUSPEND = 0x02
    };

    enum RemapConfig {
        REMAP_CONFIG_P0 = 0x21,
        REMAP_CONFIG_P1 = 0x24, // Default
        REMAP_CONFIG_P2 = 0x24,
        REMAP_CONFIG_P3 = 0x21,
        REMAP_CONFIG_P4 = 0x24,
        REMAP_CONFIG_P5 = 0x21,
        REMAP_CONFIG_P6 = 0x21,
        REMAP_CONFIG_P7 = 0x24
    };

    enum RemapSign {
        REMAP_SIGN_P0 = 0x04,
        REMAP_SIGN_P1 = 0x00, // Default
        REMAP_SIGN_P2 = 0x06,
        REMAP_SIGN_P3 = 0x02,
        REMAP_SIGN_P4 = 0x03,
        REMAP_SIGN_P5 = 0x01,
        REMAP_SIGN_P6 = 0x07,
        REMAP_SIGN_P7 = 0x05
    };

    BNO055(uint8_t address = 0x28);

    bool begin(OperationMode mode = OPERATION_MODE_NDOF);
    
    void setMode(OperationMode mode);
    void setExtCrystalUse(bool usextal);
    void setAxisRemap(RemapConfig config, RemapSign sign);

    typedef struct
    {
        int16_t accel_offset_x;
        int16_t accel_offset_y;
        int16_t accel_offset_z;
        int16_t mag_offset_x;
        int16_t mag_offset_y;
        int16_t mag_offset_z;
        int16_t gyro_offset_x;
        int16_t gyro_offset_y;
        int16_t gyro_offset_z;
        int16_t accel_radius;
        int16_t mag_radius;
    } bno055_offsets_t;

    void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
    bool isFullyCalibrated();

    bool getSensorOffsets(bno055_offsets_t &offsets_type);
    void setSensorOffsets(const bno055_offsets_t &offsets_type);
    
    // Data getters
    void getAcceleration(float* x, float* y, float* z);
    void getMagnetometer(float* x, float* y, float* z);
    void getGyroscope(float* x, float* y, float* z);
    void getLinearAcceleration(float* x, float* y, float* z);
    void getGravity(float* x, float* y, float* z);
    void getQuaternion(float* w, float* x, float* y, float* z);
    void getEuler(float* roll, float* pitch, float* yaw);

private:
    uint8_t _address;
    OperationMode _mode;

    void write(uint8_t reg, uint8_t data);
    uint8_t read(uint8_t reg);
    bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len);
    void readVector3D(uint8_t reg, float lsb, float* x, float* y, float* z);
};
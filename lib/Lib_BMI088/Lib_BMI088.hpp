#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

/**
 * @brief Bosch BMI088 6-axis IMU Library
 * Supports both I2C and SPI interfaces.
 */
class BMI088 {
public:
    enum AccelRange {
        ACCEL_3G = 0x00,
        ACCEL_6G = 0x01,
        ACCEL_12G = 0x02,
        ACCEL_24G = 0x03
    };

    enum AccelODR {
        ACCEL_ODR_12_5HZ = 0x05,
        ACCEL_ODR_25HZ = 0x06,
        ACCEL_ODR_50HZ = 0x07,
        ACCEL_ODR_100HZ = 0x08,
        ACCEL_ODR_200HZ = 0x09,
        ACCEL_ODR_400HZ = 0x0A,
        ACCEL_ODR_800HZ = 0x0B,
        ACCEL_ODR_1600HZ = 0x0C
    };

    enum GyroRange {
        GYRO_2000DPS = 0x00,
        GYRO_1000DPS = 0x01,
        GYRO_500DPS = 0x02,
        GYRO_250DPS = 0x03,
        GYRO_125DPS = 0x04
    };

    enum GyroODR {
        GYRO_ODR_2000HZ_532HZ = 0x00, // ODR 2000Hz, Filter 532Hz
        GYRO_ODR_2000HZ_230HZ = 0x01,
        GYRO_ODR_1000HZ_116HZ = 0x02,
        GYRO_ODR_400HZ_47HZ = 0x03,
        GYRO_ODR_200HZ_23HZ = 0x04,
        GYRO_ODR_100HZ_12HZ = 0x05,
        GYRO_ODR_200HZ_64HZ = 0x06,
        GYRO_ODR_100HZ_32HZ = 0x07
    };

    BMI088();

    /**
     * @brief Initialize sensor using I2C
     * @param wire Pointer to TwoWire instance (default &Wire)
     * @param accelAddr I2C address of Accelerometer (default 0x18)
     * @param gyroAddr I2C address of Gyroscope (default 0x68)
     * @return true if initialization successful
     */
    bool beginI2C(TwoWire* wire = &Wire, uint8_t accelAddr = 0x18, uint8_t gyroAddr = 0x68);

    /**
     * @brief Initialize sensor using SPI
     * @param spi Pointer to SPIClass instance (default &SPI)
     * @param accelCS Chip Select pin for Accelerometer
     * @param gyroCS Chip Select pin for Gyroscope
     * @return true if initialization successful
     */
    bool beginSPI(SPIClass* spi, uint8_t accelCS, uint8_t gyroCS);

    // Configuration
    void setAccelConfig(AccelRange range, AccelODR odr);
    void setGyroConfig(GyroRange range, GyroODR odr);

    // Data Acquisition
    void getAcceleration(float* x, float* y, float* z);
    void getGyroscope(float* x, float* y, float* z);
    float getTemperature();

    // Axis Remapping
    void setAxisRemap(uint8_t xMap, uint8_t yMap, uint8_t zMap, float xSign, float ySign, float zSign);

private:
    // Interface Type
    enum Interface {
        INTERFACE_I2C,
        INTERFACE_SPI
    };

    Interface _interface;
    
    // I2C Settings
    TwoWire* _wire;
    uint8_t _accelAddr;
    uint8_t _gyroAddr;

    // SPI Settings
    SPIClass* _spi;
    uint8_t _accelCS;
    uint8_t _gyroCS;

    // Scales
    float _accelScale;
    float _gyroScale;

    // Axis Remapping
    uint8_t _axisMap[3];
    float _axisSign[3];

    // Internal Helper Functions
    void writeRegister(bool isAccel, uint8_t reg, uint8_t data);
    uint8_t readRegister(bool isAccel, uint8_t reg);
    void readRegisters(bool isAccel, uint8_t reg, uint8_t count, uint8_t* dest);
    
    void softResetAccel();
    void softResetGyro();
};

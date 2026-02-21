#include "Lib_BMI088.hpp"

// Registers - Accelerometer
#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_CONF         0x40
#define BMI088_ACC_RANGE        0x41
#define BMI088_ACC_PWR_CONF     0x7C
#define BMI088_ACC_PWR_CTRL     0x7D
#define BMI088_ACC_SOFTRESET    0x7E
#define BMI088_ACC_X_LSB        0x12

// Registers - Gyroscope
#define BMI088_GYRO_CHIP_ID     0x00
#define BMI088_GYRO_X_LSB       0x02
#define BMI088_GYRO_RANGE       0x0F
#define BMI088_GYRO_BANDWIDTH   0x10
#define BMI088_GYRO_LPM1        0x11
#define BMI088_GYRO_SOFTRESET   0x14

// Chip IDs
#define BMI088_ACC_CHIP_ID_VAL  0x1E
#define BMI088_GYRO_CHIP_ID_VAL 0x0F

BMI088::BMI088() {
    _interface = INTERFACE_I2C;
    _accelScale = 0.0f;
    _gyroScale = 0.0f;
    for(int i=0; i<3; i++) {
        _axisMap[i] = i;
        _axisSign[i] = 1.0f;
    }
}

bool BMI088::beginI2C(TwoWire* wire, uint8_t accelAddr, uint8_t gyroAddr) {
    _interface = INTERFACE_I2C;
    _wire = wire;
    _accelAddr = accelAddr;
    _gyroAddr = gyroAddr;

    // Check Chip IDs
    uint8_t accId = readRegister(true, BMI088_ACC_CHIP_ID);
    uint8_t gyroId = readRegister(false, BMI088_GYRO_CHIP_ID);

    if (accId != BMI088_ACC_CHIP_ID_VAL) {
        Serial.print("BMI088 Accel ID Mismatch: 0x"); Serial.println(accId, HEX);
        return false;
    }
    if (gyroId != BMI088_GYRO_CHIP_ID_VAL) {
        Serial.print("BMI088 Gyro ID Mismatch: 0x"); Serial.println(gyroId, HEX);
        return false;
    }

    softResetAccel();
    softResetGyro();

    // Power up Accel
    // ACC_PWR_CTRL: 0x04 to enable
    writeRegister(true, BMI088_ACC_PWR_CTRL, 0x04);
    // ACC_PWR_CONF: 0x00 (Active mode)
    writeRegister(true, BMI088_ACC_PWR_CONF, 0x00);
    
    delay(50); // Wait for power up

    // Default Config
    setAccelConfig(ACCEL_24G, ACCEL_ODR_400HZ);
    setGyroConfig(GYRO_2000DPS, GYRO_ODR_400HZ_47HZ);

    return true;
}

bool BMI088::beginSPI(SPIClass* spi, uint8_t accelCS, uint8_t gyroCS) {
    _interface = INTERFACE_SPI;
    _spi = spi;
    _accelCS = accelCS;
    _gyroCS = gyroCS;

    pinMode(_accelCS, OUTPUT);
    pinMode(_gyroCS, OUTPUT);
    digitalWrite(_accelCS, HIGH);
    digitalWrite(_gyroCS, HIGH);

    _spi->begin();

    // Dummy read for SPI to wake up or sync if needed (BMI088 specific behavior)
    // Actually, just reading ID is enough test.
    
    // Check Chip IDs
    uint8_t accId = readRegister(true, BMI088_ACC_CHIP_ID);
    uint8_t gyroId = readRegister(false, BMI088_GYRO_CHIP_ID);

    if (accId != BMI088_ACC_CHIP_ID_VAL) {
        Serial.print("BMI088 Accel ID Mismatch (SPI): 0x"); Serial.println(accId, HEX);
        return false;
    }
    if (gyroId != BMI088_GYRO_CHIP_ID_VAL) {
        Serial.print("BMI088 Gyro ID Mismatch (SPI): 0x"); Serial.println(gyroId, HEX);
        return false;
    }

    softResetAccel();
    softResetGyro();

    // Power up Accel
    writeRegister(true, BMI088_ACC_PWR_CTRL, 0x04);
    writeRegister(true, BMI088_ACC_PWR_CONF, 0x00);
    
    delay(50);

    // Default Config
    setAccelConfig(ACCEL_24G, ACCEL_ODR_400HZ);
    setGyroConfig(GYRO_2000DPS, GYRO_ODR_400HZ_47HZ);

    return true;
}

void BMI088::softResetAccel() {
    writeRegister(true, BMI088_ACC_SOFTRESET, 0xB6);
    delay(50); // Wait for reset
    
    // In SPI mode, dummy read might be needed to switch to SPI protocol?
    // Datasheet says: "After a softreset, the sensor reboots and the interface selection is performed."
    // For SPI, CS rising edge is enough.
    if (_interface == INTERFACE_SPI) {
        // Dummy read to ensure SPI mode is detected
        readRegister(true, BMI088_ACC_CHIP_ID);
    }
}

void BMI088::softResetGyro() {
    writeRegister(false, BMI088_GYRO_SOFTRESET, 0xB6);
    delay(50);
}

void BMI088::setAccelConfig(AccelRange range, AccelODR odr) {
    writeRegister(true, BMI088_ACC_RANGE, range);
    writeRegister(true, BMI088_ACC_CONF, (0x08 | odr)); // 0x08 is BWP (OSR4) - Normal bandwidth

    switch(range) {
        case ACCEL_3G: _accelScale = 3.0f / 32768.0f; break;
        case ACCEL_6G: _accelScale = 6.0f / 32768.0f; break;
        case ACCEL_12G: _accelScale = 12.0f / 32768.0f; break;
        case ACCEL_24G: _accelScale = 24.0f / 32768.0f; break;
    }
}

void BMI088::setGyroConfig(GyroRange range, GyroODR odr) {
    writeRegister(false, BMI088_GYRO_RANGE, range);
    writeRegister(false, BMI088_GYRO_BANDWIDTH, odr);

    switch(range) {
        case GYRO_2000DPS: _gyroScale = 2000.0f / 32767.0f; break;
        case GYRO_1000DPS: _gyroScale = 1000.0f / 32767.0f; break;
        case GYRO_500DPS: _gyroScale = 500.0f / 32767.0f; break;
        case GYRO_250DPS: _gyroScale = 250.0f / 32767.0f; break;
        case GYRO_125DPS: _gyroScale = 125.0f / 32767.0f; break;
    }
}

void BMI088::getAcceleration(float* x, float* y, float* z) {
    uint8_t buf[6];
    readRegisters(true, BMI088_ACC_X_LSB, 6, buf);

    int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);

    float ax = rawX * _accelScale * 9.80665f;
    float ay = rawY * _accelScale * 9.80665f;
    float az = rawZ * _accelScale * 9.80665f;

    float temp[3] = {ax, ay, az};
    *x = temp[_axisMap[0]] * _axisSign[0];
    *y = temp[_axisMap[1]] * _axisSign[1];
    *z = temp[_axisMap[2]] * _axisSign[2];
}

void BMI088::getGyroscope(float* x, float* y, float* z) {
    uint8_t buf[6];
    readRegisters(false, BMI088_GYRO_X_LSB, 6, buf);

    int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);

    float gx = rawX * _gyroScale;
    float gy = rawY * _gyroScale;
    float gz = rawZ * _gyroScale;

    float temp[3] = {gx, gy, gz};
    *x = temp[_axisMap[0]] * _axisSign[0];
    *y = temp[_axisMap[1]] * _axisSign[1];
    *z = temp[_axisMap[2]] * _axisSign[2];
}

float BMI088::getTemperature() {
    uint8_t buf[2];
    readRegisters(true, 0x22, 2, buf); // ACC_TEMP
    int16_t tempRaw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (tempRaw > 1023) tempRaw -= 2048;
    return (tempRaw * 0.125f) + 23.0f;
}

void BMI088::setAxisRemap(uint8_t xMap, uint8_t yMap, uint8_t zMap, float xSign, float ySign, float zSign) {
    _axisMap[0] = xMap;
    _axisMap[1] = yMap;
    _axisMap[2] = zMap;
    _axisSign[0] = xSign;
    _axisSign[1] = ySign;
    _axisSign[2] = zSign;
}

// Internal Helper Functions

void BMI088::writeRegister(bool isAccel, uint8_t reg, uint8_t data) {
    if (_interface == INTERFACE_I2C) {
        uint8_t addr = isAccel ? _accelAddr : _gyroAddr;
        _wire->beginTransmission(addr);
        _wire->write(reg);
        _wire->write(data);
        _wire->endTransmission();
    } else {
        uint8_t cs = isAccel ? _accelCS : _gyroCS;
        _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1MHz safe
        digitalWrite(cs, LOW);
        _spi->transfer(reg & 0x7F); // Write: MSB 0
        _spi->transfer(data);
        digitalWrite(cs, HIGH);
        _spi->endTransaction();
    }
}

uint8_t BMI088::readRegister(bool isAccel, uint8_t reg) {
    uint8_t data = 0;
    if (_interface == INTERFACE_I2C) {
        uint8_t addr = isAccel ? _accelAddr : _gyroAddr;
        _wire->beginTransmission(addr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(addr, (uint8_t)1);
        if (_wire->available()) data = _wire->read();
    } else {
        uint8_t cs = isAccel ? _accelCS : _gyroCS;
        _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(cs, LOW);
        _spi->transfer(reg | 0x80); // Read: MSB 1
        if (isAccel) {
            _spi->transfer(0x00); // Dummy byte for Accel in SPI
        }
        data = _spi->transfer(0x00);
        digitalWrite(cs, HIGH);
        _spi->endTransaction();
    }
    return data;
}

void BMI088::readRegisters(bool isAccel, uint8_t reg, uint8_t count, uint8_t* dest) {
    if (_interface == INTERFACE_I2C) {
        uint8_t addr = isAccel ? _accelAddr : _gyroAddr;
        _wire->beginTransmission(addr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(addr, count);
        for(uint8_t i=0; i<count; i++) {
            if (_wire->available()) dest[i] = _wire->read();
        }
    } else {
        uint8_t cs = isAccel ? _accelCS : _gyroCS;
        _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(cs, LOW);
        _spi->transfer(reg | 0x80); // Read: MSB 1
        if (isAccel) {
            _spi->transfer(0x00); // Dummy byte for Accel in SPI
        }
        for(uint8_t i=0; i<count; i++) {
            dest[i] = _spi->transfer(0x00);
        }
        digitalWrite(cs, HIGH);
        _spi->endTransaction();
    }
}

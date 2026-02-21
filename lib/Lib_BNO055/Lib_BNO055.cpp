#include "Lib_BNO055.hpp"

// Registers
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_MAG_DATA_X_LSB_ADDR 0x0E
#define BNO055_GYRO_DATA_X_LSB_ADDR 0x14
#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_GRAVITY_DATA_X_LSB_ADDR 0x2E
#define BNO055_TEMP_ADDR 0x34
#define BNO055_CALIB_STAT_ADDR 0x35
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41
#define BNO055_AXIS_MAP_SIGN_ADDR 0x42
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E

#define BNO055_ACCEL_OFFSET_X_LSB_ADDR 0x55
#define BNO055_MAG_OFFSET_X_LSB_ADDR 0x5B
#define BNO055_GYRO_OFFSET_X_LSB_ADDR 0x61
#define BNO055_ACCEL_RADIUS_LSB_ADDR 0x67
#define BNO055_MAG_RADIUS_LSB_ADDR 0x69

BNO055::BNO055(uint8_t address)
{
    _address = address;
    _mode = OPERATION_MODE_CONFIG;
}

bool BNO055::begin(OperationMode mode)
{
    Wire.begin();
    Wire.setClock(100000); // Explicitly set to 100kHz
    
    // Increase timeout for BNO055 clock stretching (25ms)
    // Note: setWireTimeout is available on AVR and some other cores.
    // If it causes compilation error on Uno R4, remove it.
    // But Uno R4 (Renesas) might support it or handle it differently.
    // Let's try setting it if possible, or just rely on clock speed.
    // Wire.setWireTimeout(25000, true); // Not supported on Uno R4

    delay(100); // Wait for bus to stabilize

    // Check Chip ID FIRST to verify communication
    Serial.println("[BNO055] Checking Chip ID (Pre-Reset)...");
    
    // Try to read ID directly without setting page first (default is page 0)
    uint8_t id = read(BNO055_CHIP_ID_ADDR);
    if (id != 0xA0)
    {
        Serial.print("[BNO055] ID Mismatch (Pre-Reset). Expected 0xA0, got 0x");
        Serial.println(id, HEX);
        // If read fails, maybe we are on wrong page? Try setting page 0 now.
        write(BNO055_PAGE_ID_ADDR, 0);
        delay(10);
        id = read(BNO055_CHIP_ID_ADDR);
        if (id != 0xA0) {
             Serial.print("[BNO055] ID Mismatch 2nd try. Got 0x");
             Serial.println(id, HEX);
        }
    }
    else
    {
        Serial.println("[BNO055] ID OK (Pre-Reset).");
    }

    Serial.println("[BNO055] Resetting...");
    // Reset
    write(BNO055_SYS_TRIGGER_ADDR, 0x20);
    delay(800); // Increase delay to be safe (datasheet says 650ms)

    // Check Chip ID AGAIN
    Serial.println("[BNO055] Checking Chip ID (Post-Reset)...");
    id = read(BNO055_CHIP_ID_ADDR);
    if (id != 0xA0)
    {
        Serial.print("[BNO055] ID Mismatch (Post-Reset). Expected 0xA0, got 0x");
        Serial.println(id, HEX);
        delay(1000);
        id = read(BNO055_CHIP_ID_ADDR);
        if (id != 0xA0)
        {
            Serial.print("[BNO055] Retry failed. Got 0x");
            Serial.println(id, HEX);
            return false;
        }
    }
    Serial.println("[BNO055] ID OK.");

    // Set to normal power mode
    write(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    delay(10);

    // Set to config mode to configure
    setMode(OPERATION_MODE_CONFIG);

    // Set to Page 1 for configuration
    write(BNO055_PAGE_ID_ADDR, 1);

    // ACC_Config <- 16G, 1000Hz, Normal (0b00001111 = 0x0F)
    // G Range: 16G (xx01), Bandwidth: 1000Hz (xxx111xx), Operation: Normal (xxx000xx) -> wait, check datasheet
    // Register 0x08 (ACC_CONFIG):
    // Bits 2:0 = Bandwidth (111 = 1000Hz)
    // Bits 4:3 = Range (00=2G, 01=4G, 10=8G, 11=16G)
    // Bits 7:5 = Operation Mode (000=Normal)
    // So 16G + 1000Hz + Normal = 0b00011111 = 0x1F
    write(0x08, 0x1F);
    delay(10);

    // GYR_Config_0 <- 2000dps, 523Hz (0x00)
    // Register 0x0A:
    // Bits 2:0 = Range (000=2000dps)
    // Bits 5:3 = Bandwidth (000=523Hz)
    write(0x0A, 0x00);
    delay(10);

    // MAG_Config <- 30Hz, Regular, Normal (0x0B)
    // Register 0x09:
    // Bits 2:0 = Data Output Rate (011=30Hz)
    // Bits 4:3 = Operation Mode (01=Regular)
    // Bits 6:5 = Power Mode (00=Normal)
    // 0b00001011 = 0x0B
    write(0x09, 0x0B);
    delay(10);

    // Set back to Page 0
    write(BNO055_PAGE_ID_ADDR, 0);

    // Set operation mode
    Serial.print("[BNO055] Setting Mode: 0x");
    Serial.println(mode, HEX);
    setMode(mode);

    return true;
}

void BNO055::write(uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(data);
    uint8_t error = Wire.endTransmission(); // Default is true (Stop)
    if (error != 0)
    {
        Serial.print("[BNO055] Write Error: ");
        Serial.print(error);
        Serial.print(" Reg: 0x");
        Serial.println(reg, HEX);
    }
}

uint8_t BNO055::read(uint8_t reg)
{
    Wire.beginTransmission(_address);
    Wire.write(reg);
    // Use false (Repeated Start) as per datasheet and Adafruit library
    uint8_t error = Wire.endTransmission(false);
    if (error != 0)
    {
        Serial.print("[BNO055] Read (Write Addr) Error: ");
        Serial.println(error);
        return 0xFF;
    }

    uint8_t n = Wire.requestFrom(_address, (uint8_t)1);
    if (n != 1)
    {
        Serial.print("[BNO055] Read (Request) Error. Got ");
        Serial.println(n);
        return 0xFF;
    }
    return Wire.read();
}

void BNO055::setMode(OperationMode mode)
{
    _mode = mode;
    write(BNO055_OPR_MODE_ADDR, _mode);
    delay(30); // Mode switch delay
}

void BNO055::setExtCrystalUse(bool usextal)
{
    OperationMode lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);
    write(BNO055_PAGE_ID_ADDR, 0);
    if (usextal)
    {
        write(BNO055_SYS_TRIGGER_ADDR, 0x80);
    }
    else
    {
        write(BNO055_SYS_TRIGGER_ADDR, 0x00);
    }
    delay(10);
    setMode(lastMode);
    delay(20);
}

void BNO055::setAxisRemap(RemapConfig config, RemapSign sign)
{
    OperationMode lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);

    write(BNO055_AXIS_MAP_CONFIG_ADDR, config);
    delay(10);
    write(BNO055_AXIS_MAP_SIGN_ADDR, sign);
    delay(10);

    setMode(lastMode);
    delay(20);
}

void BNO055::getCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
{
    uint8_t calData = read(BNO055_CALIB_STAT_ADDR);
    if (sys)
        *sys = (calData >> 6) & 0x03;
    if (gyro)
        *gyro = (calData >> 4) & 0x03;
    if (accel)
        *accel = (calData >> 2) & 0x03;
    if (mag)
        *mag = calData & 0x03;
}

bool BNO055::isFullyCalibrated()
{
    uint8_t sys, gyro, accel, mag;
    getCalibration(&sys, &gyro, &accel, &mag);
    return (sys == 3 && gyro == 3 && accel == 3 && mag == 3);
}

bool BNO055::getSensorOffsets(bno055_offsets_t &offsets_type)
{
    OperationMode lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);

    uint8_t buffer[22];
    readLen(BNO055_ACCEL_OFFSET_X_LSB_ADDR, buffer, 22);

    offsets_type.accel_offset_x = (buffer[1] << 8) | buffer[0];
    offsets_type.accel_offset_y = (buffer[3] << 8) | buffer[2];
    offsets_type.accel_offset_z = (buffer[5] << 8) | buffer[4];
    offsets_type.mag_offset_x = (buffer[7] << 8) | buffer[6];
    offsets_type.mag_offset_y = (buffer[9] << 8) | buffer[8];
    offsets_type.mag_offset_z = (buffer[11] << 8) | buffer[10];
    offsets_type.gyro_offset_x = (buffer[13] << 8) | buffer[12];
    offsets_type.gyro_offset_y = (buffer[15] << 8) | buffer[14];
    offsets_type.gyro_offset_z = (buffer[17] << 8) | buffer[16];
    offsets_type.accel_radius = (buffer[19] << 8) | buffer[18];
    offsets_type.mag_radius = (buffer[21] << 8) | buffer[20];

    setMode(lastMode);
    return true;
}

void BNO055::setSensorOffsets(const bno055_offsets_t &offsets_type)
{
    OperationMode lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);

    // Write 22 bytes starting from ACCEL_OFFSET_X_LSB_ADDR
    Wire.beginTransmission(_address);
    Wire.write(BNO055_ACCEL_OFFSET_X_LSB_ADDR);

    Wire.write((uint8_t)(offsets_type.accel_offset_x & 0xFF));
    Wire.write((uint8_t)((offsets_type.accel_offset_x >> 8) & 0xFF));
    Wire.write((uint8_t)(offsets_type.accel_offset_y & 0xFF));
    Wire.write((uint8_t)((offsets_type.accel_offset_y >> 8) & 0xFF));
    Wire.write((uint8_t)(offsets_type.accel_offset_z & 0xFF));
    Wire.write((uint8_t)((offsets_type.accel_offset_z >> 8) & 0xFF));

    Wire.write((uint8_t)(offsets_type.mag_offset_x & 0xFF));
    Wire.write((uint8_t)((offsets_type.mag_offset_x >> 8) & 0xFF));
    Wire.write((uint8_t)(offsets_type.mag_offset_y & 0xFF));
    Wire.write((uint8_t)((offsets_type.mag_offset_y >> 8) & 0xFF));
    Wire.write((uint8_t)(offsets_type.mag_offset_z & 0xFF));
    Wire.write((uint8_t)((offsets_type.mag_offset_z >> 8) & 0xFF));

    Wire.write((uint8_t)(offsets_type.gyro_offset_x & 0xFF));
    Wire.write((uint8_t)((offsets_type.gyro_offset_x >> 8) & 0xFF));
    Wire.write((uint8_t)(offsets_type.gyro_offset_y & 0xFF));
    Wire.write((uint8_t)((offsets_type.gyro_offset_y >> 8) & 0xFF));
    Wire.write((uint8_t)(offsets_type.gyro_offset_z & 0xFF));
    Wire.write((uint8_t)((offsets_type.gyro_offset_z >> 8) & 0xFF));

    Wire.write((uint8_t)(offsets_type.accel_radius & 0xFF));
    Wire.write((uint8_t)((offsets_type.accel_radius >> 8) & 0xFF));
    Wire.write((uint8_t)(offsets_type.mag_radius & 0xFF));
    Wire.write((uint8_t)((offsets_type.mag_radius >> 8) & 0xFF));

    Wire.endTransmission();

    setMode(lastMode);
}

void BNO055::getAcceleration(float *x, float *y, float *z)
{
    // 1m/s^2 = 100 LSB
    readVector3D(BNO055_ACCEL_DATA_X_LSB_ADDR, 100.0f, x, y, z);
}

void BNO055::getMagnetometer(float *x, float *y, float *z)
{
    // 1uT = 16 LSB
    readVector3D(BNO055_MAG_DATA_X_LSB_ADDR, 16.0f, x, y, z);
}

void BNO055::getGyroscope(float *x, float *y, float *z)
{
    // 1dps = 16 LSB
    readVector3D(BNO055_GYRO_DATA_X_LSB_ADDR, 16.0f, x, y, z);
}

void BNO055::getLinearAcceleration(float *x, float *y, float *z)
{
    // 1m/s^2 = 100 LSB
    readVector3D(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, 100.0f, x, y, z);
}

void BNO055::getGravity(float *x, float *y, float *z)
{
    // 1m/s^2 = 100 LSB
    readVector3D(BNO055_GRAVITY_DATA_X_LSB_ADDR, 100.0f, x, y, z);
}

void BNO055::getQuaternion(float *w, float *x, float *y, float *z)
{
    // 1 = 2^14 LSB
    uint8_t buffer[8];
    readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
    int16_t wRaw = (((int16_t)buffer[1]) << 8) | ((int16_t)buffer[0]);
    int16_t xRaw = (((int16_t)buffer[3]) << 8) | ((int16_t)buffer[2]);
    int16_t yRaw = (((int16_t)buffer[5]) << 8) | ((int16_t)buffer[4]);
    int16_t zRaw = (((int16_t)buffer[7]) << 8) | ((int16_t)buffer[6]);

    const float scale = (1.0f / (1 << 14));
    *w = wRaw * scale;
    *x = xRaw * scale;
    *y = yRaw * scale;
    *z = zRaw * scale;
}

void BNO055::getEuler(float *roll, float *pitch, float *yaw)
{
    // 1 degree = 16 LSB
    uint8_t buffer[6];
    readLen(BNO055_EULER_H_LSB_ADDR, buffer, 6);
    int16_t hRaw = (((int16_t)buffer[1]) << 8) | ((int16_t)buffer[0]);
    int16_t rRaw = (((int16_t)buffer[3]) << 8) | ((int16_t)buffer[2]);
    int16_t pRaw = (((int16_t)buffer[5]) << 8) | ((int16_t)buffer[4]);

    *yaw = (float)hRaw / 16.0f;
    *roll = (float)rRaw / 16.0f;
    *pitch = (float)pRaw / 16.0f;
}

bool BNO055::readLen(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    Wire.beginTransmission(_address);
    Wire.write(reg);
    uint8_t error = Wire.endTransmission(false);
    if (error != 0)
    {
        Serial.print("[BNO055] readLen Write Error: ");
        Serial.println(error);
        return false;
    }

    uint8_t n = Wire.requestFrom(_address, len);
    if (n != len)
    {
        Serial.print("[BNO055] readLen Request Error. Got ");
        Serial.println(n);
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        buffer[i] = Wire.read();
    }
    return true;
}

void BNO055::readVector3D(uint8_t reg, float lsb, float *x, float *y, float *z)
{
    uint8_t buffer[6];
    if (!readLen(reg, buffer, 6)) return; // Error check

    int16_t xRaw = (((int16_t)buffer[1]) << 8) | ((int16_t)buffer[0]);
    int16_t yRaw = (((int16_t)buffer[3]) << 8) | ((int16_t)buffer[2]);
    int16_t zRaw = (((int16_t)buffer[5]) << 8) | ((int16_t)buffer[4]);

    // DEBUG: Print raw values occasionally
    // static int debugCount = 0;
    // bool suspicious = (xRaw == -1 && yRaw == -1 && zRaw == -1);

    // if (suspicious || (debugCount++ % 100 == 0)) {
    //     if (suspicious) Serial.print("[BNO055] SUSPICIOUS DATA! ");
    //     Serial.print("[BNO055] Raw Reg 0x"); Serial.print(reg, HEX);
    //     Serial.print(": ");
    //     Serial.print(xRaw); Serial.print(", ");
    //     Serial.print(yRaw); Serial.print(", ");
    //     Serial.println(zRaw);

    //     // Check Operation Mode and System Status
    //     uint8_t mode = read(BNO055_OPR_MODE_ADDR);
    //     uint8_t status = read(BNO055_SYS_TRIGGER_ADDR); // Using SYS_TRIGGER as proxy or read 0x39 (SYS_STATUS)
    //     uint8_t sys_stat = read(0x39); // SYS_STATUS register

    //     Serial.print("[BNO055] Mode: 0x"); Serial.print(mode, HEX);
    //     Serial.print(" SysStat: 0x"); Serial.println(sys_stat, HEX);
    // }

    *x = (float)xRaw / lsb;
    *y = (float)yRaw / lsb;
    *z = (float)zRaw / lsb;
}

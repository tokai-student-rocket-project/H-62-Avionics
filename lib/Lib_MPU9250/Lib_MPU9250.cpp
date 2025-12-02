#include "Lib_MPU9250.hpp"

// Registers
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define USER_CTRL 0x6A
#define WHO_AM_I_MPU9250 0x75

// AK8963 Registers
#define AK8963_WHO_AM_I 0x00
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03
#define AK8963_CNTL 0x0A
#define AK8963_ASAX 0x10

MPU9250::MPU9250()
{
    _ascale = AFS_16G;
    _gscale = GFS_2000DPS;
    _dlpf = DLPF_41HZ;
    updateAccelRes();
    updateGyroRes();

    // Mag: 16-bit resolution (0.15 uT per LSB)
    // AK8963 16bit mode: 0.15uT/LSB.
    mRes = 0.15f; // uT per LSB

    // Initialize Accel Calibration
    for(int i=0; i<3; i++) {
        _accelBias[i] = 0.0f;
        _accelScale[i] = 1.0f;
        _gyroBias[i] = 0.0f;
        _magBias[i] = 0.0f;
        _magScale[i] = 1.0f;
        _magBias[i] = 0.0f;
        _magScale[i] = 1.0f;
        _axisMap[i] = i; // Default: 0->0, 1->1, 2->2
        _axisSign[i] = 1.0f; // Default: Positive
        _magMap[i] = i; // Default: 0->0, 1->1, 2->2
        _magSign[i] = 1.0f; // Default: Positive
    }
}

bool MPU9250::begin(Ascale ascale, Gscale gscale, DLPF dlpf, TwoWire *wire)
{
    _ascale = ascale;
    _gscale = gscale;
    _dlpf = dlpf;
    _wire = wire;
    updateAccelRes();
    updateGyroRes();

    // Wire.begin() should be called in setup() by user

    // Check MPU9250 connection
    uint8_t c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    Serial.print("WHO_AM_I: 0x");
    Serial.println(c, HEX);
    if (c != 0x71)
    { // WHO_AM_I should be 0x71
        return false;
    }

    // Reset MPU9250 - Removed to avoid potential I2C bus hang or config loss
    // writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
    // delay(100);

    // Wake up device
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    delay(100);

    // Config gyro and accel
    // DLPF_CFG
    writeByte(MPU9250_ADDRESS, CONFIG, _dlpf);

    // Gyro Config
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, _gscale << 3);
    // Verify Gyro Config
    uint8_t gyro_cfg = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
    Serial.print("GYRO_CONFIG: Wrote 0x"); Serial.print(_gscale << 3, HEX);
    Serial.print(", Read 0x"); Serial.println(gyro_cfg, HEX);

    // Accel Config
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, _ascale << 3);
    // Verify Accel Config
    uint8_t accel_cfg = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
    Serial.print("ACCEL_CONFIG: Wrote 0x"); Serial.print(_ascale << 3, HEX);
    Serial.print(", Read 0x"); Serial.println(accel_cfg, HEX);

    // Accel Config 2: DLPF_CFG
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, _dlpf);

    // Enable Bypass Mode to access AK8963
    // Reset I2C Master and Signal Paths
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x07); // I2C_MST_RST | FIFO_RST | SIG_COND_RST
    delay(100);
    
    // Disable I2C Master mode
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
    delay(10);
    
    // Enable Bypass mode
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);
    delay(10);
    
    // Verify Bypass Config
    uint8_t bypass_cfg = readByte(MPU9250_ADDRESS, INT_PIN_CFG);
    Serial.print("INT_PIN_CFG: Wrote 0x02, Read 0x"); Serial.println(bypass_cfg, HEX);

    // Init AK8963
    uint8_t mag_who = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
    Serial.print("AK8963 WHO_AM_I: 0x");
    Serial.println(mag_who, HEX);
    if (mag_who != 0x48)
    {
        return false;
    }

    // Power down magnetometer
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
    delay(10);

    // Enter Fuse ROM access mode
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);
    delay(10);

    // Read calibration data
    uint8_t rawData[3];
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);
    magCalibration[0] = (float)(rawData[0] - 128) / 256.0f + 1.0f;
    magCalibration[1] = (float)(rawData[1] - 128) / 256.0f + 1.0f;
    magCalibration[2] = (float)(rawData[2] - 128) / 256.0f + 1.0f;
    Serial.print("Mag Calibration: ");
    Serial.print(magCalibration[0]); Serial.print(", ");
    Serial.print(magCalibration[1]); Serial.print(", ");
    Serial.println(magCalibration[2]);

    // Power down magnetometer
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
    delay(10);

    // Set magnetometer to 16 bit resolution, 100Hz continuous measurement
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x16);
    delay(10);
    
    // Verify Mag Config
    uint8_t mag_cntl = readByte(AK8963_ADDRESS, AK8963_CNTL);
    Serial.print("AK8963 CNTL: Wrote 0x16, Read 0x"); Serial.println(mag_cntl, HEX);

    return true;
}

void MPU9250::getAcceleration(float *x, float *y, float *z)
{
    uint8_t rawData[6];
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

    int16_t accel_x = ((int16_t)rawData[0] << 8) | rawData[1];
    int16_t accel_y = ((int16_t)rawData[2] << 8) | rawData[3];
    int16_t accel_z = ((int16_t)rawData[4] << 8) | rawData[5];

    float ax = ((float)accel_x * aRes * 9.80665f - _accelBias[0]) * _accelScale[0];
    float ay = ((float)accel_y * aRes * 9.80665f - _accelBias[1]) * _accelScale[1];
    float az = ((float)accel_z * aRes * 9.80665f - _accelBias[2]) * _accelScale[2];

    *x = ax * _axisSign[0]; // Remapping logic might be complex if we just multiply.
    // Correct logic: Output[0] = Input[Map[0]] * Sign[0]
    // But here Map[0] is the index of the sensor axis that goes to Output X.
    
    // Let's use a temporary array
    float temp[3] = {ax, ay, az};
    *x = temp[_axisMap[0]] * _axisSign[0];
    *y = temp[_axisMap[1]] * _axisSign[1];
    *z = temp[_axisMap[2]] * _axisSign[2];
}

void MPU9250::getGyroscope(float *x, float *y, float *z)
{
    uint8_t rawData[6];
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

    int16_t gyro_x = ((int16_t)rawData[0] << 8) | rawData[1];
    int16_t gyro_y = ((int16_t)rawData[2] << 8) | rawData[3];
    int16_t gyro_z = ((int16_t)rawData[4] << 8) | rawData[5];

    float gx = (float)gyro_x * gRes - _gyroBias[0];
    float gy = (float)gyro_y * gRes - _gyroBias[1];
    float gz = (float)gyro_z * gRes - _gyroBias[2];

    float temp[3] = {gx, gy, gz};
    *x = temp[_axisMap[0]] * _axisSign[0];
    *y = temp[_axisMap[1]] * _axisSign[1];
    *z = temp[_axisMap[2]] * _axisSign[2];
}

void MPU9250::getMagnetometer(float *x, float *y, float *z)
{
    uint8_t rawData[7];
    uint8_t st1 = readByte(AK8963_ADDRESS, AK8963_ST1);
    // Wait for magnetometer data ready bit to be set
    if (st1 & 0x01)
    {
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if (!(c & 0x08))
        { // Check if magnetic sensor overflow set, if not then report data
            // Standard MPU9250 / AK8963 alignment:
            // AK8963 X axis is aligned with MPU9250 Y axis
            // AK8963 Y axis is aligned with MPU9250 X axis
            // AK8963 Z axis is aligned with MPU9250 -Z axis
            // We need to align Mag to Accel/Gyro frame first.
            
            int16_t mag_x = ((int16_t)rawData[1] << 8) | rawData[0];
            int16_t mag_y = ((int16_t)rawData[3] << 8) | rawData[2];
            int16_t mag_z = ((int16_t)rawData[5] << 8) | rawData[4];

            float mag_x_adj = (float)mag_x * mRes * magCalibration[0] - _magBias[0];
            float mag_y_adj = (float)mag_y * mRes * magCalibration[1] - _magBias[1];
            float mag_z_adj = (float)mag_z * mRes * magCalibration[2] - _magBias[2];
            
            // Standard MPU9250 / AK8963 alignment:
            // AK8963 X axis is aligned with MPU9250 Y axis
            // AK8963 Y axis is aligned with MPU9250 X axis
            // AK8963 Z axis is aligned with MPU9250 -Z axis
            // However, user requested to return to "regulation" (datasheet raw output).
            // So we output AK8963 axes as they are, without internal swapping.
            
            float mx = mag_x_adj * _magScale[0];
            float my = mag_y_adj * _magScale[1];
            float mz = mag_z_adj * _magScale[2];

            float temp[3] = {mx, my, mz};
            *x = temp[_magMap[0]] * _magSign[0];
            *y = temp[_magMap[1]] * _magSign[1];
            *z = temp[_magMap[2]] * _magSign[2];
        } else {
             Serial.print("Mag Overflow. ST1: 0x"); Serial.print(st1, HEX);
             Serial.print(", ST2: 0x"); Serial.println(c, HEX);
             Serial.print("Raw: ");
             for(int i=0; i<7; i++) {
                 Serial.print(rawData[i], HEX); Serial.print(" ");
             }
             Serial.println();
        }
    } else {
        // Serial.print("Mag Not Ready: 0x"); Serial.println(st1, HEX);
    }
}

void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    _wire->beginTransmission(address);
    _wire->write(subAddress);
    _wire->write(data);
    _wire->endTransmission();
}

uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data;
    _wire->beginTransmission(address);
    _wire->write(subAddress);
    _wire->endTransmission(false);
    _wire->requestFrom(address, (uint8_t)1);
    data = _wire->read();
    return data;
}

void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    _wire->beginTransmission(address);
    _wire->write(subAddress);
    _wire->endTransmission(false);
    uint8_t i = 0;
    _wire->requestFrom(address, count);
    while (_wire->available())
    {
        dest[i++] = _wire->read();
    }
}

void MPU9250::setAccelRange(Ascale ascale)
{
    _ascale = ascale;
    updateAccelRes();
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, _ascale << 3);
}

void MPU9250::setGyroRange(Gscale gscale)
{
    _gscale = gscale;
    updateGyroRes();
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, _gscale << 3);
}

void MPU9250::setDLPFBandwidth(DLPF dlpf)
{
    _dlpf = dlpf;
    writeByte(MPU9250_ADDRESS, CONFIG, _dlpf);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, _dlpf);
}

void MPU9250::updateAccelRes()
{
    switch (_ascale)
    {
    case AFS_2G:
        aRes = 2.0f / 32768.0f;
        break;
    case AFS_4G:
        aRes = 4.0f / 32768.0f;
        break;
    case AFS_8G:
        aRes = 8.0f / 32768.0f;
        break;
    case AFS_16G:
        aRes = 16.0f / 32768.0f;
        break;
    }
}

void MPU9250::updateGyroRes()
{
    switch (_gscale)
    {
    case GFS_250DPS:
        gRes = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gRes = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gRes = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gRes = 2000.0f / 32768.0f;
        break;
    }
}

void MPU9250::setAccelCalibration(float biasX, float biasY, float biasZ, float scaleX, float scaleY, float scaleZ) {
    _accelBias[0] = biasX;
    _accelBias[1] = biasY;
    _accelBias[2] = biasZ;
    _accelScale[0] = scaleX;
    _accelScale[1] = scaleY;
    _accelScale[2] = scaleZ;
}

void MPU9250::calibrateAccel() {
    Serial.println("Starting Accelerometer Calibration...");
    Serial.println("Please rotate the sensor to 6 positions (X+, X-, Y+, Y-, Z+, Z-).");
    Serial.println("Send any character to continue to next step.");

    float maxVal[3] = {-1000.0f, -1000.0f, -1000.0f};
    float minVal[3] = {1000.0f, 1000.0f, 1000.0f};
    float ax, ay, az;

    const char* steps[] = {
        "Place Z+ (Flat up)", 
        "Place Z- (Flat down)", 
        "Place X+ (Connector up)", 
        "Place X- (Connector down)", 
        "Place Y+ (Right side up)", 
        "Place Y- (Left side up)"
    };

    for (int i = 0; i < 6; i++) {
        Serial.print("Step "); Serial.print(i+1); Serial.print(": "); Serial.println(steps[i]);
        while (!Serial.available()) delay(10);
        while (Serial.available()) Serial.read(); // Clear buffer

        Serial.println("Measuring...");
        for (int j = 0; j < 100; j++) {
            getAcceleration(&ax, &ay, &az);
            // Temporarily remove current calibration to get raw values
            // Revert formula: raw = val / scale + bias
            // But since we are inside the class, we can just read raw registers or use getAcceleration with identity calibration.
            // However, getAcceleration already applies calibration. 
            // To get raw values for calibration, we should temporarily reset calibration.
            // But simpler: just use the raw register reading logic here.
            
            uint8_t rawData[6];
            readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
            int16_t raw_x = ((int16_t)rawData[0] << 8) | rawData[1];
            int16_t raw_y = ((int16_t)rawData[2] << 8) | rawData[3];
            int16_t raw_z = ((int16_t)rawData[4] << 8) | rawData[5];
            
            float accX = (float)raw_x * aRes * 9.80665f;
            float accY = (float)raw_y * aRes * 9.80665f;
            float accZ = (float)raw_z * aRes * 9.80665f;

            if (accX > maxVal[0]) maxVal[0] = accX;
            if (accX < minVal[0]) minVal[0] = accX;
            if (accY > maxVal[1]) maxVal[1] = accY;
            if (accY < minVal[1]) minVal[1] = accY;
            if (accZ > maxVal[2]) maxVal[2] = accZ;
            if (accZ < minVal[2]) minVal[2] = accZ;
            
            delay(10);
        }
        Serial.println("Done.");
    }

    // Calculate Bias and Scale
    // Target is 9.80665 m/s^2 (1G)
    float gravity = 9.80665f;
    
    for(int k=0; k<3; k++) {
        _accelBias[k] = (maxVal[k] + minVal[k]) / 2.0f;
        _accelScale[k] = gravity / ((maxVal[k] - minVal[k]) / 2.0f);
    }

    Serial.println("Calibration Complete!");
    Serial.println("Copy these lines to your setup():");
    Serial.print("mpu.setAccelCalibration(");
    Serial.print(_accelBias[0], 4); Serial.print(", ");
    Serial.print(_accelBias[1], 4); Serial.print(", ");
    Serial.print(_accelBias[2], 4); Serial.print(", ");
    Serial.print(_accelScale[0], 4); Serial.print(", ");
    Serial.print(_accelScale[1], 4); Serial.print(", ");
    Serial.print(_accelScale[2], 4); Serial.println(");");
}

void MPU9250::setGyroBias(float biasX, float biasY, float biasZ) {
    _gyroBias[0] = biasX;
    _gyroBias[1] = biasY;
    _gyroBias[2] = biasZ;
}

void MPU9250::calibrateGyro() {
    Serial.println("Starting Gyroscope Calibration...");
    Serial.println("Please keep the sensor stationary and flat.");
    Serial.println("Send any character to start.");
    
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read(); // Clear buffer

    Serial.println("Measuring...");
    
    float gx, gy, gz;
    float sumX = 0, sumY = 0, sumZ = 0;
    int samples = 1000;

    for (int i = 0; i < samples; i++) {
        // Read raw gyro data directly to avoid applying existing bias
        uint8_t rawData[6];
        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

        int16_t raw_x = ((int16_t)rawData[0] << 8) | rawData[1];
        int16_t raw_y = ((int16_t)rawData[2] << 8) | rawData[3];
        int16_t raw_z = ((int16_t)rawData[4] << 8) | rawData[5];

        sumX += (float)raw_x * gRes;
        sumY += (float)raw_y * gRes;
        sumZ += (float)raw_z * gRes;
        
        delay(2);
    }

    _gyroBias[0] = sumX / samples;
    _gyroBias[1] = sumY / samples;
    _gyroBias[2] = sumZ / samples;

    Serial.println("Calibration Complete!");
    Serial.println("Copy these lines to your setup():");
    Serial.print("mpu.setGyroBias(");
    Serial.print(_gyroBias[0], 4); Serial.print(", ");
    Serial.print(_gyroBias[1], 4); Serial.print(", ");
    Serial.print(_gyroBias[2], 4); Serial.println(");");
}

void MPU9250::setMagCalibration(float biasX, float biasY, float biasZ, float scaleX, float scaleY, float scaleZ) {
    _magBias[0] = biasX;
    _magBias[1] = biasY;
    _magBias[2] = biasZ;
    _magScale[0] = scaleX;
    _magScale[1] = scaleY;
    _magScale[2] = scaleZ;
}

void MPU9250::calibrateMag() {
    Serial.println("Starting Magnetometer Calibration...");
    Serial.println("Please rotate the sensor in a figure-8 motion covering all directions.");
    Serial.println("Send any character to start measuring (15 seconds).");
    
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read(); // Clear buffer

    Serial.println("Measuring...");
    
    float maxVal[3] = {-10000.0f, -10000.0f, -10000.0f};
    float minVal[3] = {10000.0f, 10000.0f, 10000.0f};
    float mx, my, mz;
    
    unsigned long startTime = millis();
    while (millis() - startTime < 15000) { // 15 seconds
        // Use raw read logic to avoid applying existing calibration
        uint8_t rawData[7];
        uint8_t st1 = readByte(AK8963_ADDRESS, AK8963_ST1);
        if (st1 & 0x01) {
            readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
            uint8_t c = rawData[6];
            if (!(c & 0x08)) {
                int16_t mag_x = ((int16_t)rawData[1] << 8) | rawData[0];
                int16_t mag_y = ((int16_t)rawData[3] << 8) | rawData[2];
                int16_t mag_z = ((int16_t)rawData[5] << 8) | rawData[4];

                float mX = (float)mag_x * mRes * magCalibration[0];
                float mY = (float)mag_y * mRes * magCalibration[1];
                float mZ = (float)mag_z * mRes * magCalibration[2];

                if (mX > maxVal[0]) maxVal[0] = mX;
                if (mX < minVal[0]) minVal[0] = mX;
                if (mY > maxVal[1]) maxVal[1] = mY;
                if (mY < minVal[1]) minVal[1] = mY;
                if (mZ > maxVal[2]) maxVal[2] = mZ;
                if (mZ < minVal[2]) minVal[2] = mZ;
            }
        }
        delay(10);
    }

    // Hard Iron Correction (Bias)
    _magBias[0] = (maxVal[0] + minVal[0]) / 2.0f;
    _magBias[1] = (maxVal[1] + minVal[1]) / 2.0f;
    _magBias[2] = (maxVal[2] + minVal[2]) / 2.0f;

    // Soft Iron Correction (Scale)
    float chordX = (maxVal[0] - minVal[0]) / 2.0f;
    float chordY = (maxVal[1] - minVal[1]) / 2.0f;
    float chordZ = (maxVal[2] - minVal[2]) / 2.0f;

    float avgChord = (chordX + chordY + chordZ) / 3.0f;

    _magScale[0] = avgChord / chordX;
    _magScale[1] = avgChord / chordY;
    _magScale[2] = avgChord / chordZ;

    Serial.println("Calibration Complete!");
    Serial.println("Copy these lines to your setup():");
    Serial.print("mpu.setMagCalibration(");
    Serial.print(_magBias[0], 4); Serial.print(", ");
    Serial.print(_magBias[1], 4); Serial.print(", ");
    Serial.print(_magBias[2], 4); Serial.print(", ");
    Serial.print(_magScale[0], 4); Serial.print(", ");
    Serial.print(_magScale[1], 4); Serial.print(", ");
    Serial.print(_magScale[2], 4); Serial.println(");");
    Serial.print(_magScale[2], 4); Serial.println(");");
}

void MPU9250::setAxisRemap(uint8_t xMap, uint8_t yMap, uint8_t zMap, float xSign, float ySign, float zSign) {
    _axisMap[0] = xMap;
    _axisMap[1] = yMap;
    _axisMap[2] = zMap;
    _axisSign[0] = xSign;
    _axisSign[1] = ySign;
    _axisSign[2] = zSign;
}

void MPU9250::setMagRemap(uint8_t xMap, uint8_t yMap, uint8_t zMap, float xSign, float ySign, float zSign) {
    _magMap[0] = xMap;
    _magMap[1] = yMap;
    _magMap[2] = zMap;
    _magSign[0] = xSign;
    _magSign[1] = ySign;
    _magSign[2] = zSign;
}

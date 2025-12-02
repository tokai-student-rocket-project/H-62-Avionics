#include <Arduino.h>
#include <TaskManager.h>
#include <Lib_BNO055.hpp>
#include <BNO055_Calibration.hpp>
#include <Lib_Madgwick.hpp>

BNO055 bno(0x28);
Madgwick madgwick;

uint8_t sys, gyro, accel, mag;
float roll, pitch, yaw;
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

void doCalibration()
{
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("Sys:");
    Serial.print(sys);
    Serial.print(" G:");
    Serial.print(gyro);
    Serial.print(" A:");
    Serial.print(accel);
    Serial.print(" M:");
    Serial.print(mag);
    Serial.println("");

    if (bno.isFullyCalibrated())
    {
        Serial.println("\nFully Calibrated!");

        BNO055::bno055_offsets_t offsets;
        bno.getSensorOffsets(offsets);

        Serial.println("--- Calibration Offsets ---");
        Serial.print("Accel: ");
        Serial.print(offsets.accel_offset_x);
        Serial.print(", ");
        Serial.print(offsets.accel_offset_y);
        Serial.print(", ");
        Serial.println(offsets.accel_offset_z);

        Serial.print("Gyro: ");
        Serial.print(offsets.gyro_offset_x);
        Serial.print(", ");
        Serial.print(offsets.gyro_offset_y);
        Serial.print(", ");
        Serial.println(offsets.gyro_offset_z);

        Serial.print("Mag: ");
        Serial.print(offsets.mag_offset_x);
        Serial.print(", ");
        Serial.print(offsets.mag_offset_y);
        Serial.print(", ");
        Serial.println(offsets.mag_offset_z);

        Serial.print("Accel Radius: ");
        Serial.println(offsets.accel_radius);
        Serial.print("Mag Radius: ");
        Serial.println(offsets.mag_radius);
        Serial.println("---------------------------");

        while (1)
            ; // 完了したら停止
    }
}

void NDOF_MODE()
{
    bno.getEuler(&roll, &pitch, &yaw);

    Serial.print(">Roll:");
    Serial.println(roll);
    Serial.print(">Pitch:");
    Serial.println(pitch);
    Serial.print(">Yaw:");
    Serial.println(yaw);
}

void AMG_MODE()
{
    // 加速度 (m/s^2)
    bno.getAcceleration(&ax, &ay, &az);

    // ジャイロ (dps)
    bno.getGyroscope(&gx, &gy, &gz);

    // 磁気 (uT)
    bno.getMagnetometer(&mx, &my, &mz);

    madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    float yaw = madgwick.getRoll();
    float pitch = madgwick.getPitch();
    float roll = madgwick.getYaw();

    Serial.print(">Roll:");
    Serial.println(roll);
    Serial.print(">Pitch:");
    Serial.println(pitch);
    Serial.print(">Yaw:");
    Serial.println(yaw);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    Wire.begin();
    Serial.println("\n--- I2C Scanner ---");
    byte error, address;
    int nDevices = 0;

    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");
            nDevices++;
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    Serial.println("Initializing BNO055...");

    if (!bno.begin(BNO055::OPERATION_MODE_AMG))
    {
        Serial.println("BNO055 init failed. Check wiring or I2C address.");
        while (1)
            delay(100);
    }
    Serial.println("BNO055 init success!");

    bno.setSensorOffsets(BNO055_CALIBRATION_PROFILE);
    bno.setAxisRemap(BNO055::REMAP_CONFIG_P0, BNO055::REMAP_SIGN_P0);
    bno.setExtCrystalUse(true);

    madgwick.begin(30);
    // Tasks.add(&NDOF_MODE)->startFps(100);
    Tasks.add(&AMG_MODE)->startFps(30);
    // Tasks.add(&doCalibration)->startFps(10);
}

void loop()
{
    Tasks.update();
}

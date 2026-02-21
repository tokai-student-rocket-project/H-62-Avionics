#include <Lib_MPU9250.hpp>
#include <Arduino.h>
#include <Lib_Madgwick.hpp>
#include <TaskManager.h>
#include <Wire.h>
#include <Lib_FlightTime.hpp>
#include <Lib_EKF.hpp>

MPU9250 mpu9250;
Madgwick madgwickFilter;
FlightTime flightTime;
EKF ekf;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float lin_ax, lin_ay, lin_az;
float ggx, ggy, ggz;

const float SAMPLE_RATE = 100.0f;

void task100Hz()
{
    static int count = 0;

    // 加速度 (m/s^2)
    mpu9250.getAcceleration(&ax, &ay, &az);

    // 角速度 (dps)
    mpu9250.getGyroscope(&gx, &gy, &gz);

    // 磁束密度 (uT)
    mpu9250.getMagnetometer(&mx, &my, &mz);

    // 重力ベクトル
    madgwickFilter.getGravity(&ggx, &ggy, &ggz);

    // 重力を差し引いた加速度
    madgwickFilter.getLinearAcceleration(ax, ay, az, &lin_ax, &lin_ay, &lin_az);

    // Serial.print(">Accel_X_m/s^2:");
    // Serial.println(ax);
    // Serial.print(">Accel_Y_m/s^2:");
    // Serial.println(ay);
    // Serial.print(">Accel_Z_m/s^2:");
    // Serial.println(az);

    // Serial.print(">Gyro_X_dps:");
    // Serial.println(gx);
    // Serial.print(">Gyro_Y_dps:");
    // Serial.println(gy);
    // Serial.print(">Gyro_Z_dps:");
    // Serial.println(gz);

    // Serial.print(">Mag_X_uT:");
    // Serial.println(mx);
    // Serial.print(">Mag_Y_uT:");
    // Serial.println(my);
    // Serial.print(">Mag_Z_uT:");
    // Serial.println(mz);

    madgwickFilter.update(gx, gy, gz, ax, ay, az, my, mx, -mz);
    ekf.update(gx, gy, gz, ax, ay, az, my, mx, -mz);

    if (count++ % 5 == 0)
    {
        // float roll = madgwickFilter.getYaw();
        // float pitch = madgwickFilter.getPitch();
        // float yaw = madgwickFilter.getRoll();

        float roll = ekf.getYaw();
        float pitch = ekf.getPitch();
        float yaw = ekf.getRoll();

        Serial.print(">Roll:");
        Serial.println(roll);
        Serial.print(">Pitch:");
        Serial.println(pitch);
        Serial.print(">Yaw:");
        Serial.println(yaw);
        // Serial.print(">Gravity_X:");
        // Serial.println(ax);

        // Serial.print(">Linear_Accel_X:");
        // Serial.println(lin_ax);
        // Serial.print(">Linear_Accel_Y:");
        // Serial.println(lin_ay);
        // Serial.print(">Linear_Accel_Z:");
        // Serial.println(lin_az);
    }
}

void taskCsv()
{
    // 加速度 (m/s^2)
    mpu9250.getAcceleration(&ax, &ay, &az);

    // 角速度 (dps)
    mpu9250.getGyroscope(&gx, &gy, &gz);

    // 磁束密度 (uT)
    mpu9250.getMagnetometer(&mx, &my, &mz);

    // madgwickFilter.updateIMU(gx, gy, gz, ax, ay, az);
    madgwickFilter.update(gz, gy, gx, az, ay, ay, my, mx, -mz);

    float roll = madgwickFilter.getRoll();
    float pitch = madgwickFilter.getPitch();
    float yaw = madgwickFilter.getYaw();

    Serial.print(flightTime.get() / 1000.0);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(yaw); // 最後は改行
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    while (!Serial)
        ;

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    mpu9250.begin();
    mpu9250.setAccelRange(MPU9250::AFS_16G);
    mpu9250.setGyroRange(MPU9250::GFS_1000DPS);
    mpu9250.setDLPFBandwidth(MPU9250::DLPF_41HZ);

    // キャリブレーション実行
    // mpu9250.calibrateAccel();
    // mpu9250.calibrateGyro();
    // mpu9250.calibrateMag();

    // madgwickFilter.begin(SAMPLE_RATE);
    // madgwickFilter.setZeta(0);
    ekf.begin(SAMPLE_RATE);

    mpu9250.setAccelCalibration(0.0670, 0.0287, 0.3400, 0.9757, 0.9913, 0.9832);
    mpu9250.setGyroBias(-0.1145, 0.6864, 0.3636);
    mpu9250.setMagCalibration(6.4066, -20.2529, 2.4527, 0.5614, 1.5111, 1.7952);

    // Serial.println("Accel_X_m/s^2,Accel_Y_m/s^2,Accel_Z_m/s^2,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps,Mag_X_uT,Mag_Y_uT,Mag_Z_uT");
    Serial.println("time,roll,pitch,yaw");

    Tasks.add(&task100Hz)->startFps(100);
    // Tasks.add(&taskCsv)->startFps(100);
}

void loop()
{
    Tasks.update();
}
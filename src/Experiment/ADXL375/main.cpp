#include <Arduino.h>
#include <SPI.h>
#include <Lib_ADXL375.hpp>

// CSピンの定義 (STM32F756ZG D10)
#define ADXL375_CS D10

// ADXL375インスタンスの作成 (ハードウェアSPI)
ADXL375 accel(ADXL375_CS, &SPI);

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(100);

    Serial.println("ADXL375 SPI Sample Start");

    // SPI通信の初期化
    SPI.begin();

    // センサの初期化
    if (!accel.begin())
    {
        Serial.println("ADXL375 not found. Check wiring!");
        while (1)
            ;
    }
    Serial.println("ADXL375 Initialized");

    // 保存されたオフセットがあればここで設定する
    accel.setTrimOffsets(1, -3, -5);

    Serial.println("Send 'c' to Calibrate.");
}

void calibrateSensor()
{
    Serial.println("Calibration starting...");
    Serial.println("Keep sensor flat and stationary (Z should be +1g).");
    delay(2000); // 準備待ち

    long x_sum = 0, y_sum = 0, z_sum = 0;
    int samples = 100;

    Serial.println("Sampling...");
    for (int i = 0; i < samples; i++)
    {
        int16_t x, y, z;
        accel.getXYZ(&x, &y, &z);
        x_sum += x;
        y_sum += y;
        z_sum += z;
        delay(10);
    }

    float x_avg = x_sum / (float)samples;
    float y_avg = y_sum / (float)samples;
    float z_avg = z_sum / (float)samples;

    Serial.print("Average RAW: X=");
    Serial.print(x_avg);
    Serial.print(" Y=");
    Serial.print(y_avg);
    Serial.print(" Z=");
    Serial.println(z_avg);

    // 目標値: X=0, Y=0, Z=20.4 (1g @ 49mg/LSB) -> 約20
    // オフセット計算: (Target - Measured) / OffsetScale
    // ADXL375 Offset Scale = 196 mg/LSB = 4 * 49mg/LSB (approx)
    // したがって、RAW値の差分を 4 で割ればよい (おおよそ)

    // 正確には:
    // Offset_Reg_Value = (Target_g - Measured_g) / 0.196g
    // Target_g for X,Y = 0
    // Target_g for Z = 1.0 (assuming flat table)

    // Measured_g = average_raw * 0.049

    // Offset X = (0 - x_avg * 0.049) / 0.196 = -x_avg * (0.049/0.196) = -x_avg / 4
    // Offset Y = -y_avg / 4
    // Offset Z = (1.0 - z_avg * 0.049) / 0.196 = (20.4*0.049 - z_avg*0.049) / 0.196 = (20.4 - z_avg) / 4

    int8_t off_x = round(-x_avg / 4.0);
    int8_t off_y = round(-y_avg / 4.0);
    int8_t off_z = round((20.4 - z_avg) / 4.0); // Z target is +1g (~20LSB)

    Serial.print("Calculated Offsets: X=");
    Serial.print(off_x);
    Serial.print(" Y=");
    Serial.print(off_y);
    Serial.print(" Z=");
    Serial.println(off_z);

    accel.setTrimOffsets(off_x, off_y, off_z);
    Serial.println("Offsets Applied!");
    Serial.println("----------------------------------------");
    Serial.println("To save permanently, add this line to setup():");
    Serial.print("accel.setTrimOffsets(");
    Serial.print(off_x);
    Serial.print(", ");
    Serial.print(off_y);
    Serial.print(", ");
    Serial.print(off_z);
    Serial.println(");");
    Serial.println("----------------------------------------");
}

void loop()
{
    if (Serial.available())
    {
        char ch = Serial.read();
        if (ch == 'c')
        {
            calibrateSensor();
        }
    }

    // 加速度データの取得 (単位: m/s^2)
    float x_acc, y_acc, z_acc;
    accel.getAcceleration(&x_acc, &y_acc, &z_acc);

    // データの表示
    Serial.print("> X: ");
    Serial.println(x_acc);
    Serial.print("> Y: ");
    Serial.println(y_acc);
    Serial.print("> Z: ");
    Serial.println(z_acc);

    // RAW値も確認したい場合は以下を使用
    // int16_t x, y, z;
    // accel.getXYZ(&x, &y, &z);
    // Serial.print(" | RAW Z: "); Serial.print(z);
}

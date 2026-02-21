#include <Arduino.h>
#include <SPI.h>
#include <Lib_ADXL375.hpp>
#include <TaskManager.h>
// CSピンの定義 (STM32F756ZG D10)
#define ADXL375_CS D10

// ADXL375インスタンスの作成 (ハードウェアSPI)
ADXL375 accel(ADXL375_CS, &SPI);

// ピンの動作確認用
static uint32_t pinName = PG1;
static uint32_t ledBlue = PB7;
static uint32_t ledRed = PB14;

void calibrateSensor();
void task800Hz();
void getPinStatus();
void spiPinConfig();
bool checkFreeFall();

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

bool checkFreeFall()
{
    float x_acc, y_acc, z_acc;
    accel.getAcceleration(&x_acc, &y_acc, &z_acc);

    // G単位に変換 ( m/s^2 を 9.80665 で割る)
    float acc_mag = sqrt(x_acc * x_acc + y_acc * y_acc + z_acc * z_acc) / 9.80665f;

    static uint32_t fallStartTime = 0;
    const float threshold = 0.3f; // 閾値を 0.25G に設定
    const uint32_t requiredTime = 10;

    if (acc_mag < threshold)
    {
        if (fallStartTime == 0)
            fallStartTime = millis();
        if (millis() - fallStartTime > requiredTime)
        {
            return true;
        }
    }
    else
    {
        fallStartTime = 0;
    }
    return false;
}

void task800Hz()
{
    static uint32_t startTime = millis();

    // 加速度データの取得 (単位: m/s^2)
    float x_acc, y_acc, z_acc;
    accel.getAcceleration(&x_acc, &y_acc, &z_acc);

    // データの表示
    Serial.print("> Time: ");
    Serial.println((millis() - startTime) / 1000.0);
    Serial.print("> X: ");
    Serial.println(x_acc);
    Serial.print("> Y: ");
    Serial.println(y_acc);
    Serial.print("> Z: ");
    Serial.println(z_acc);

    // CSV形式で出力: 時間[ms], X, Y, Z

    // Serial.print((millis() - startTime) / 1000);
    // Serial.print(",");
    // Serial.print(x_acc);
    // Serial.print(",");
    // Serial.print(y_acc);
    // Serial.print(",");
    // Serial.println(z_acc);
    Serial.print("> Freefall: ");
    Serial.println(checkFreeFall());
}

void getPinStatus()
{
    digitalWrite(pinName, !digitalRead(pinName));
    if (digitalRead(pinName) == 1)
    {
        digitalWrite(ledBlue, HIGH);
        digitalWrite(ledRed, HIGH);
    }
    else
    {
        digitalWrite(ledBlue, LOW);
        digitalWrite(ledRed, LOW);
    }

    Serial.println(digitalRead(pinName));
}

void spiPinConfig()
{
    SPI.setMISO(PB4);
    SPI.setMOSI(PB5);
    SPI.setSCLK(PB3);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(100);

    // SPI通信の初期化
    spiPinConfig();
    SPI.begin();

    // センサの初期化
    if (!accel.begin())
    {
        while (1)
            ;
    }

    pinMode(pinName, OUTPUT);
    pinMode(ledBlue, OUTPUT);
    pinMode(ledRed, OUTPUT);

    // 保存されたオフセットがあればここで設定する
    accel.setTrimOffsets(1, -3, -5);

    // CSVヘッダーの出力
    Serial.println("time_ms,acc_x,acc_y,acc_z");

    Tasks.add(&task800Hz)->startFps(800);
    Tasks.add(&getPinStatus)->startFps(2);
}

void loop()
{
    Tasks.update();

    if (Serial.available())
    {
        char ch = Serial.read();
        if (ch == 'c')
        {
            calibrateSensor();
        }
    }
}

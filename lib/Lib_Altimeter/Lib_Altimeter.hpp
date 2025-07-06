#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LPS28.h>

class Altimeter
{
public:
    void initialize(uint32_t i2cAddress);
    void getConfiguration();

    float getPressure();
    float getTemperature();

    void setAutoReferencePressure(bool enable);
    void setReferencePressure(float referencePressure_kPa);
    void resetAutoReferencePressure();
    void setAutozero(bool enable);
    void resetAutozero();
    float getAltitude(float temperature_C);
    int16_t getReferencePressure();

private:
    Adafruit_LPS28 *_lps28;

    float _pressure_hPa;
    int16_t _referencePressure_hPa;
};
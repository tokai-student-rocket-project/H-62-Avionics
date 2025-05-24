#include "Lib_Altimeter.hpp"

void Altimeter::initialize(uint32_t i2cAddress) {
    uint32_t _i2cAddress = i2cAddress;

    _lps28 = new Adafruit_LPS28();
    _lps28->begin(&Wire, _i2cAddress);
    _lps28->setDataRate(LPS28_ODR_200_HZ);
    _lps28->setAveraging(LPS28_AVG_512);
    _lps28->setLowPassODR9(true);
    // _lps28->setFIFOmode(false, LPS28_FIFO_CONTINUOUS);
}

void Altimeter::getConfiguration() {
    Serial.print("FullScaleMode: ");
    Serial.println(_lps28->getFullScaleMode() ? "ON" : "OFF");
    Serial.print("CurrentSensorStatus: ");
    Serial.println(_lps28->getStatus(), HEX);
}

float Altimeter::getPressure() {
    _pressure_kPa = _lps28->getPressure(); // getPressure() は[hPa]で帰ってくる
    return _pressure_kPa;
}

float Altimeter::getTemperature() {
    return _lps28->getTemperature(); // getTemperature() は[°C]で帰ってくる
}

void Altimeter::setReferencePressure(float referencePressure_kPa) {
  _referencePressure_kPa = referencePressure_kPa;
}

void Altimeter::setAutoReferencePressure(bool enable) {
    _lps28->setAutoReferencePressure(enable);
    if (_lps28->getAutoReferencePressure() == false){
        Serial.println("AutoReferencePressure: DISABLE");
    }
    _referencePressure_kPa = _lps28->getReferencePressure(); // getReferencePressure() は[Pa]で帰ってくる
}

void Altimeter::resetAutoReferencePressure() {
    _lps28->resetAutoReferencePressure();
}

void Altimeter::setAutozero(bool enable) {
    _lps28->setAutoZero(enable);
    if (_lps28->getAutoZero() == false){
        Serial.println("AutoZero: DISABLE");
    }
}

void Altimeter::resetAutozero() {
    _lps28->resetAutoZero();
}

float Altimeter::getAltitude(float temperature_C) {
    float altitude_m = (((pow((_referencePressure_kPa / _pressure_kPa), (1.0 / 5.257))) - 1.0) * (temperature_C + 273.15)) / 0.0065;
    // float altitude_m = (((pow((1013.25 / _pressure_kPa), (1.0 / 5.257))) - 1.0) * temperature_C + 273.15) / 0.0065;
    return altitude_m;
}

int16_t Altimeter::getReferencePressure() {
    return _lps28->getReferencePressure();
}


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LPS28.h>
#include <TaskManager.h>
#include "Lib_Altimeter.hpp"


namespace sensor {
    Altimeter primary;
    Altimeter secondary;

    void measurePressure();
    void referencePressure();
}

void sensor::measurePressure(){
  sensor::primary.setReferencePressure(1013.25);
  Serial.print(">Primary Pressure: ");
  Serial.println(sensor::primary.getPressure());

  Serial.print(">Sencondary Pressure: ");
  Serial.println(sensor::secondary.getPressure());

  float temperature_C = (sensor::primary.getTemperature()+sensor::secondary.getTemperature()) / 2;
  Serial.print(">Altitude: ");
  Serial.println(sensor::primary.getAltitude(temperature_C));

  Serial.print(">Primary ReferencePressure: ");
  Serial.println(sensor::primary.getReferencePressure()); 

  Serial.print(">Secondary ReferencePressure: ");
  Serial.println(sensor::secondary.getReferencePressure());
}

void setup() {
  Serial.begin(115200);

  sensor::primary.initialize(0x5C);
  sensor::secondary.initialize(0x5D);



  Tasks.add(&sensor::measurePressure)->startFps(400); // 400Hz
}

void loop() {
  Tasks.update();
}
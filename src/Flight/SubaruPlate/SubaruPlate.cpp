#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LPS28.h>
#include <TaskManager.h>


namespace sensor {
    Adafruit_LPS28 primaryPressure;
    Adafruit_LPS28 secondaryPressure;

    void measurePressure();
}

void sensor::measurePressure(){


  float pressurePrimary = sensor::primaryPressure.getPressure();
  float temperaturePrimary = sensor::primaryPressure.getTemperature();

  Serial.print(">Pressure_MAIN:");
  Serial.println(pressurePrimary);
//   Serial.print(", ");
//   Serial.print("Temperature (°C): ");
//   Serial.println(temperaturePrimary);

  float pressureSecondary = sensor::secondaryPressure.getPressure();
  float temperatureSecondary = sensor::secondaryPressure.getTemperature();

  Serial.print(">Pressure_BACKUP:");
  Serial.println(pressureSecondary);
//   Serial.print(", ");
//   Serial.print("Temperature (°C): ");
//   Serial.println(temperatureSecondary);
}

void setup() {
  Serial.begin(115200);

  // Initialize the sensor
  sensor::primaryPressure.begin(&Wire, 0x5C);
  sensor::secondaryPressure.begin(&Wire, 0x5D);

  sensor::primaryPressure.setDataRate(LPS28_ODR_200_HZ);
  sensor::primaryPressure.setAveraging(LPS28_AVG_512);
//   sensor::primaryPressure.setLowPassODR9(true);
  sensor::primaryPressure.setFIFOmode(false, LPS28_FIFO_BYPASS_TO_CONTINUOUS);
  sensor::primaryPressure.setFIFOWatermark(10);

  sensor::secondaryPressure.setDataRate(LPS28_ODR_200_HZ);
  sensor::secondaryPressure.setAveraging(LPS28_AVG_512);
  sensor::secondaryPressure.setLowPassODR9(true);
  sensor::secondaryPressure.setFIFOmode(false, LPS28_FIFO_CONTINUOUS);
  sensor::secondaryPressure.setFIFOWatermark(10);



  // Enable DRDY interrupt on the interrupt pin
  sensor::primaryPressure.setInterruptPin(
    true,  // Polarity: Active high
    false  // Pin mode: Push-pull
  );

  sensor::secondaryPressure.setInterruptPin(
    true,  // Polarity: Active high
    false  // Pin mode: Push-pull
  );

  // Enable DRDY interrupt output on INT pin (we could use this with an interrupt)
  sensor::primaryPressure.setIntPinOutput(
    false,  // DRDY active
    false, // DRDY pulse not enabled
    false, // INT output not enabled
    false, // FIFO full interrupt not enabled
    true, // FIFO watermark interrupt not enabled
    false  // FIFO overrun interrupt not enabled
  );

  sensor::secondaryPressure.setIntPinOutput(
    true,  // DRDY active
    false, // DRDY pulse not enabled
    false, // INT output not enabled
    false, // FIFO full interrupt not enabled
    true, // FIFO watermark interrupt not enabled
    false  // FIFO overrun interrupt not enabled
  );

  Tasks.add(&sensor::measurePressure)->startFps(400); // 200Hz
}

void loop() {
  Tasks.update();
}
#include <Arduino.h>
#include <MAX17055.h>
#include <Wire.h>

#define GAUGE_SDA 33
#define GAUGE_SCL 22

#define USR_LED 32

MAX17055 gauge;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(GAUGE_SDA, GAUGE_SCL);
  gauge.init();
  gauge.setBatteryModel(MAX17055::LithiumPolymer);
  gauge.setCapacity(2500);
  gauge.setRSense(0.01);
  gauge.setVEmpty(3.3);
  gauge.setVRecovery(3.88);
  //pinMode(USR_LED, OUTPUT);

}

void loop() {
    Serial.printf("Capacity: %d\n", gauge.getCapacity());
    Serial.printf("Percentage: %f%\n", gauge.getStateOfCharge());
    Serial.printf("Cell Voltage: %fmV\n", gauge.getCellVoltage());
    Serial.printf("Current: %.2fmA\n", gauge.getCurrent());
    Serial.printf("Average current: %fmA\n", gauge.getTemperature());
    Serial.printf("Temperature: %f°C\n", gauge.getTemperature());
    Serial.printf("Fully charged: %s\n", gauge.isBatteryFullyCharged() ? "yes" : "no");
    Serial.printf("Data ready: %s\n", gauge.isDataReady() ? "yes" : "no");
    Serial.printf("Empty: %s\n", gauge.isBatteryEmpty() ? "yes" : "no");
    Serial.printf("Discharging: %s\n", gauge.isBatteryDischarging() ? "yes" : "no");
    //digitalWrite(USR_LED, HIGH);
    delay(500);
    //digitalWrite(USR_LED, LOW);
    delay(500);
}
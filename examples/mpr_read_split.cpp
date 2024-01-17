/*

This example shows read of Honeywell Pressure sensor MPR series I2C
It is split in start measurement and read result so the system is not blocked while measurement is taken.

*/

#include <Arduino.h>
#include <OneWire.h>
#include "DS28E18.h"

#define P_1W  27
#define READ_PERIODE_MS 1000
#define MPR_MEASUREMENT_TIME_MS 10

OneWire oneWire(P_1W);
uint32_t last_measurement = 0;
bool measurement_started = false;

DS28E18 ds28e18_sensors(&oneWire);

// some constants from Honeywell datasheet
const uint32_t outputmax = 15099494; // output at maximum pressure [counts]
const uint32_t outputmin = 1677722; // output at minimum pressure [counts]
const float pmax = 1.72369; // maximum value of pressure range bar
const float pmin = 0; // minimum value of pressure range bar

void setup() {
  Serial.begin(115200);
  // initialize bus
  if(ds28e18_sensors.begin()) {
    Serial.printf("%d DS28E18 found\r\n", ds28e18_sensors.getDeviceCount());
    for(uint8_t i = 0; i < ds28e18_sensors.getDeviceCount(); ++i) {
      Serial.printf("Sensor %d load sequence ", i);
      // load the Honeywell MPR sensor sequence
      if(ds28e18_sensors.MPR_sensor_init(i)) {
        Serial.println("ok");
      } else {
        Serial.println("failed");
      }
    }
  }
}

void loop() {
  uint32_t ti = millis();
  // initiate measurement
  if((ti - last_measurement) > READ_PERIODE_MS) {
    last_measurement = ti;
    Serial.println("---");
    for(uint8_t i = 0; i < ds28e18_sensors.getDeviceCount(); ++i) {
      if(ds28e18_sensors.MPR_sensor_measure(i)) {
        Serial.printf("Sensor %d: measurement start\r\n", i);
        measurement_started |= true;
      } else {
        Serial.printf("Sensor %d measurement start failed\r\n", i);
      }
    }
  }
  if(measurement_started && ((ti - last_measurement) > MPR_MEASUREMENT_TIME_MS)) {
    measurement_started = false;
    Serial.println("---");
    for(uint8_t i = 0; i < ds28e18_sensors.getDeviceCount(); ++i) {
      uint8_t status;
      uint32_t value;
      if(ds28e18_sensors.MPR_sensor_read_result(i)) {
        if(ds28e18_sensors.MPR_sensor_get_result(i, status, value)) {
          float pressure = ((value - outputmin) * (pmax - pmin)) / (outputmax - outputmin) + pmin;
          Serial.printf("Sensor %d: Status %02X, Value %0.5f Bar\r\n", i, status, pressure);
        } else {
          Serial.printf("Sensor %d get result failed\r\n", i);
        }
      } else {
        Serial.printf("Sensor %d read result failed\r\n", i);
      }
    }
  }
}


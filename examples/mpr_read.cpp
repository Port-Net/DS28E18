#include <Arduino.h>
#include <OneWire.h>
#include "DS28E18.h"

#define P_1W  27
#define READ_PERIODE_MS 1000
#define MPR_MEASUREMENT_TIME_MS 10

OneWire oneWire(P_1W);
uint32_t last_measurement = 0;

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
      if(ds28e18_sensors.load_MPR_sequencer(i)) {
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
      if(ds28e18_sensors.run_MPR_sequencer(i)) {
        uint8_t status;
        uint32_t value;
        if(ds28e18_sensors.read_MPR_result(i, status, value)) {
          float pressure = ((value - outputmin) * (pmax - pmin)) / (outputmax - outputmin) + pmin;
          Serial.printf("Sensor %d: Status %02X, Value %0.5f Bar\r\n", i, status, pressure);
        } else {
          Serial.printf("Sensor %d read failed\r\n", i);
        }
      } else {
        Serial.printf("Sensor %d sequencer failed\r\n", i);
      }
    }
  }
}

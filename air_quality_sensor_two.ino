#include <Wire.h>

#include <Arduino.h>
#include <NOxGasIndexAlgorithm.h>
#include <SensirionI2CSgp41.h>

#include <VOCGasIndexAlgorithm.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <SDS011.h>

SDS011 sds011_pms_sensor;
float pm10, pm25;

SensirionI2CSgp41 sgp41_voc_nox_sensor;
VOCGasIndexAlgorithm voc_algorithm;
NOxGasIndexAlgorithm nox_algorithm;
// time in seconds needed for NOx conditioning
uint16_t conditioning_s = 10;


#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme_temp_sensor; 
unsigned long delayTime_bme280;


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }

  Wire.begin();
  sgp41_voc_nox_sensor.begin(Wire);

  // delay(10000);  // needed on to ensure that the display is On before the code starts.

  int32_t index_offset;
  int32_t learning_time_offset_hours;
  int32_t learning_time_gain_hours;
  int32_t gating_max_duration_minutes;
  int32_t std_initial;
  int32_t gain_factor;
  voc_algorithm.get_tuning_parameters(
    index_offset, learning_time_offset_hours, learning_time_gain_hours,
    gating_max_duration_minutes, std_initial, gain_factor);

  Serial.println("\nVOC Gas Index Algorithm parameters");
  Serial.print("Index offset:\t");
  Serial.println(index_offset);
  Serial.print("Learning time offset hours:\t");
  Serial.println(learning_time_offset_hours);
  Serial.print("Learning time gain hours:\t");
  Serial.println(learning_time_gain_hours);
  Serial.print("Gating max duration minutes:\t");
  Serial.println(gating_max_duration_minutes);
  Serial.print("Std inital:\t");
  Serial.println(std_initial);
  Serial.print("Gain factor:\t");
  Serial.println(gain_factor);

  nox_algorithm.get_tuning_parameters(
    index_offset, learning_time_offset_hours, learning_time_gain_hours,
    gating_max_duration_minutes, std_initial, gain_factor);

  Serial.println("\nNOx Gas Index Algorithm parameters");
  Serial.print("Index offset:\t");
  Serial.println(index_offset);
  Serial.print("Learning time offset hours:\t");
  Serial.println(learning_time_offset_hours);
  Serial.print("Gating max duration minutes:\t");
  Serial.println(gating_max_duration_minutes);
  Serial.print("Gain factor:\t");
  Serial.println(gain_factor);
  Serial.println("");

  unsigned status;
    
  // default settings
  status = bme.begin(0x76);  
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }

  sds011_pms_sensor.begin(2, 3); //RX, TX
}

void loop() {
  float humidity = bme_temp_sensor.readHumidity();     // %RH
  float temperature = bme_temp_sensor.readTemperature();  // degreeC
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure = ");

  Serial.print(bme_temp_sensor.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme_temp_sensor.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println();

  uint16_t error;
  uint16_t srawVoc = 0;
  uint16_t srawNox = 0;
  uint16_t defaultCompenstaionRh = 0x8000;  // in ticks as defined by sgp41_voc_nox_sensor
  uint16_t defaultCompenstaionT = 0x6666;   // in ticks as defined by sgp41_voc_nox_sensor
  uint16_t compensationRh = 0;              // in ticks as defined by sgp41_voc_nox_sensor
  uint16_t compensationT = 0;               // in ticks as defined by sgp41_voc_nox_sensor

  // 1. Sleep: Measure every second (1Hz), as defined by the Gas Index
  // Algorithm
  //    prerequisite
  // 2. Measure temperature and humidity for SGP internal compensation
  // convert temperature and humidity to ticks as defined by sgp41_voc_nox_sensor
  // interface
  // NOTE: in case you read RH and T raw signals check out the
  // ticks specification in the datasheet, as they can be different for
  // different sensors
  
  compensationT = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
  compensationRh = static_cast<uint16_t>(humidity * 65535 / 100);

  // 3. Measure SGP4x signals
  if (conditioning_s > 0) {
    // During NOx conditioning (10s) SRAW NOx will remain 0
    error = sgp41_voc_nox_sensor.executeConditioning(compensationRh, compensationT, srawVoc);
    conditioning_s--;
  } else {
    error = sgp41_voc_nox_sensor.measureRawSignals(compensationRh, compensationT, srawVoc, srawNox);
  }

  // 4. Process raw signals by Gas Index Algorithm to get the VOC and NOx
  // index
  //    values
  int32_t voc_index;
  int32_t nox_index;
  if (error) {
    Serial.print("sgp41_voc_nox_sensor - Error trying to execute measureRawSignals(): ");
    char  errorMessage[256];
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    voc_index = voc_algorithm.process(srawVoc);
    nox_index = nox_algorithm.process(srawNox);
    Serial.print("VOC Index: ");
    Serial.print(voc_index);
    Serial.print("\t");
    Serial.print("NOx Index: ");
    Serial.println(nox_index);
  }

  error = sds011_pms_sensor.read(&pm25, &mp10);
	if (!error) {
		Serial.print("P2.5: " + String(pm25));
		Serial.println(" P10:  " + String(pm10));
	}
	delay(100);
  
}

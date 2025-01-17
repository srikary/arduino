#include <Wire.h>
#include <Arduino.h>

#include <SensirionI2CSgp41.h>
// #include <NOxGasIndexAlgorithm.h>
// #include <VOCGasIndexAlgorithm.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <SDS011.h>

#include <MQUnifiedsensor.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#define Voltage_Resolution 5
#define MQ135_PIN A1
#define MQ9_PIN (A2)
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
#define RatioMQ9CleanAir        (9.6) //RS / R0 = 60 ppm

#define TFT_CS        10
#define TFT_RST        8 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         9

//Declare Sensor
MQUnifiedsensor MQ135("Arduino", Voltage_Resolution, ADC_Bit_Resolution, MQ135_PIN, "MQ-135");
MQUnifiedsensor MQ9("Arduino", Voltage_Resolution, ADC_Bit_Resolution, MQ9_PIN, "MQ-9");

SDS011 sds011_pms_sensor;
float pm10, pm25;

SensirionI2CSgp41 sgp41_voc_nox_sensor;
// VOCGasIndexAlgorithm voc_algorithm;                                                                    
// NOxGasIndexAlgorithm nox_algorithm;
// time in seconds needed for NOx conditioning
uint16_t conditioning_s = 10;


#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme_temp_sensor; 
unsigned long delayTime_bme280;

uint16_t AQI = 999;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }

  Wire.begin();

  tft.initR(INITR_BLACKTAB); 
  tft.setRotation(3); 
  tft.invertDisplay(false);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.fillScreen(ST77XX_BLACK);

  sgp41_voc_nox_sensor.begin(Wire);

  // // delay(10000);  // needed on to ensure that the display is On before the code starts.

  // int32_t index_offset;
  // int32_t learning_time_offset_hours;
  // int32_t learning_time_gain_hours;
  // int32_t gating_max_duration_minutes;
  // int32_t std_initial;
  // int32_t gain_factor;
  // voc_algorithm.get_tuning_parameters(
  //   index_offset, learning_time_offset_hours, learning_time_gain_hours,
  //   gating_max_duration_minutes, std_initial, gain_factor);

  // Serial.println(F("\nVOC Gas Index Algorithm parameters"));
  // Serial.print(F("Index offset:\t"));
  // Serial.println(index_offset);
  // Serial.print(F("Learning time offset hours:\t"));
  // Serial.println(learning_time_offset_hours);
  // Serial.print(F("Learning time gain hours:\t"));
  // Serial.println(learning_time_gain_hours);
  // Serial.print(F("Gating max duration minutes:\t"));
  // Serial.println(gating_max_duration_minutes);
  // Serial.print(F("Std inital:\t"));
  // Serial.println(std_initial);
  // Serial.print(F("Gain factor:\t"));
  // Serial.println(gain_factor);

  // nox_algorithm.get_tuning_parameters(
  //   index_offset, learning_time_offset_hours, learning_time_gain_hours,
  //   gating_max_duration_minutes, std_initial, gain_factor);

  // Serial.println(F("\nNOx Gas Index Algorithm parameters"));
  // Serial.print(F("Index offset:\t"));
  // Serial.println(index_offset);
  // Serial.print(F("Learning time offset hours:\t"));
  // Serial.println(learning_time_offset_hours);
  // Serial.print(F("Gating max duration minutes:\t"));
  // Serial.println(gating_max_duration_minutes);
  // Serial.print(F("Gain factor:\t"));
  // Serial.println(gain_factor);
  // Serial.println(F(""));

  unsigned status;
    
  // default settings
  status = bme_temp_sensor.begin(0x76);  
  if (!status) {
      Serial.println(F("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
      Serial.print(F("SensorID was: 0x")); Serial.println(bme_temp_sensor.sensorID(),16);
      Serial.print(F("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n"));
      Serial.print(F("   ID of 0x56-0x58 represents a BMP 280,\n"));
      Serial.print(F("        ID of 0x60 represents a BME 280.\n"));
      Serial.print(F("        ID of 0x61 represents a BME 680.\n"));
      while (1) delay(10);
  }

  sds011_pms_sensor.begin(2, 3); //RX, TX

  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(102.2); MQ135.setB(-2.473); // Configure the equation to to calculate NH4 concentration
  /*
    Exponential regression:
    GAS      | a      | b
    CO       | 605.18 | -3.937  
    Alcohol  | 77.255 | -3.18 
    CO2      | 110.47 | -2.862
    Toluen  | 44.947 | -3.445
    NH4      | 102.2  | -2.473
    Aceton  | 34.668 | -3.369
  */
  
  MQ135.init(); 
  Serial.println(F("Calibrating MQ135 please wait"));
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  }
  MQ135.setR0(calcR0/10);
  Serial.println(F("Done Calibrating MQ135"));
  if(isinf(calcR0)) {Serial.println(F("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply")); while(1);}
  if(calcR0 == 0){Serial.println(F("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply")); while(1);}
  
  MQ9.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ9.setA(1000.5); MQ9.setB(-2.186); // Configure the equation to to calculate LPG concentration
  /*
    Exponential regression:
    GAS     | a      | b
    LPG     | 1000.5 | -2.186
    CH4     | 4269.6 | -2.648
    CO      | 599.65 | -2.244
  */
  MQ9.init(); 
  Serial.println(F("Calibrating MQ9 please wait"));
  // delay(90000);  // TODO: Maybe uncomment this for the heater to work.
  calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ9.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ9.calibrate(RatioMQ9CleanAir);
  }
  MQ9.setR0(calcR0/10);
  Serial.println(F("Done Calibrating MQ9"));
  if(isinf(calcR0)) {Serial.println(F("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply")); while(1);}
  if(calcR0 == 0){Serial.println(F("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply")); while(1);}
   
  
}

void loop() {
  Serial.println(F("Starting loop"));
  float humidity = bme_temp_sensor.readHumidity();     // %RH
  float temperature = bme_temp_sensor.readTemperature();  // degreeC
  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(F(" °C"));
  Serial.print(F("Pressure = "));
  Serial.print(bme_temp_sensor.readPressure() / 100.0F);
  Serial.println(F(" hPa"));
  Serial.print(F("Approx. Altitude = "));
  Serial.print(bme_temp_sensor.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(F(" m"));
  Serial.print(F("Humidity = "));
  Serial.print(humidity);
  Serial.println(F(" %"));

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

 if (!error) {
    Serial.print(F("VOC : "));
    Serial.print(srawVoc);
    Serial.print(F("\t"));
    Serial.print(F("NOx : "));
    Serial.println(srawNox);
 }

  // // 4. Process raw signals by Gas Index Algorithm to get the VOC and NOx
  // // index
  // //    values
  // int32_t voc_index;
  // int32_t nox_index;
  // if (error) {
  //   Serial.print(F("sgp41_voc_nox_sensor - Error trying to execute measureRawSignals(): "));
  //   char  errorMessage[256];
  //   errorToString(error, errorMessage, 256);
  //   Serial.println(errorMessage);
  // } else {
  //   voc_index = voc_algorithm.process(srawVoc);
  //   nox_index = nox_algorithm.process(srawNox);
  //   Serial.print(F("VOC Index: "));
  //   Serial.print(voc_index);
  //   Serial.print(F("\t"));
  //   Serial.print(F("NOx Index: "));
  //   Serial.println(nox_index);
  // }

  error = sds011_pms_sensor.read(&pm25, &pm10);
  AQI = 999;
	if (!error) {
		Serial.print(F("P2.5: "));
    Serial.print(pm25);
    AQI = pm25_aqi_us(pm25);
		Serial.print(F(" P10:  "));
    Serial.println(pm10);
	}

  MQ9.update(); // Update data, the arduino will read the voltage from the analog pin
  float lpg_index = MQ9.readSensor();
  Serial.print(F("LPG :"));
  Serial.println(lpg_index);

  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  float smoke_index = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  Serial.print(F("Smoke: "));
  Serial.println(smoke_index);

  tft.setCursor(0, 0);
  
  setTextColorForParameter(temperature, 40);
  tft.print(F(" "));
  tft.print(temperature);
  tft.print(F("C |"));

  tft.print(F(" ")); 
  tft.print(humidity);
  tft.println(F("% "));

  tft.print(" Press: ");
  tft.println(bme_temp_sensor.readPressure() / 100.0F);
  tft.print("Alt: ");
  tft.println(bme_temp_sensor.readAltitude(SEALEVELPRESSURE_HPA));

  setTextColorForAQI(AQI);
  tft.print(F(" AQI US : "));
  tft.print(AQI);
  tft.println(F("            "));

  setTextColorForParameter(pm25, 40);
  tft.print(F(" PM 2.5 : "));
  tft.print(pm25);
  tft.println(F(" ug/m3       "));

  setTextColorForParameter(pm10, 200);
  tft.print(F(" PM  10 : "));
  tft.print(pm10);
  tft.println(F(" ug/m3       "));

  setTextColorForParameter(srawVoc, 2);
  tft.print(F(" VOC : "));
  tft.print(srawVoc);
  tft.print(F(" NOx : "));
  tft.println(srawNox);

  setTextColorForParameter(lpg_index, 7);
  tft.print(F(" LPG : "));
  tft.print(lpg_index);
  tft.print(F(" Smoke : "));
  tft.println(smoke_index);

	delay(1000);
}

float linear(uint16_t aqi_high, uint16_t aqi_low,
                               float conc_high, float conc_low,
                               float concentration) {
  float f;
  f = ((concentration - conc_low) / (conc_high - conc_low)) *
          (aqi_high - aqi_low) +
      aqi_low;
  return f;
}

uint16_t pm25_aqi_us(float concentration) {
  float c;
  float AQI;
  c = (floor(10 * concentration)) / 10;
  if (c < 0)
    AQI = 0;
  else if (c >= 0 && c < 12.1f) {
    AQI = linear(50, 0, 12, 0, c);
  } else if (c >= 12.1f && c < 35.5f) {
    AQI = linear(100, 51, 35.4f, 12.1f, c);
  } else if (c >= 35.5f && c < 55.5f) {
    AQI = linear(150, 101, 55.4f, 35.5f, c);
  } else if (c >= 55.5f && c < 150.5f) {
    AQI = linear(200, 151, 150.4f, 55.5f, c);
  } else if (c >= 150.5f && c < 250.5f) {
    AQI = linear(300, 201, 250.4f, 150.5f, c);
  } else if (c >= 250.5f && c < 350.5f) {
    AQI = linear(400, 301, 350.4f, 250.5f, c);
  } else if (c >= 350.5f && c < 500.5f) {
    AQI = linear(500, 401, 500.4f, 350.5f, c);
  } else {
    AQI = 99999; //
  }
  return round(AQI);
}

void setTextColorForAQI(uint16_t aqi) {
  if (aqi >=0 && aqi <= 50) {
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  } else if (aqi > 50 && aqi <= 100) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  } else if (aqi > 100 && aqi <= 150) {
    tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  } else if (aqi > 150 && aqi <= 200) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  } else if (aqi > 200 && aqi <= 300) {
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  } else if (aqi > 300) {
    tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  }
}

void setTextColorForParameter(float value, float threshold) {
  if (value > threshold) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  } else {
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  }
}

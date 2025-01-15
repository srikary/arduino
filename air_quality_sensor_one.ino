#include <Arduino.h>
#include "MHZ19.h"               
#include "WZ.h"
#include <SoftwareSerial.h>       

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include <DFRobot_DHT11.h>

#include <AGS10.h>
#include <Wire.h>
#include <PMS.h>
#include <MQUnifiedsensor.h>

// Connections
// PMS5003 PM2.5 Sensor
// https://github.com/fu-hsi/PMS/blob/master/src/PMS.cpp
// https://www.aliexpress.com/item/1005005967735332.html
// https://cdn-shop.adafruit.com/product-files/3686/plantower-pms5003-manual_v2-3.pdf
// VCC -> 5v, GND -> GND, TX (Grey) -> 2, RX (Yellow) -> 3 (needs voltage divider 3.3v)

// MHZ19B CO2 Sensor
// https://www.aliexpress.com/item/1005006394895362.html
// https://github.com/WifWaf/MH-Z19/blob/master/examples/BasicUsage/BasicUsage.ino
// VCC -> 5v, GND -> GND, TX (Purple) -> 4, RX (Grey) -> 5

// ZE08-HCHO
// https://github.com/leonlucc/WZ/blob/master/examples/arduino-nano-wz-active-mode/arduino-nano-wz-active-mode.ino
// VCC -> 5v, GND -> GND, TX (Yellow) -> 6, RX (Orange) -> 7 (through voltage divider 3.3v orange + white)

// Sensiron SGP41 - TVOC
// https://sensirion.com/products/catalog/SGP41
// https://github.com/Sensirion/arduino-gas-index-algorithm/blob/master/src/SensirionGasIndexAlgorithm.h
// https://github.com/Sensirion/arduino-i2c-sgp41/tree/master/src
// // VCC (Yellow) -> 5v, GND (Orange) -> GND, SDA (Brown) -> A4, SCL (Red) -> A5

// AGS10 - TVOC
// https://github.com/gina-seth/AGS10_sensor/blob/main/examples/AGS10_readTVOC_basic/AGS10_readTVOC_basic.ino
// VCC -> 5v, GND, SDA, SCL

// DHT11 - Temp Humidity
// https://github.com/DFRobot/DFRobot_DHT11
// + -> 5v, - -> GND, out -> A0

// MQ Sensors
// https://github.com/miguel5612/MQSensorsLib


AGS10 ags10_sensor = AGS10();

DFRobot_DHT11 DHT;
#define DHT11_PIN A0

// For the breakout board, you can use any 2 or 3 pins.
// These pins will also work for the 1.8" TFT shield.
#define TFT_CS        10
#define TFT_RST        8 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         9

#define         MQ2_PIN                 (A1)
#define         MQ7_PIN                 (A2)
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (10) // For arduino UNO/MEGA/NANO
#define         RatioMQ2CleanAir        (9.83) //RS / R0 = 9.83 ppm 
#define RatioMQ7CleanAir 27.5 //RS / R0 = 27.5 ppm 

MQUnifiedsensor MQ2("Arduino", Voltage_Resolution, ADC_Bit_Resolution, MQ2_PIN, "MQ-2");
MQUnifiedsensor MQ7("Arduino", Voltage_Resolution, ADC_Bit_Resolution, MQ7_PIN, "MQ-7");

unsigned long oldTime = 0;

unsigned long mhz19b_co2_data_timer = 0;

SoftwareSerial pms5003_serial(2, 3); // RX, TX
SoftwareSerial mhz19_serial(4, 5);
SoftwareSerial ze08_serial(6,7);
WZ wz_ze08_sensor(ze08_serial);
MHZ19 mhz19b;


PMS pms_5003_sensor(pms5003_serial);
PMS::DATA pms_5003_data;

WZ::DATA hcho_data;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(9600);
  pms5003_serial.begin(9600);
  pms_5003_sensor.passiveMode();    // Switch to passive mode
  Serial.println(F("Waking up, wait 30 seconds for stable readings"));
  pms_5003_sensor.wakeUp();
  mhz19_serial.begin(9600);                               // (Uno example) device to MH-Z19 serial start   
  mhz19b.begin(mhz19_serial);                                // *Serial(Stream) refence must be passed to library begin(). 
  mhz19b.autoCalibration(true); 
  ze08_serial.begin(9600);
  // wz.passiveMode();
  wz_ze08_sensor.activeMode();
  ags10_sensor.begin();
  

  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(574.25); MQ2.setB(-2.222); // Configure the equation to to calculate LPG concentration
  /*
    Exponential regression:
    Gas    | a      | b
    H2     | 987.99 | -2.162
    LPG    | 574.25 | -2.222
    CO     | 36974  | -3.109
    Alcohol| 3616.1 | -2.675
    Propane| 658.71 | -2.168
  */

  // /*****************************  MQ Init ********************************************/ 
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/ 
  MQ2.init(); 
  Serial.print(F("Calibrating MQ2 please wait. Only execute this in Clean Air Conditions"));
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
  }
  MQ2.setR0(calcR0/10);
  Serial.println(F("Done Calibrating MQ2"));
  
  if(isinf(calcR0)) {Serial.println(F("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply")); while(1);}
  if(calcR0 == 0){Serial.println(F("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply")); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 

  MQ7.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ7.setA(99.042); MQ7.setB(-1.518); // Configure the equation to calculate CO concentration value

  /*
    Exponential regression:
  GAS     | a      | b
  H2      | 69.014  | -1.374
  LPG     | 700000000 | -7.703
  CH4     | 60000000000000 | -10.54
  CO      | 99.042 | -1.518
  Alcohol | 40000000000000000 | -12.35
  */
  
  MQ7.init(); 

  Serial.print(F("Calibrating MQ7. Please wait"));
  calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ7.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
  }
  MQ7.setR0(calcR0/10);
  Serial.println(F("Done Calibrating MQ7"));
  
  if(isinf(calcR0)) {Serial.println(F("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply")); while(1);}
  if(calcR0 == 0){Serial.println(F("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply")); while(1);}


  tft.init(240, 320);   
}
 
     
void loop() {
  Serial.println(F("Starting loop"));
  pms5003_serial.listen();
  pms_5003_sensor.requestRead();
  if (pms_5003_sensor.readUntil(pms_5003_data, 10000)) {
    // reading data was successful!
    Serial.print(F("PM 1.0 (ug/m3): "));
    Serial.println(pms_5003_data.PM_AE_UG_1_0);

    Serial.print(F("PM 2.5 (ug/m3): "));
    Serial.println(pms_5003_data.PM_AE_UG_2_5);

    Serial.print(F("PM 10.0 (ug/m3): "));
    Serial.println(pms_5003_data.PM_AE_UG_10_0);
    Serial.println(F("---------------------------------------"));
  }

  mhz19_serial.listen();
  int CO2; 
  int8_t Temp;
  if (millis() - mhz19b_co2_data_timer >= 2000) {
      /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
      if below background CO2 levels or above range (useful to validate sensor). You can use the 
      usual documented command with getCO2(false) */

      CO2 = mhz19b.getCO2(false, true);                             // Request CO2 (as ppm)
      
      Serial.print(F("CO2 (ppm): "));                      
      Serial.println(CO2);                                

      
      Temp = mhz19b.getTemperature(true);                     // Request Temperature (as Celsius)
      Serial.print(F("Temperature [from MHZ19B] (C): "));                  
      Serial.println(Temp);                               

      mhz19b_co2_data_timer = millis();
  }
    
  ze08_serial.listen();
  wz_ze08_sensor.requestRead();
  if (wz_ze08_sensor.readUntil(hcho_data))
  {
      Serial.println(F("Sending command"));
      Serial.print(F("HCHO (ppd): "));
      Serial.println(hcho_data.HCHO_PPB);
      Serial.print(F("HCHO (ug/m3): "));
      Serial.println(hcho_data.HCHO_UGM3); 
  }

  // if (wz_ze08_sensor.read(hcho_data))
  // {
  //     Serial.print(F("HCHO (ppd): "));
  //     Serial.println(hcho_data.HCHO_PPB);
  //     Serial.print(F("HCHO (ug/m3): "));
  //     Serial.println(hcho_data.HCHO_UGM3); // no data here, 0 returned
  //     Serial.println();
  // } else {
  //   Serial.println(F("NO HCHO data"));
  // }

  DHT.read(DHT11_PIN);
  Serial.print(F("temp:"));
  Serial.print(DHT.temperature);
  Serial.print(F("  humi:"));
  Serial.println(DHT.humidity);

  Serial.print(F("TVOC "));
  Serial.print(ags10_sensor.readTVOC());
  Serial.println(F(" ppm"));


  MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
  float lpg_percentage = MQ2.readSensor();
  Serial.print(F("LPG: "));
  Serial.print(lpg_percentage);
  Serial.println(F("ppm")); 
  
  MQ7.update(); // Update data, the arduino will read the voltage from the analog pin 
  float co_percentage = MQ7.readSensor();
  Serial.print(F("CO: "));
  Serial.print(co_percentage);
  Serial.println(F("ppm"));
  

  Serial.println(F("Printing to TFT"));
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextWrap(false);
  
  
  // tft.setTextColor(ST77XX_RED);
  // tft.setTextColor(ST77XX_YELLOW);
  // tft.setTextColor(ST77XX_GREEN);
  tft.setTextColor(ST77XX_BLUE);
  // tft.setTextColor(ST77XX_WHITE);
  // tft.setTextColor(ST77XX_MAGENTA);
  tft.setTextSize(3);

  tft.print(F("PM 1.0: "));
  tft.println(pms_5003_data.PM_AE_UG_1_0);
  tft.print(F("PM 2.5: "));
  tft.println(pms_5003_data.PM_AE_UG_2_5);
  tft.print(F("PM  10: "));
  tft.println(pms_5003_data.PM_AE_UG_10_0);
  tft.print(F("CO2   : "));
  tft.println(CO2);
  tft.print(F("Temp  : "));
  tft.print(DHT.temperature);
  tft.println(F(" C"));
  tft.print(F("Hum   : ")); 
  tft.print(DHT.humidity);
  tft.println(F(" %"));
  tft.print(F("TVOC  :"));
  tft.print(ags10_sensor.readTVOC());
  tft.println(F("ppd"));
  tft.print(F("HCHO  :"));
  tft.print(hcho_data.HCHO_PPB);
  tft.println(F("ppd"));
  tft.print(F("HCHO  :"));
  tft.print(hcho_data.HCHO_UGM3);
  tft.println(F("g/m3"));
  
  tft.print(F("LPG   :"));
  tft.println(lpg_percentage);
  tft.print(F("CO    :"));
  tft.println(co_percentage);
  delay(1000);
}

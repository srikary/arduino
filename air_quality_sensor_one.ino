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

AGS10 ags10_sensor = AGS10();

DFRobot_DHT11 DHT;
#define DHT11_PIN A0


// For the breakout board, you can use any 2 or 3 pins.
// These pins will also work for the 1.8" TFT shield.
#define TFT_CS        10
#define TFT_RST        8 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         9
               
#define         MQ2_PIN                       (A1)     //define which   analog input channel you are going to use
#define         MQ7_PIN                       (A2)     //define which   analog input channel you are going to use
#define         RL_VALUE                     (5)      //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR           (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                      //which is derived from the   chart in datasheet
 
/**********************Software Related Macros***********************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are   going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL   (500)   //define the time interal(in milisecond) between each samples in the
                                                      //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are   going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)      //define the time interal(in milisecond) between each samples in 
 
/*********************Application Related Macros*********************************/
#define          GAS_LPG                      (0)
#define          GAS_CO                       (1)
#define          GAS_SMOKE                    (2)
 
/****************************Globals**********************************************/
float            LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve.   
                                                    //with these two points,   a line is formed which is "approximately equivalent"
                                                    //to   the original curve. 
                                                    //data   format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float            COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve.   
                                                    //with these two points,   a line is formed which is "approximately equivalent" 
                                                    //to   the original curve.
                                                    //data   format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float            SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve.   
                                                    //with these two points,   a line is formed which is "approximately equivalent" 
                                                    //to   the original curve.
                                                    //data   format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float            Ro_mq2           =  10;                 //Ro is initialized to 10 kilo ohms
float            Ro_mq7           =  10;                 //Ro is initialized to 10 kilo ohms

// Connections
// PMS5003 PM2.5 Sensor
// https://how2electronics.com/interfacing-pms5003-air-quality-sensor-arduino/#google_vignette
// https://www.aliexpress.com/item/1005005967735332.html
// VCC -> 5v, GND -> GND, TX (Grey) -> 2, RX (Yellow) -> Unconnected (needs voltage divider 3.3v)

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
// + -> 5v, - -> GND, out -> A0

unsigned long getDataTimer = 0;

SoftwareSerial pms5003_serial(2, 3); // RX, TX
SoftwareSerial mhz19_serial(4, 5);
SoftwareSerial ze08_serial(6,7);
WZ wz_ze08_sensor(ze08_serial);
MHZ19 mhz19b;

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
  
struct pms5003data data;
WZ::DATA hcho_data;

// SensirionI2CSgp41 sgp41;

// VOCGasIndexAlgorithm voc_algorithm;
// NOxGasIndexAlgorithm nox_algorithm;

// // time in seconds needed for NOx conditioning
// uint16_t conditioning_s = 10;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void setup() {

  Serial.begin(9600);
  
  pms5003_serial.begin(9600);
  mhz19_serial.begin(9600);                               // (Uno example) device to MH-Z19 serial start   
  mhz19b.begin(mhz19_serial);                                // *Serial(Stream) refence must be passed to library begin(). 
  mhz19b.autoCalibration(true); 
  ze08_serial.begin(9600);
  // wz.passiveMode();
  wz_ze08_sensor.activeMode();

  ags10_sensor.begin();

  Ro_mq2 = MQCalibration(MQ2_PIN);
  Ro_mq7 = MQCalibration(MQ7_PIN);
  tft.init(240, 320);   
}
 

    
void loop() {
  pms5003_serial.listen();
  if (readPMSdata(&pms5003_serial)) {
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");
  }

  mhz19_serial.listen();
  int CO2; 
  int8_t Temp;
  if (millis() - getDataTimer >= 2000) {
      /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
      if below background CO2 levels or above range (useful to validate sensor). You can use the 
      usual documented command with getCO2(false) */

      CO2 = mhz19b.getCO2(false, true);                             // Request CO2 (as ppm)
      
      Serial.print("CO2 (ppm): ");                      
      Serial.println(CO2);                                

      
      Temp = mhz19b.getTemperature(true);                     // Request Temperature (as Celsius)
      Serial.print("Temperature [from MHZ19B] (C): ");                  
      Serial.println(Temp);                               

      getDataTimer = millis();
  }
    
  ze08_serial.listen();
  wz_ze08_sensor.requestRead();
  if (wz_ze08_sensor.readUntil(hcho_data))
  {
      Serial.println("Sending command");
      Serial.print("HCHO (ppd): ");
      Serial.println(hcho_data.HCHO_PPB);
      Serial.print("HCHO (ug/m3): ");
      Serial.println(hcho_data.HCHO_UGM3); 
  }

  // if (wz_ze08_sensor.read(hcho_data))
  // {
  //     Serial.print("HCHO (ppd): ");
  //     Serial.println(hcho_data.HCHO_PPB);
  //     Serial.print("HCHO (ug/m3): ");
  //     Serial.println(hcho_data.HCHO_UGM3); // no data here, 0 returned
  //     Serial.println();
  // } else {
  //   Serial.println("NO HCHO data");
  // }

  DHT.read(DHT11_PIN);
  Serial.print("temp:");
  Serial.print(DHT.temperature);
  Serial.print("  humi:");
  Serial.println(DHT.humidity);

  Serial.print("TVOC ");
  Serial.print(ags10_sensor.readTVOC());
  Serial.println(" ppm");

  int lpg_percentage = MQGetGasPercentage(MQRead(MQ2_PIN)/Ro_mq2, GAS_LPG);
  Serial.print("LPG: ");
  Serial.print(lpg_percentage);
  Serial.println("ppm"); 
  
  int co_percentage = MQGetGasPercentage(MQRead(MQ7_PIN)/Ro_mq7, GAS_CO);
  Serial.print("CO: ");
  Serial.print(co_percentage);
  Serial.println("ppm");
  

  Serial.println("Printing to TFT");
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

  tft.print("PM 1.0 : ");
  tft.println(data.pm10_standard);
  tft.print("PM 2.5 : ");
  tft.println(data.pm25_standard);
  tft.print("PM  10 : ");
  tft.println(data.pm100_standard);
  tft.print("CO2    : ");
  tft.println(CO2);
  tft.print("Temp   : ");
  tft.print(DHT.temperature);
  tft.println(" C");
  tft.print("Hum    : "); 
  tft.print(DHT.humidity);
  tft.println(" %");
  tft.print("TVOC   :");
  tft.print(ags10_sensor.readTVOC());
  tft.println(" ppd");
  tft.print("HCHO   : ");
  tft.print(hcho_data.HCHO_PPB);
  tft.println(" ppd");
  tft.print("HCHO   : ");
  tft.print(hcho_data.HCHO_UGM3);
  tft.println(" g/m3");
  
  tft.print("LPG    : ");
  tft.println(lpg_percentage);
  tft.print("CO     : ");
  tft.println(co_percentage);
  delay(1000);
}
 
boolean readPMSdata(Stream *s) {
  while (! s->available()) {
    delay(100);
    // return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
  // while (s->peek() != 0x42) {
  //   s->read();
  // }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}
 
/****************   MQResistanceCalculation **************************************
Input:   raw_adc   - raw value read from adc, which represents the voltage
Output:  the calculated   sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider.   Given the voltage
         across the load resistor and its resistance, the resistance   of the sensor
         could be derived.
**********************************************************************************/   
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
   
/*************************** MQCalibration **************************************
Input:    mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function   assumes that the sensor is in clean air. It use  
         MQResistanceCalculation   to calculates the sensor resistance in clean air 
         and then divides it   with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs   slightly between different sensors.
**********************************************************************************/   
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
   for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
     val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
   }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average   value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided   by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according   to the chart in the datasheet 
 
  return val; 
}
/***************************   MQRead *******************************************
Input:   mq_pin - analog   channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation   to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor   is in the different consentration of the target
         gas. The sample times   and the time interval between samples could be configured
         by changing   the definition of the macros.
**********************************************************************************/   
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++)   {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
   }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/***************************   MQGetGasPercentage ********************************
Input:   rs_ro_ratio -   Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the   target gas
Remarks: This function passes different curves to the MQGetPercentage   function which 
         calculates the ppm (parts per million) of the target   gas.
**********************************************************************************/   
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id   == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else   if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
   } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
   }    
 
  return 0;
}
 
/***************************  MQGetPercentage   ********************************
Input:   rs_ro_ratio - Rs divided by Ro
          pcurve      - pointer to the curve of the target gas
Output:  ppm of   the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic   value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided.   As it is a 
         logarithmic coordinate, power of 10 is used to convert the   result to non-logarithmic 
         value.
**********************************************************************************/   
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,(((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}


#include <Time.h>
#include <OneWire.h> 
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Servo.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

// BEGIN LCD Config Params
#define LCD_I2C_ADDR    0x27 // <<----- Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
// END LCD Config Params.
LiquidCrystal_I2C	lcd(LCD_I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

#define RELAY1_PIN  4   // Positive Pump Terminal
#define RELAY2_PIN  5   // Negative Pump Terminal 
#define RELAY3_PIN  6   // Pump Power ON/OFF
#define RELAY4_PIN  7   // Stove Power ON/OFF
#define STRAINER_SERVO_PIN 8

#define SET_ALARM_PIN 9
#define TEA_NOW_VS_ALARM_MODE_PIN 10
#define CLEANER_HOUR_MODE_PIN 11
#define EGG_MODE_MINUTE_PIN 12
#define TIME_INTERRUPT_PIN 2
#define STATE_INTERRUPT_PIN 3


#define DAY 1
#define MONTH 1
#define YEAR 2014
#define INTERRUPT_BUTTON_DELAY 16000

#define ULONG_MAX ((1 << 31) - 1)

#define DIR_UP -1
#define DIR_DOWN 1

#define DEBUG true

Servo strainer_servo;

#define DS18S20_PIN 10 
//Temperature chip i/o
OneWire ds(DS18S20_PIN); 

enum State {
  SLEEP = 0,
  START_TEA= 1,
  DISPENSE_MILK = 2,
  CONTROL_STRAINER = 3,
  CHECK_TEMP = 4,
  START_STEEPING = 5,
  CLEAN = 6,
  BOIL_EGG = 7,
  REFRESH_LCD = 8,
  DONE = 9
};

static const int num_states = 10;
const State kStateState = SLEEP;
const State kEndState = DONE;

volatile bool active_states[num_states];
volatile unsigned long loop_delay;
// Button debounce
long time_lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 100;   
long state_lastDebounceTime = 0;  // the last time the output pin was toggled

struct SleepData {
  volatile int wakeup_hour;
  volatile int wakeup_minute;
};
volatile SleepData sleep_data;

struct StrainerData {
  static const int high_pos = 0;
  static const int low_pos = 180;
  static const int mid_pos = 100;
  static const unsigned long steep_duration_before_stirring = 60000;
  static const unsigned long steep_duration = 180000;
  static const unsigned long move_delay = 10;
  static const int dx = 1;
  int current_pos;
  int sign;
  bool is_lowered;
  unsigned long lowered_millis;
};
StrainerData strainer_data;

void resetStrainerData() {
  strainer_data.current_pos = strainer_data.high_pos;
  strainer_data.sign = DIR_DOWN;
  strainer_data.is_lowered = false;
}

struct StoveData {
  static const float boiling_point_upper = 73.80f; //100.0f;
  static const float boiling_point_lower = 67.0f; // 98.0f;
  static const unsigned long max_on_duration = 1800000;  // 30 Minutes

  bool is_on;
  bool should_keep_on;
  unsigned long start_millis;
  bool boiling_point_reached;
  unsigned long boiling_point_reached_millis;
  unsigned long stop_millis;
  float temperature;
};
StoveData stove_data;

void resetStoveData() {
  turnStoveOff();
  stove_data.boiling_point_reached = false;
  stove_data.should_keep_on = false;
}

struct MilkPumpData {
  static const unsigned long duration_per_cup = 1000; // TODO 600000;
  static const unsigned long forward_prime_duration = 1000; // 50000;
  static const unsigned long reverse_prime_duration = 1000; // 40000;
  volatile int num_cups;
  bool is_on;
  bool is_forward;
  bool forward_done;
  bool reverse_done;
  unsigned long forward_start_time;
  unsigned long reverse_start_time;
  unsigned long pump_end_time;
  unsigned long next_check_time;
};
MilkPumpData milk_pump_data;

struct EggData {
  static const unsigned long boil_time_base = 600000;
  static const unsigned long boil_time_per_egg = 60000;
  static const int num_eggs = 1;
  unsigned long start;
};
EggData egg_data; 

struct CleanerData {
  static const unsigned long forward_run_time = 60000;
  static const unsigned long reverse_run_time = 5000;
  static const int num_cycles = 3;
};
CleanerData cleaner_data; 

void resetMilkPumpData() {
  stopPump();
  milk_pump_data.is_on = false;
  milk_pump_data.is_forward = false;
  milk_pump_data.forward_done = false;
  milk_pump_data.reverse_done = false;
  milk_pump_data.forward_start_time = ULONG_MAX;
  milk_pump_data.reverse_start_time = ULONG_MAX;
  milk_pump_data.pump_end_time = ULONG_MAX;
  milk_pump_data.next_check_time = ULONG_MAX;
}

void setDelay(unsigned long newdelay) {
  loop_delay = min (loop_delay, newdelay);
}

void turnStoveOn() {
  digitalWrite(RELAY4_PIN, LOW);
  stove_data.is_on = true;
  if (DEBUG) {
    Serial.println("Stove ON");
  }
}

void turnStoveOff() {
  digitalWrite(RELAY4_PIN, HIGH);
  stove_data.is_on = false;
  if (DEBUG) {
    Serial.println("Stove OFF");
  }
}

void runPumpForward() {
  digitalWrite(RELAY1_PIN, LOW); // Runs Pump in forward direction
  digitalWrite(RELAY2_PIN, LOW); 
  digitalWrite(RELAY3_PIN, LOW); 
  if (DEBUG) {
    Serial.println("Pump F");
  }
}

void runPumpBackward() {
  digitalWrite(RELAY1_PIN, HIGH);  // Runs Pump in reverse direction.
  digitalWrite(RELAY2_PIN, HIGH); 
  digitalWrite(RELAY3_PIN, LOW); 
  if (DEBUG) {
    Serial.println("Pump R");
  }
}

void stopPump() {
  digitalWrite(RELAY3_PIN, HIGH);
  if (DEBUG) {
    Serial.println("Pump Off"); 
  }
}

void initButtonPin(int pin_num) {
  pinMode(pin_num, INPUT);
  digitalWrite(pin_num, HIGH);
}

bool isButtonPressed(int pin_num) {
  int buttonState = digitalRead(pin_num);
  if (DEBUG && (buttonState == LOW)) {
    Serial.print("Button Pressed");
    Serial.println(pin_num, DEC);
  }
  return buttonState == LOW;
}

void Init() {
  stopPump();
  turnStoveOff();
  for (int i = 0; i < num_states; ++i) {
    active_states[i] = false;
  }
  active_states[REFRESH_LCD] = true;
  active_states[SLEEP] = true;
}

void setup()
{    
  Serial.begin(9600);
  // Initialise the Arduino relay pins for OUTPUT
  pinMode(RELAY1_PIN, OUTPUT);       
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  initButtonPin(SET_ALARM_PIN);
  initButtonPin(TEA_NOW_VS_ALARM_MODE_PIN);
  initButtonPin(CLEANER_HOUR_MODE_PIN);
  initButtonPin(EGG_MODE_MINUTE_PIN);
  pinMode(TIME_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(STATE_INTERRUPT_PIN, INPUT_PULLUP);


  mlx.begin(); 
  strainer_servo.attach(STRAINER_SERVO_PIN);  // attaches the servo on pin 9 to the servo object  
  strainer_servo.write(strainer_data.high_pos); 
  strainer_servo.detach();

  milk_pump_data.num_cups = 1;
  setTime(6, 8, 55, DAY, MONTH, YEAR);
  sleep_data.wakeup_hour = 6;
  sleep_data.wakeup_minute = 10;

  // LCD Initialization
  lcd.begin (16,2); //  <<----- My LCD was 16x2
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(LOW);
  lcd.home (); // go home
  Init();
  lcd.clear(); // For some reason, this doesnt seem to work in an ISR.
  //Interrupts
  attachInterrupt(0, timeInterrupt, FALLING); // Connect the set time button to pin 2 on Uno and 3 on leonardo
  attachInterrupt(1, stateInterrupt, FALLING);  // Connect the set state button to pin 3 on Uno and 2 on leonardo
}

void loop()
{
  loop_delay = 1000;
  //loop_delay = 600000;
  for (int state_int = kStateState; state_int <= kEndState; ++state_int) {
    State state = static_cast<State>(state_int);
    if (!active_states[state]) {
      continue;
    }
    switch (state) {
    case SLEEP:
      if (DEBUG) {
        Serial.println("SLEEP");
      }
      if (shouldIWakeUp()) {
        active_states[SLEEP] = false;
        active_states[START_TEA] = true;
      }
      break;
    case START_TEA:
      if (DEBUG) {
        Serial.println("START_TEA");
      }
      resetMilkPumpData();
      resetStrainerData();
      resetStoveData();
      strainer_servo.attach(STRAINER_SERVO_PIN);
      strainer_data.current_pos = strainer_data.high_pos;
      strainer_servo.write(strainer_data.high_pos);
      turnStoveOn();
      stove_data.should_keep_on = true;
      stove_data.start_millis = millis();
      active_states[START_TEA] = false;
      active_states[DISPENSE_MILK] = true;
      active_states[CHECK_TEMP] = true;
      active_states[CONTROL_STRAINER] = true;
      lcd.clear();
      lcd.setBacklight(HIGH); 
      break;
    case DISPENSE_MILK:
      if (DEBUG) {
        Serial.println("DISPENSE_MILK");
      }
      if (milk_pump_data.forward_done && milk_pump_data.reverse_done) {
        active_states[DISPENSE_MILK] = false;
      } 
      else {
        if (milk_pump_data.next_check_time == ULONG_MAX) { // Start pump
          runPumpForward();
          milk_pump_data.is_on = true;
          milk_pump_data.is_forward = true;
          milk_pump_data.forward_start_time = millis();
          unsigned long next_check = millis() 
            + milk_pump_data.forward_prime_duration 
              + (milk_pump_data.num_cups * milk_pump_data.duration_per_cup);
          milk_pump_data.next_check_time = next_check;
        } 
        else {
          unsigned long time_now = millis();
          if (time_now >= milk_pump_data.next_check_time) {  // We need to check for something
            if (milk_pump_data.is_on) {
              if (milk_pump_data.is_forward) {
                stopPump();
                runPumpBackward();
                milk_pump_data.is_forward = false;
                milk_pump_data.forward_done = true;
                milk_pump_data.reverse_start_time = millis();
                milk_pump_data.next_check_time = millis() + milk_pump_data.reverse_prime_duration;
              } 
              else {
                stopPump();
                milk_pump_data.is_on = false;
                milk_pump_data.reverse_done = true;
                time_now = millis();
                milk_pump_data.pump_end_time = time_now;
                active_states[DISPENSE_MILK] = false;
              }       
            } 
            else {
              active_states[DISPENSE_MILK] = false;
            }
          } 
          else {
            unsigned long mydelay = milk_pump_data.next_check_time - time_now;
            setDelay(mydelay);
          }
        }
      }
      break;
    case CONTROL_STRAINER:
      if (DEBUG) {
        Serial.println("CONTROL_STRAINER");
      }
      if (stove_data.boiling_point_reached) {
        if (strainer_data.is_lowered) { // Already lowered wait for sometime before stirring.
          if ((millis() - strainer_data.lowered_millis) >= strainer_data.steep_duration_before_stirring) {
            if (strainer_data.current_pos <= strainer_data.mid_pos) {
              strainer_data.sign = DIR_DOWN;
            } 
            else if (strainer_data.current_pos >= strainer_data.low_pos) {
              strainer_data.sign = DIR_UP;
            }   
            strainer_data.current_pos += (strainer_data.dx * strainer_data.sign) ;
            strainer_servo.write(strainer_data.current_pos);
            setDelay(strainer_data.move_delay);
          }
        } 
        else { // Lower and activate steep timer.
          strainer_data.sign = DIR_DOWN;
          if (strainer_data.current_pos < strainer_data.low_pos) {
            strainer_data.current_pos += (strainer_data.dx * strainer_data.sign) ;
            strainer_servo.write(strainer_data.current_pos);
            setDelay(strainer_data.move_delay);
          } 
          else {
            strainer_data.is_lowered = true;
            strainer_data.lowered_millis = millis();
            active_states[START_STEEPING] = true;
          }
        }
      }
      break;
    case CHECK_TEMP:
      {
        if (DEBUG) {
          Serial.println("CHECK_TEMP");
        }
        Serial.println("");
        stove_data.temperature = mlx.readObjectTempC();
        Serial.print("Temp From Infrared = ");    
        Serial.print(stove_data.temperature); 
        Serial.println("*C");
        // float temperature = getTemp();
        // Serial.print("Temp = ");
        // Serial.println(temperature);
        unsigned long stove_on_duration = millis() - stove_data.start_millis;
        if (stove_data.should_keep_on && (stove_on_duration < stove_data.max_on_duration)) {
          if (stove_data.temperature >= stove_data.boiling_point_upper) {
            if (stove_data.is_on) { 
              turnStoveOff();
            }
          } 
          else if (stove_data.temperature <= stove_data.boiling_point_lower) {
            if (!stove_data.is_on) { 
              turnStoveOn();
            }
          } 
          else {
            if (!stove_data.boiling_point_reached) {
              stove_data.boiling_point_reached_millis = millis();
              stove_data.boiling_point_reached = true;
            }
          }
        } 
        else {
          turnStoveOff();
          stove_data.stop_millis = millis();
          active_states[CHECK_TEMP] = false;
          active_states[DONE] = true;
        }
        setDelay(1000);
      }
      break;
    case START_STEEPING:
      if (DEBUG) {
        Serial.println("START_STEEPING");
      }
      if ((millis() - strainer_data.lowered_millis) >= strainer_data.steep_duration) {
        stove_data.should_keep_on = false;
        active_states[DONE] = true;
        active_states[CONTROL_STRAINER] = false;
        active_states[START_STEEPING] = false;
      }
      break;
    case CLEAN:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("    Cleaning");
      for (int i = 0 ; i < cleaner_data.num_cycles; ++i) {
        runPumpForward();
        delay(cleaner_data.forward_run_time);
        runPumpBackward();
        delay(cleaner_data.reverse_run_time);
      }
      stopPump();
      active_states[CLEAN] = false;
      active_states[SLEEP] = true;
      lcd.clear();
      break;
    case BOIL_EGG:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Boiling 'A' Egg");
      turnStoveOn();
      delay(egg_data.boil_time_base + (egg_data.num_eggs * egg_data.boil_time_per_egg));
      turnStoveOff();
      active_states[BOIL_EGG] = false;
      active_states[SLEEP] = true;
      lcd.clear();
      break;
    case REFRESH_LCD:
      if (DEBUG) {
        Serial.println("REFRESH_LCD");
      }
      {
        if (active_states[SLEEP]) {
          printSleepLCD();
        } 
        else if (active_states[CHECK_TEMP]){
          printTeaMakingLCD();
        } 
        setDelay(60000);
      }
      break;  
    case DONE:
      if (DEBUG) {
        Serial.println("DONE");
      }
      strainer_data.sign = DIR_UP;
      while (strainer_data.current_pos > strainer_data.high_pos) {
        strainer_data.current_pos += (strainer_data.dx * strainer_data.sign) ;
        strainer_servo.write(strainer_data.current_pos);
        delay(strainer_data.move_delay);
      }

      // If stove is Off, go back to sleep. One does not rest until the stove is Off.
      if (!stove_data.is_on) {
        // Deactivate all states except DONE
        for (int i = 0; i < num_states; ++i) {
          active_states[i] = false;
        }
        active_states[SLEEP] = true;
        active_states[REFRESH_LCD] = true;
        lcd.clear();
        lcd.setBacklight(LOW); 
        strainer_servo.detach();
      }
      break;
    }
  }
  delay(loop_delay);
}

void printSleepLCD() {
  time_t time_now = now();
  int curr_hour = hour(time_now);
  int curr_minute = minute(time_now);
  // No tea making in progress.
  // Print, Time, Alarm Time, 
  lcd.setCursor(0,0);
  lcd.print("Time : ");
  printTime(0, 7, curr_hour, curr_minute);
  lcd.setCursor(15, 0);
  lcd.print(milk_pump_data.num_cups, DEC);
  lcd.setCursor(0, 1);
  lcd.print("Alarm: ");
  printTime(1, 7, sleep_data.wakeup_hour, sleep_data.wakeup_minute);
  lcd.setCursor(13, 1);
  lcd.print("Cup");
}

void printTeaMakingLCD() {
  time_t time_now = now();
  int curr_hour = hour(time_now);
  int curr_minute = minute(time_now);
  printTime(0, 0, curr_hour, curr_minute);
  lcd.setCursor(6, 0);
  unsigned int temp_int = static_cast<unsigned int>(stove_data.temperature);
  lcd.print(temp_int, DEC);
  lcd.print("*C");
  printMillis(0, 11, millis() - stove_data.start_millis);
  lcd.setCursor(0, 1);
  if (stove_data.is_on) {
    lcd.print("Y");
  } 
  else {
    lcd.print("N");
  }
  if ( strainer_data.is_lowered) {
    printMillis(1, 2, millis() - strainer_data.lowered_millis);
  }
  lcd.setCursor(8,1);
  if (active_states[DISPENSE_MILK]) {
    lcd.print("P");
  } 
  else {
    lcd.print(" ");
  }
  lcd.setCursor(10,1);
  if (active_states[CHECK_TEMP]) {
    lcd.print("T");
  } 
  else {
    lcd.print(" ");
  }
  lcd.setCursor(12,1);
  if (active_states[CONTROL_STRAINER]) {
    lcd.print("S");
  } 
  else {
    lcd.print(" ");
  }
  lcd.setCursor(14,1);
  if (active_states[DONE]) {
    lcd.print("D");
  } 
  else {
    lcd.print(" ");
  }
}

void printMillis(int x, int y, unsigned long ms) {
  lcd.setCursor(y, x);
  int mins = static_cast<int>(ms / 60000);
  int secs = static_cast<int>((ms/1000) % 60);
  printTwoDigitToLCD(mins);
  lcd.print(":");
  printTwoDigitToLCD(secs);
}

void printTime(int x, int y, int h_m, int m_s) {
  lcd.setCursor(y, x);
  printTwoDigitToLCD(h_m);
  lcd.print(":");
  printTwoDigitToLCD(m_s);
}

void printTwoDigitToLCD(int val) {
  if (val < 10) {
    lcd.print ("0");
  }
  lcd.print(val, DEC);
}

bool shouldIWakeUp() {
  if (!(sleep_data.wakeup_hour == 0 && sleep_data.wakeup_minute == 0)) {
    time_t time_now = now();
    int curr_hour = hour(time_now);
    int curr_minute = minute(time_now);
    int delta = ((curr_hour * 60 + curr_minute) - (sleep_data.wakeup_hour * 60 + sleep_data.wakeup_minute));
    if (delta >= 0 && delta < 4) {
      return true;
    }
  }
  return false;
}

void stateInterrupt() {
  if ((millis() - state_lastDebounceTime) > debounceDelay) {
    state_lastDebounceTime = millis();
    if (DEBUG) {
      Serial.println("State Interrupt");
    }
    if (isButtonPressed(TEA_NOW_VS_ALARM_MODE_PIN)) {
      Init();
      active_states[SLEEP] = false;
      active_states[START_TEA] = true;
      loop_delay = 1000;
    } 
    else if (isButtonPressed(CLEANER_HOUR_MODE_PIN)) {
      Init();
      active_states[SLEEP] = false;
      active_states[CLEAN] = true;
      loop_delay = 1000;
    } 
    else if (isButtonPressed(EGG_MODE_MINUTE_PIN)) {
      Init();
      active_states[SLEEP] = false;
      active_states[BOIL_EGG] = true;
      loop_delay = 1000;
    }
  }
}

void timeInterrupt() {
  if ((millis() - time_lastDebounceTime) > debounceDelay) {
    time_lastDebounceTime = millis();
    if (DEBUG) {
      Serial.println("Time Interrupt");
    }
    if (isButtonPressed(TEA_NOW_VS_ALARM_MODE_PIN)) {
      if (milk_pump_data.num_cups == 1) {
        milk_pump_data.num_cups = 2;
      } 
      else {
        milk_pump_data.num_cups = 1;
      }
    }

    if (isButtonPressed(SET_ALARM_PIN)) {
      setAlarm();
      loop_delay = 1000;
    } 
    else {
      setTime();
      loop_delay = 1000;
    }
  }
}

void setTime() {
  if (isButtonPressed(CLEANER_HOUR_MODE_PIN)) {
    time_t time_now = now();
    int c_hour = hour(time_now);
    int c_min = minute(time_now);
    c_hour = (c_hour + 1) % 24;
    setTime(c_hour, c_min, 0, DAY, MONTH, YEAR);
  }

  if (isButtonPressed(EGG_MODE_MINUTE_PIN)) {
    time_t time_now = now();
    int c_hour = hour(time_now);
    int c_min = minute(time_now);
    c_min = (c_min + 1) % 60;
    setTime(c_hour, c_min, 0, DAY, MONTH, YEAR);
  }
}

void setAlarm() {
  if (isButtonPressed(CLEANER_HOUR_MODE_PIN)) {
    sleep_data.wakeup_hour = (sleep_data.wakeup_hour + 1) % 24;
  }
  if (isButtonPressed(EGG_MODE_MINUTE_PIN)) {
    sleep_data.wakeup_minute = (sleep_data.wakeup_minute + 1) % 60;
  }
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1010;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1020;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1030;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);  
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  return TemperatureSum;
}




#include <Time.h>
#include <OneWire.h> 
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Servo.h>

// Adafruit_MLX90614 mlx = Adafruit_MLX90614(); TODO: activate this.

#define RELAY1_PIN  2   // Positive Pump Terminal
#define RELAY2_PIN  3   // Negative Pump Terminal 
#define RELAY3_PIN  4   // Pump Power ON/OFF
#define RELAY4_PIN  5   // Stove Power ON/OFF
#define STRAINER_SERVO_PIN 9

#define ULONG_MAX ((1 << 31) - 1)

#define DIR_UP -1
#define DIR_DOWN 1


Servo strainer_servo;

#define DS18S20_PIN 8 //DS18S20 Signal pin on digital 6
//Temperature chip i/o
OneWire ds(DS18S20_PIN); // on digital pin 6

enum State {
  SLEEP = 0,
  START= 1,
  DISPENSE_MILK = 2,
  CONTROL_STRAINER = 3,
  CHECK_TEMP = 4,
  START_STEEPING = 5,
  DONE = 6
};

static const int num_states = 7;
const State kStateState = SLEEP;
const State kEndState = DONE;

bool active_states[num_states];
unsigned long loop_delay;

struct SleepData {
  int wakeup_hour;
  int wakeup_minute;
};
SleepData sleep_data;

struct StrainerData {
  static const int high_pos = 0;
  static const int low_pos = 180;
  static const int mid_pos = 100;
  static const unsigned long steep_duration_before_stirring = 3000;//120000;
  static const unsigned long steep_duration = 20000; //180000;
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
  static const float boiling_point_upper = 30.0f; //100.0f;
  static const float boiling_point_lower = 29.0f; // 98.0f;

  bool is_on;
  bool should_keep_on;
  unsigned long start_millis;
  bool boiling_point_reached;
  unsigned long boiling_point_reached_millis;
  unsigned long stop_millis;
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
  static const unsigned long reverse_prime_duration = 2000; // 40000;
  static const int num_cups = 2;

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
  Serial.println("Stove ON");
}

void turnStoveOff() {
  digitalWrite(RELAY4_PIN, HIGH);
  stove_data.is_on = false;
  Serial.println("Stove OFF");
}

void runPumpForward() {
  digitalWrite(RELAY1_PIN, LOW); // Runs Pump in forward direction
  digitalWrite(RELAY2_PIN, LOW); 
  digitalWrite(RELAY3_PIN, LOW); 
  Serial.println("Pump F");
}

void runPumpBackward() {
  digitalWrite(RELAY1_PIN, HIGH);  // Runs Pump in reverse direction.
  digitalWrite(RELAY2_PIN, HIGH); 
  digitalWrite(RELAY3_PIN, LOW); 
  Serial.println("Pump R");
}

void stopPump() {
  digitalWrite(RELAY3_PIN, HIGH);
 Serial.println("Pump Off"); 
}

void setup()
{    
  Serial.begin(9600);
  // Initialise the Arduino relay pins for OUTPUT
  pinMode(RELAY1_PIN, OUTPUT);       
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  // mlx.begin(); // TODO(activate then when mlx is delivered)
  stopPump();
  turnStoveOff();
  
  strainer_servo.attach(STRAINER_SERVO_PIN);  // attaches the servo on pin 9 to the servo object  
  active_states[SLEEP] = true;
  for (int i = 1; i < num_states; ++i) {
    active_states[i] = false;
  }
  setTime(6, 9, 55, 27, 11, 2014);
  sleep_data.wakeup_hour = 6;
  sleep_data.wakeup_minute = 10;
}

void loop()
{
  loop_delay = 1000;
  for (int state_int = kStateState; state_int <= kEndState; ++state_int) {
    State state = static_cast<State>(state_int);
    if (!active_states[state]) {
      continue;
    }
    switch (state) {
    case SLEEP:
    Serial.println("SLEEP");
      if (shouldIWakeUp()) {
        active_states[SLEEP] = false;
        active_states[START] = true;
      } 
      else {
        loop_delay = 10000;
        // loop_delay = 600000; TODO (replace with aboce.)
      }
      break;
    case START:
      resetMilkPumpData();
      resetStrainerData();
      resetStoveData();
      strainer_data.current_pos = strainer_data.high_pos;
      strainer_servo.write(strainer_data.high_pos);
      turnStoveOn();
      stove_data.should_keep_on = true;
      stove_data.start_millis = millis();
      active_states[START] = false;
      active_states[DISPENSE_MILK] = true;
      active_states[CHECK_TEMP] = true;
      active_states[CONTROL_STRAINER] = true;
      Serial.println("START");
      break;
    case DISPENSE_MILK:
    Serial.println("DISPENSE_MILK");
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
    Serial.println("CONTROL_STRAINER");
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
        Serial.println("CHECK_TEMP");
        // Serial.print("Temp From Infrared = "); 
        // Serial.print(mlx.readObjectTempC()); 
        // Serial.println("*C");
        float temperature = getTemp();
        Serial.print("Temp = ");
        Serial.println(temperature);
        if (stove_data.should_keep_on) {
          if (temperature >= stove_data.boiling_point_upper) {
            if (stove_data.is_on) { 
              turnStoveOff();
            }
          } 
          else if (temperature <= stove_data.boiling_point_lower) {
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
        }
        setDelay(1000);
      }
      break;
    case START_STEEPING:
      Serial.println("START_STEEPING");
      if ((millis() - strainer_data.lowered_millis) >= strainer_data.steep_duration) {
        stove_data.should_keep_on = false;
        active_states[DONE] = true;
        active_states[CONTROL_STRAINER] = false;
        active_states[START_STEEPING] = false;
      }
      break;
    case DONE:
      Serial.println("DONE");
      strainer_data.sign = DIR_UP;
      while (strainer_data.current_pos > strainer_data.high_pos) {
        strainer_data.current_pos += (strainer_data.dx * strainer_data.sign) ;
        strainer_servo.write(strainer_data.current_pos);
        delay(strainer_data.move_delay);
      }
      if (!stove_data.is_on) {
        active_states[DONE] = false;
        active_states[SLEEP] = true;
      }
      break;
    }
  }
  delay(loop_delay);
}

bool shouldIWakeUp() {
  time_t time_now = now();
  int curr_hour = hour(time_now);
  int curr_minute = minute(time_now);
  Serial.print(curr_hour);
  Serial.print(":");
  Serial.print(curr_minute);
  Serial.print("  ");
  Serial.print(sleep_data.wakeup_hour);
  Serial.print(":");
  Serial.println(sleep_data.wakeup_minute);
  
  int delta = ((curr_hour * 60 + curr_minute) - (sleep_data.wakeup_hour * 60 + sleep_data.wakeup_minute));
  if (delta >= 0 && delta < 4) {
    return true;
  }
  return false;
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


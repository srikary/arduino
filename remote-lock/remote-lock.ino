// Connect the VT to interrupt 0 pin
// Connect D0 to Analog 0 pin.

#include <Servo.h>

#define SERVO_PIN 5
#define OPEN_POS 160
#define CLOSE_POS 0
#define KEEP_OPEN_MSEC 5000
Servo myservo;
volatile int d4_val; // storage for data states
volatile bool dirty = false; // interrupt has occurred flag
int pos = 0;
int move_delay = 7;

// Interrupt Service Routine attached to INT0 vector
void pinInt0ISR()
{
  d4_val = analogRead(A0);
  // Provide a visual clue of the interrupt
  digitalWrite(13, !digitalRead(13));  // Toggle LED on pin 13
  // Grab the data states
  dirty = true;                          // flag interrupt occurance
}

void setup() {
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);
  pos = CLOSE_POS;
  myservo.write(pos);
  myservo.detach();
  attachInterrupt(0, pinInt0ISR, RISING);
}

void openLatch() {
  while (pos < OPEN_POS) {
    pos ++;
    myservo.write(pos);
    delay(move_delay);
  }
}

void closeLatch() {
  while (pos > CLOSE_POS) {
    pos --;
    myservo.write(pos);
    delay(move_delay);
  }
}

void loop() {
  if (dirty)
  {
    dirty = false;              // clear interrupt occurance flag
    myservo.attach(SERVO_PIN);
    openLatch();
    delay(KEEP_OPEN_MSEC);
    closeLatch();
    myservo.detach();
  }
}

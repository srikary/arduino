

// include the servo library
#include <Servo.h>

Servo servo1; 
Servo servo2;
Servo servo3; 
Servo servo4;
Servo servo5; 
Servo servo6;

void setup() {
  servo1.attach(2); 
  servo2.attach(3); 
  servo3.attach(4); 
  servo4.attach(5);
  servo5.attach(6); 
  servo6.attach(7);
  Serial.begin(9600); // open a serial connection to your computer
  servo1.write(0);
  servo2.write(90);
  servo3.write(0);
  servo4.write(90);
  servo5.write(0);
  servo6.write(0);
}

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    int servo_num = Serial.parseInt();
    // do it again:
    int pos = Serial.parseInt();
    switch(servo_num) {
      case 1:
        servo1.write(pos);
        break;
      case 2:
        servo2.write(pos);
        break;
      case 3:
        servo3.write(pos);
        break;
      case 4:
        servo4.write(pos);
        break;
      case 5:
        servo5.write(pos);
        break;
      case 6:
        servo6.write(pos);
        break;
    }
  }
}









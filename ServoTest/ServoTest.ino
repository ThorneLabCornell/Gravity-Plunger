#include <Servo.h>

Servo Servo1;

int servoPin = 9;
int potPin = A0;


void setup() {
    Servo1.attach(servoPin);
    Serial.begin(9600);  
}


void loop() {
  int angle = 0;
  int step = 1;
  while(angle<45){
    angle++;
    Serial.println(angle);
    Servo1.write(angle);
  }
  delay(1000);
  // Downward cycle
  while(angle>0){
    angle--;
    Serial.println(angle);
    Servo1.write(angle);
  }
    
}
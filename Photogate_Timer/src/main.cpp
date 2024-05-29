#include <Arduino.h>


const int pg1 = A0; // Pin for photo interrupter 1
const int pg2 = A1; // Pin for photo interrupter 2
unsigned long time1 = 0; // Time when interrupter 1 was activated
unsigned long time2 = 0; // Time when interrupter 2 was activated
int pg1val = 1023;
int pg2val = 1023;
unsigned long reactionTime;

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(9600);         
  pinMode(pg1, INPUT);  
  pinMode(pg2, INPUT);
  analogReference(DEFAULT);    // Set the default reference voltage (important for 3.3V boards)
}

void loop() {
  while (pg1val > 100) {
    pg1val = analogRead(pg1);
    Serial.println(pg1val);
  }
  time1 = millis();
  while (pg2val > 100) {
    pg2val = analogRead(pg2);
    Serial.println(pg2val);
  }
  time2 = millis();
  reactionTime = time2 - time1;
  Serial.print("Time difference: ");
  Serial.println(reactionTime);
  delay(10000);
  pg1val = 1023;
  pg2val = 1023;
}
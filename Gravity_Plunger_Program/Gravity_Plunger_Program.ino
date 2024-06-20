#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int waitTime = 2000;
Servo Servo1;
const int pg1 = A0; // Pin for photo interrupter 1
const int pg2 = A1; // Pin for photo interrupter 2
const int button = A3; // Pin for the button
const int servoPin = 9;
unsigned long time1 = 0; // Time when interrupter 1 was activated
unsigned long time2 = 0; // Time when interrupter 2 was activated
int pg1val = 1023;
int pg2val = 1023;
unsigned long reactionTime;
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
    Serial.begin(9600);  
    Servo1.attach(servoPin);
    lcd.init(); // Initialize the LCD
    lcd.backlight(); // Turn on the backlight
    pinMode(button, INPUT);  
    pinMode(pg1, INPUT);  
    pinMode(pg2, INPUT);
    
}

void writePos(int angle)
{
  int pulseWidth = map(angle, 0, 270, 500, 2500); 
  Servo1.writeMicroseconds(pulseWidth);
}

void loop() {
  int timeOut = 0;
  writePos(90);
  delay(1000);
  lcd.setCursor(0, 0); // Set cursor to the first column of the first row
  lcd.print("Ready"); // Display label
  while(!digitalRead(button)) {
  }
  lcd.setCursor(0, 0);
  lcd.print("Plunging");
  writePos(110);
  delay(50);
  while (pg1val > 100 && timeOut < 99999) {
    pg1val = analogRead(pg1);
    timeOut++;
  }
  time1 = millis();
  lcd.setCursor(0, 0);
  lcd.print("Flag 1 Hit");
  delay(waitTime);
  writePos(180);
  timeOut = 0;
  
  while (pg2val > 100 && timeOut < 99999) {
    pg2val = analogRead(pg2);
    timeOut++;
  }
  lcd.setCursor(0, 0);
  lcd.print("Flag 2 Hit");
  time2 = millis();
  reactionTime = time2 - time1;
  lcd.clear(); // Clear the LCD screen
  lcd.setCursor(0, 0); // Set cursor to the first column of the first row
  lcd.print("Time difference:"); // Display label
  delay(100);
  lcd.setCursor(0, 1); // Set cursor to the first column of the second row
  lcd.print(reactionTime); // Display time difference on the LCD
  delay(100);
  lcd.print(" ms");
  while(!digitalRead(button)) {
  }
  pg1val = 1023;
  pg2val = 1023;
  lcd.clear();
}
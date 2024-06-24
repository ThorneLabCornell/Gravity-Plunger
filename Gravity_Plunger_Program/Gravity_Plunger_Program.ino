#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int waitTime = 000;
Servo Servo1;
const int pg1 = A0; // Pin for photo interrupter 1
const int pg2 = A1; // Pin for photo interrupter 2
const int button = A3; // Pin for the button
const int servoPin = 9;
unsigned long reactionStart = 0; // Time when interrupter 1 was activated
unsigned long reactionEnd = 0; // Time when interrupter 2 was activated
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
  // Set up for plunge
  int timeOut = 0;
  writePos(90);
  delay(1000);
  lcd.clear();
  delay(50);
  lcd.setCursor(0, 0); // Set cursor to the first column of the first row
  lcd.print("Ready"); // Display label

  // Initiating plunge
  while(!digitalRead(button)) {}
  lcd.clear();
  delay(50);
  lcd.print("Plunging");
  writePos(110);

  // Wait for first photogate to be activated
  while (pg1val > 100 && timeOut < 99999) {
    pg1val = analogRead(pg1);
    timeOut++;
  }
  reactionStart = millis();
  delay(waitTime);
  writePos(180);
  
  // Wait for second photogate to be activated
  while (pg2val > 100 && timeOut < 99999) {
    pg2val = analogRead(pg2);
    timeOut++;
  }
  reactionEnd = millis();
  
  // Display reaction time information
  reactionTime = reactionEnd - reactionStart;
  lcd.clear();
  delay(50);
  lcd.setCursor(0, 0);
  lcd.print(reactionTime);
  lcd.print("ms");
  
  
  

  while(!digitalRead(button)) {}
  // Reset photogate read values
  pg1val = 1023;
  pg2val = 1023;
}
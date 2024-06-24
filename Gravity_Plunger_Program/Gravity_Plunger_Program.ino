#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int waitTime = 00;
Servo Servo1;
const int pg1 = A0; // Pin for photo interrupter 1
const int pg2 = A1; // Pin for photo interrupter 2
const int button = A3; // Pin for the button
const int servoPin = 9;
const float  flagLength = 0.00654;

int pg1val = 1023;
int pg2val = 1023;

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
  unsigned long reactionStart; // Time when interrupter 1 was activated
  unsigned long reactionEnd; // Time when interrupter 2 was activated
  unsigned long finalTime;
  float finalVelocity;
  int reactionTime;
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
  reactionStart = micros();
  delay(waitTime);
  writePos(180);
  
  // Wait for second photogate to be activated
  while (pg2val > 100 && timeOut < 99999) {
    pg2val = analogRead(pg2);
    timeOut++;
  }
  reactionEnd = micros();
  Serial.println("Wait for second flag to pass");
  while (pg2val < 100 && timeOut < 99999) {
    pg2val = analogRead(pg2);
    timeOut++;
  }
  finalTime = micros();
  
  // Display reaction time information
  reactionTime = (reactionEnd - reactionStart)/1000;
  finalVelocity = (flagLength/float(finalTime - reactionEnd))*1000000.0;
  lcd.clear();
  delay(50);
  lcd.setCursor(0, 0);
  lcd.print(reactionTime);
  lcd.print("ms");
  lcd.setCursor(0, 10);
  lcd.print(finalVelocity);
  lcd.print("m/s");
  
  Serial.print("Reaction End: ");
  Serial.println(reactionEnd);
  Serial.print("Reaction Start: ");
  Serial.println(reactionStart);
  Serial.print("Reaction Time: ");
  Serial.println(reactionTime);
  Serial.print("Final Time: ");
  Serial.println(finalTime);
  Serial.print("Final Velocity: ");
  Serial.println(finalVelocity);
  
  
  

  while(!digitalRead(button)) {}
  // Reset photogate read values
  pg1val = 1023;
  pg2val = 1023;
}
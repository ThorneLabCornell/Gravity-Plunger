#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int waitTime = 500;

const int pg1 = A0; // Pin for photo interrupter 1
const int pg2 = A1; // Pin for photo interrupter 2
const int solenoid = A2; // Pin for the solenoid
const int button = A3; // Pin for the button
unsigned long time1 = 0; // Time when interrupter 1 was activated
unsigned long time2 = 0; // Time when interrupter 2 was activated
int pg1val = 1023;
int pg2val = 1023;
unsigned long reactionTime;
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
  Serial.begin(9600);         
  pinMode(pg1, INPUT);  
  pinMode(pg2, INPUT);
  pinMode(solenoid, OUTPUT);
  pinMode(button, INPUT);
  analogReference(DEFAULT);    // Set the default reference voltage
}

void loop() {
  lcd.setCursor(0, 0); // Set cursor to the first column of the first row
  lcd.print("Waiting"); // Display label

  while(!digitalRead(button)) {
  }
  digitalWrite(solenoid, HIGH);
  delay(10);
  digitalWrite(solenoid, LOW);
  delay(waitTime);
  digitalWrite(solenoid, HIGH);
  delay(100);
  digitalWrite(solenoid, LOW);

  Serial.println("Waiting for first flag");
  while (pg1val > 100) {
    pg1val = analogRead(pg1);
  }
  time1 = millis();

  Serial.println("Waiting for second flag");
  while (pg2val > 100) {
    pg2val = analogRead(pg2);
  }
  time2 = millis();

  reactionTime = time2 - time1;
  Serial.print("Time difference: ");
  Serial.println(reactionTime);
  lcd.clear(); // Clear the LCD screen
  lcd.setCursor(0, 0); // Set cursor to the first column of the first row
  lcd.print("Time difference:"); // Display label
  lcd.setCursor(0, 1); // Set cursor to the first column of the second row
  lcd.print(reactionTime); // Display time difference on the LCD
  lcd.print(" ms");
  delay(10000);
  pg1val = 1023;
  pg2val = 1023;
  lcd.clear();
}
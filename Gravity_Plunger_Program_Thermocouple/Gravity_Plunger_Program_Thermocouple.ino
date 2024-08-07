#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int waitTime = 0; // Modify this value to add pause when waitTime > 0
const int sensorOne = A1;     // Pin for substrate level photoelectric sensor
const int sensorTwo = A0;     // Pin for nitrogen level photoelectric sensor
const int button = A3;  // Pin for the activation button
const int servoPin = 9; // Pin for servo PWM signal
const int stmPin = 13; // Pin for servo PWM signal
const float flagLength = 0.013; // [m]
const int threshold = 800; // adc readout below this value indicates sensor detects flag
int sensorOneVal = 1023; // max value of adc
int sensorTwoVal = 1023;

Servo Servo1;
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  Serial.begin(9600); // Initiatize serial communication
  Servo1.attach(servoPin); // Initialize the servo
  // lcd.init();       // Initialize the LCD
  // lcd.backlight();  // Turn on the backlight
  pinMode(button, INPUT);
  pinMode(sensorOne, INPUT); // Initiatize photogate 1
  pinMode(sensorTwo, INPUT); // Initiatize photogate 2
  pinMode(stmPin, OUTPUT);
}

// Maps servo angle [0,270] with corresponding PWM signal [500,2500] and writes it to PWM
void writePos(int angle) {
  int pulseWidth = map(angle, 0, 270, 500, 2500); 
  Servo1.writeMicroseconds(pulseWidth);
}

void loop() {
  // Set up for plunge
  unsigned long reactionStart;  // Time when interrupter 1 was activated
  unsigned long reactionEnd;    // Time when interrupter 2 was activated
  unsigned long finalTime;
  float finalVelocity;
  int reactionTime;

  writePos(90); // Default arm position allows for positioning rod for plunge
  delay(1000); // Wait time to allow for arm to reach position
  // lcd.clear();
  delay(50);
  // lcd.setCursor(0, 0);  // Set cursor to the first column of the first row
  // lcd.print("Ready");   // Display label

  
  while (!digitalRead(button)) {} // Wait for activation button to be pressed
  // Initiating plunge
  // lcd.clear();
  delay(50);
  // lcd.print("Plunging");
  if (waitTime != 0) { // If wait time is 0, the arm should drop fully to allow for plunge, otherwise only drop by 20 degrees to allow for deposition then pause
    writePos(110);
  } else {
    writePos(180);
  }

  while (sensorOneVal > threshold) { // Wait for flag to enter substrate level photoelectric
    sensorOneVal = analogRead(sensorOne);
  }
  reactionStart = micros();
  
  // Pause Time
  if (waitTime != 0) {
    delay(waitTime);
    writePos(180);
  }
  digitalWrite(stmPin, HIGH);

  while (sensorTwoVal > threshold) { // Wait for flag to enter nitrogen level photoelectric
    sensorTwoVal = analogRead(sensorTwo);
  }
  reactionEnd = micros();
  while (sensorTwoVal < threshold) { // Wait for flag to leave nitrogen level photogate
    sensorTwoVal = analogRead(sensorTwo);
  }
  finalTime = micros();
  
  digitalWrite(stmPin,LOW);

  // Display reaction time information to LCD
  reactionTime = (reactionEnd - reactionStart) / 1000;
  finalVelocity = (flagLength / float(finalTime - reactionEnd)) * 1000000.0;
  // lcd.clear();
  // delay(100);
  // lcd.setCursor(0, 0);
  // lcd.print(reactionTime);
  // lcd.print("ms");
  // lcd.setCursor(0, 10);
  // lcd.print(finalVelocity);
  // lcd.print("m/s");

  // Display reaction time information to serial
  Serial.print("Reaction Time: ");
  Serial.print(reactionTime);
  Serial.println(" ms");

  Serial.print("Final Time: ");
  Serial.print((finalTime - reactionStart)/1000);
  Serial.println(" ms");

  Serial.print("Final Velocity: ");
  Serial.print(finalVelocity);
  Serial.println(" m/s");

  Serial.println("------Plunge Complete------");



  while (!digitalRead(button)) {} // Wait for activation button to be pressed to reset
  // Reset photogate read values
  sensorOneVal = 1023;
  sensorTwoVal = 1023;
}
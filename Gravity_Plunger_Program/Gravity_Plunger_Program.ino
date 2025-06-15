#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int waitTime = 0; // Modify this value to add pause when waitTime > 0
const int sensorOne = A1;     // Pin for substrate level photoelectric sensor
const int sensorTwo = A0;     // Pin for nitrogen level photoelectric sensor
const int button = A3;  // Pin for the activation button
const int servoPin = 9; // Pin for servo PWM signal
//const float flagLength = 0.013; // [m]
const float sensorStartStartLength = 0.0207; // DMW - 2025/05/09: dual sensor start-to-start distance [m]
const float sensorStartEndLength = 0.0288; // DMW - 2025/05/09: dual sensor start-to-end distance [m]
const int threshold = 875; // adc readout below this value indicates sensor detects flag
int sensorOneVal = 1023; // max value of adc
int sensorOneVal2 = 1023;
int sensorTwoVal = 1023;
int sensorTwoVal2 = 1023;
bool firstAdjust = true;

Servo Servo1;
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  Serial.begin(9600); // Initiatize serial communication
  Servo1.attach(servoPin); // Initialize the servo
  lcd.init();       // Initialize the LCD
  lcd.backlight();  // Turn on the backlight
  pinMode(button, INPUT);
  pinMode(sensorOne, INPUT); // Initiatize photogate 1
  pinMode(sensorTwo, INPUT); // Initiatize photogate 2
}

void wait(unsigned long milliseconds) {
    unsigned long microseconds = milliseconds * 1000; // Convert milliseconds to microseconds

    // Loop to handle the delay in microseconds
    while (microseconds > 0) {
        // If the remaining delay is larger than 16383 (the maximum argument for delayMicroseconds)
        if (microseconds > 16383) {
            delayMicroseconds(16383);  // Delay for 16383 microseconds
            microseconds -= 16383;     // Subtract 16383 microseconds from the remaining delay
        } else {
            delayMicroseconds(microseconds); // Delay for the remaining time
            microseconds = 0;                // Set remaining delay to 0
        }
    }
}

// Maps servo angle [0,270] with corresponding PWM signal [500,2500] and writes it to PWM
void writePos(int angle) {
  int pulseWidth = map(angle, 0, 270, 500, 2500); 
  Servo1.writeMicroseconds(pulseWidth);
}

void loop() {
  // Set up for plunge
  unsigned long reactionStart;  // Time when interrupter 1 was activated
  unsigned long reactionStart2; // DMW - 2025/05/09
  unsigned long reactionEnd;    // Time when interrupter 2 was activated
  unsigned long finalTime;
  unsigned long reactionEnd2;   // DMW - 2025/05/09
  float finalVelocity;
  float finalVelocity1;
  float finalVelocity2;
  float finalVelocity3;
  float finalVelocity4;
  float finalVelocity5;
  float finalVelocity6;
  int reactionTime;

  // DMW - 2025/05/05
  if (firstAdjust) { // Adjust plunger for the first time before moving servo
    while (!digitalRead(button)) {} // Wait for activation button to be pressed
    firstAdjust = false;
  }

  writePos(90); // Default arm position allows for positioning rod for plunge
  wait(1000); // Wait time to allow for arm to reach position
  lcd.clear();
  wait(50);
  lcd.setCursor(0, 0);  // Set cursor to the first column of the first row
  lcd.print("Ready");   // Display label
  lcd.setCursor(0,1);
  lcd.print("Wait: ");
  lcd.print(waitTime);
  lcd.print("ms");

  
  while (!digitalRead(button)) {} // Wait for activation button to be pressed
  // Initiating plunge
  lcd.clear();
  wait(50);
  lcd.print("Plunging");
  if (waitTime != 0) { // If wait time is 0, the arm should drop fully to allow for plunge, otherwise only drop by 20 degrees to allow for deposition then pause
    writePos(110);
  } else {
    writePos(180);
  }

  while (sensorOneVal > threshold) { // Wait for flag to enter substrate level photoelectric
    sensorOneVal = analogRead(sensorOne);
    sensorOneVal2 = analogRead(sensorOne);
    //Serial.print("Sensor One Value: ");
    //Serial.println(sensorOneVal);
    //TODO: prevent infinite loop due to lower loop platform positioning -> no need based on mechanical positioning constraints
  }
  reactionStart = micros();
  
  // Pause Time
  if (waitTime != 0) {
    wait(waitTime);
    writePos(180);
  }

  // DMW - 2025/05/09
  while (sensorOneVal < threshold) { // Wait for flag to enter substrate level photoelectric
    sensorOneVal = analogRead(sensorOne);
  }
  reactionStart2 = micros();
  
  while (sensorTwoVal > threshold) { // Wait for flag to enter nitrogen level photoelectric
    sensorTwoVal = analogRead(sensorTwo);
    sensorTwoVal2 = analogRead(sensorTwo); // lowest time point testing
  }
  reactionEnd = micros();
  
  while (sensorTwoVal < threshold) { // Wait for flag to leave nitrogen level photogate
    sensorTwoVal = analogRead(sensorTwo);
  }
  finalTime = micros();
  reactionEnd2 = micros();

  // Display reaction time information to LCD
  //reactionTime = (reactionEnd - reactionStart) / 1000;
  //finalVelocity = (flagLength / float(finalTime - reactionEnd)) * 1000000.0;
  finalVelocity1 = (sensorStartStartLength / float(reactionTime)) * 1000000.0; // DMW - 2025/05/09
  //finalVelocity2 = 
  lcd.clear();
  wait(100);
  lcd.setCursor(0, 0);
  //lcd.print(reactionTime);
  //lcd.print("ms");
  
  lcd.setCursor(0, 10);
  //lcd.print(finalVelocity);
  //lcd.print("m/s");

  // Display reaction time information to serial
  //Serial.print("Reaction Time: ");
  //Serial.print(reactionTime);
  //Serial.println(" ms");

  //Serial.print("Final Time: ");
  //Serial.print((finalTime - reactionStart)/1000);
  //Serial.println(" ms");

  //Serial.print("Final Velocity: ");
  //Serial.print(finalVelocity);
  //Serial.println(" m/s");

  //Serial.print("Reaction End Time: ");
  //Serial.println(reactionEnd);

  //Serial.print("Reaction End Sensor Value: ");
  //Serial.println(sensorTwoVal2);

  //Serial.print("Final Time: ");
  //Serial.println(finalTime);

  //Serial.print("Reaction Final Time Value: ");
  //Serial.println(sensorTwoVal);

  //Serial.print("Velocity 1: ");
  //Serial.println(
  
  Serial.println("------Plunge Complete------");



  while (!digitalRead(button)) {} // Wait for activation button to be pressed to reset
  // Reset photogate read values
  sensorOneVal = 1023;
  sensorOneVal2 = 1023;
  sensorTwoVal = 1023;
  sensorTwoVal2 = 1023;
}

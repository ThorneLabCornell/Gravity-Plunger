/*
  DMW – 2025-06-11
  Gravity Plunger Program w/ Hall Effect Sensors
  ---------------------------------------------------
*/

// Libraries
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Constants
const int waitTime = 0;                         // plunge-pause-plunge (PPP) wait time
const uint8_t depoSensorPin = A0;               // deposition sensor pin
const uint8_t LN2SensorPin = A1;                // nitrogen level sensor pin
const uint8_t buttonPin = A3;                   // button pin
const uint8_t servoPin = 9;                     // servo pin
const uint16_t samplingTime_US = 500;           // sampling time (us): 500 us -> 2 kHz
const uint16_t depoSensorMinNoMagnet = 513;     // deposition sensor minimum threshold w/o magnet detection (ADC)
const uint16_t depoSensorMaxNoMagnet = 519;     // deposition sensor maximum threshold w/o magnet detection (ADC)
const uint16_t LN2SensorMinNoMagnet = 510;      // nitrogen level sensor minimum threshold w/o magnet detection (ADC output)
const uint16_t LN2SensorMaxNoMagnet = 514;      // nitrogen level sensor maximum threshold w/o magnet detection (ADC output)
const float LN2SensorThresholdDistance = 0.022; // nitrogen level sensor threshold distance (m)
const unsigned long plungeTimeOut = 300000;

// Global Variables
uint16_t depoSensorVal = 0;                     // deposition sensor value (ADC)
uint16_t LN2SensorVal = 0;                      // nitrogen level sensor value (ADC)
//bool firstAdjust = true;                      // program start plunger adjustment Boolean

Servo Servo1;
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address to 0x27 for a 16 chars and 2 line display

/* ---------- 3-point moving-average state ---------- */
const uint8_t FILT_N = 3;  // window length
uint16_t buf1[FILT_N] = { 0 };
uint16_t buf2[FILT_N] = { 0 };
uint32_t tot1 = 0, tot2 = 0;  // running sums
uint8_t idx = 0;              // circular index
float avg1 = 0, avg2 = 0;

/* ---------- extremum tracking --------------------- */
uint16_t depoSensorMax = 0, depoSensorMin = 1023, LN2SensorMax = 0, LN2SensorMin = 1023;
uint32_t depoSensorMaxTime = 0, depoSensorMinTime = 0, LN2SensorMaxTime = 0, LN2SensorMinTime = 0;

extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;

void resetMicros()
{
    cli();                // stop interrupts while we poke Timer-0
    TCNT0 = 0;            // reset the hardware counter (8-bit)
    timer0_overflow_count = 0;
    timer0_millis         = 0;   // keeps micros() and millis() consistent
    sei();                // re-enable interrupts
}

void setup() {
  Serial.begin(115200);
  delay(100);  // USB settle

  #if defined(analogReadResolution)
    analogReadResolution(10);
  #endif

  Servo1.attach(servoPin);
  lcd.init();
  lcd.backlight();
  pinMode(buttonPin, INPUT);
  pinMode(depoSensorPin, INPUT);
  pinMode(LN2SensorPin, INPUT);

  /* prime the filter with the first sample so start-up values are reasonable */
  uint16_t s1 = analogRead(depoSensorPin);
  uint16_t s2 = analogRead(LN2SensorPin);
  for (uint8_t i = 0; i < FILT_N; ++i) {
    buf1[i] = s1;
    tot1 += s1;
    buf2[i] = s2;
    tot2 += s2;
  }
  avg1 = depoSensorMax = depoSensorMin = s1;
  avg2 = LN2SensorMax = LN2SensorMin = s2;
}

/* update running 3-point average for deposition sensor */
inline void updateMovingAvg1(uint16_t s1) {
  /* remove oldest sample from totals */
  tot1 -= buf1[idx];

  /* store new samples */
  buf1[idx] = s1;

  /* add new samples to totals */
  tot1 += s1;

  /* advance circular index */
  idx = (idx + 1) % FILT_N;

  /* compute current averages */
  avg1 = (float)tot1 / FILT_N;
}

/* update running 3-point average for nitrogen level sensor */
inline void updateMovingAvg2(uint16_t s2) {
  /* remove oldest sample from totals */
  tot2 -= buf2[idx];

  /* store new samples */
  buf2[idx] = s2;

  /* add new samples to totals */
  tot2 += s2;

  /* advance circular index */
  idx = (idx + 1) % FILT_N;

  /* compute current averages */
  avg2 = (float)tot2 / FILT_N;
}

void wait(unsigned long milliseconds) {
  unsigned long microseconds = milliseconds * 1000;  // Convert milliseconds to microseconds

  // Loop to handle the delay in microseconds
  while (microseconds > 0) {
    // If the remaining delay is larger than 16383 (the maximum argument for delayMicroseconds)
    if (microseconds > 16383) {
      delayMicroseconds(16383);  // Delay for 16383 microseconds
      microseconds -= 16383;     // Subtract 16383 microseconds from the remaining delay
    } else {
      delayMicroseconds(microseconds);  // Delay for the remaining time
      microseconds = 0;                 // Set remaining delay to 0
    }
  }
}

// Maps servo angle [0,270] with corresponding PWM signal [500,2500] and writes it to PWM
void writePos(int angle) {
  int pulseWidth = map(angle, 0, 270, 500, 2500);
  Servo1.writeMicroseconds(pulseWidth);
}

void loop() {
  unsigned long reactionStart = 0;
  unsigned long reactionEnd = 0;
  unsigned long finalTime = 0;
  float finalVelocity = 0;
  int reactionTime = 0;

  resetMicros();

/*
  if (firstAdjust) {                    // Adjust plunger for the first time before moving servo
    while (!digitalRead(buttonPin)) {}  // Wait for button to be pressed
    firstAdjust = false;
  }
*/

  writePos(90);  // Default arm position allows for positioning rod for plunge
  wait(1000);    // Wait time to allow for arm to reach position
  lcd.clear();
  wait(50);
  lcd.setCursor(0, 0);  // Set cursor to the first column of the first row
  lcd.print("Ready");   // Display label
  lcd.setCursor(0, 1);
  lcd.print("Wait: ");
  lcd.print(waitTime);
  lcd.print("ms");


  while (!digitalRead(buttonPin)) {}  // Wait for button to be pressed
  // Initiating plunge
  lcd.clear();
  wait(50);
  lcd.print("Plunging");
  if (waitTime != 0) {  // If wait time is 0, the arm should drop fully to allow for plunge, otherwise only drop by 20 degrees to allow for deposition then pause
    writePos(110);
  } else {
    writePos(180);
  }

  while (avg1 > depoSensorMinNoMagnet && avg1 < depoSensorMaxNoMagnet) { // Wait for deposition sensor to detect flag magnetic field
    depoSensorVal = analogRead(depoSensorPin);
    updateMovingAvg1(depoSensorVal);
  }
  reactionStart = micros();

  // Pause Time
  if (waitTime != 0) {
    wait(waitTime);
    writePos(180);
  }
  
  while (avg2 > LN2SensorMinNoMagnet && avg2 < LN2SensorMaxNoMagnet) { // Wait for nitrogen level sensor to detect flag magnetic field
    LN2SensorVal = analogRead(LN2SensorPin);
    updateMovingAvg2(LN2SensorVal);
  }
  reactionEnd = micros();
  
  while (avg2 <= LN2SensorMinNoMagnet || avg2 >= LN2SensorMaxNoMagnet) { // Wait for nitrogen level sensor to stop detecting flag magnetic field
    LN2SensorVal = analogRead(LN2SensorPin);
    updateMovingAvg2(LN2SensorVal);
  }
  finalTime = micros();

  // Display reaction time information to LCD
  reactionTime = (reactionEnd - reactionStart) / 1000;
  finalVelocity = (LN2SensorThresholdDistance / float(finalTime - reactionEnd)) * 1000000.0;

  lcd.clear();
  wait(100);
  lcd.setCursor(0, 0);
  lcd.print(reactionTime);
  lcd.print("ms");
  
  lcd.setCursor(0, 10);
  lcd.print(finalVelocity);
  lcd.print("m/s");

  // Display reaction time information to serial
  //Serial.print("Reaction Time: ");
  //Serial.print(reactionTime);
  //Serial.println(" ms");

  //Serial.print("Final Velocity: ");
  //Serial.print(finalVelocity);
  //Serial.println(" m/s");

  Serial.print("Reaction Start: ");
  Serial.print(reactionStart);
  Serial.println(" us");

  Serial.print("Reaction End: ");
  Serial.print(reactionEnd);
  Serial.println(" us");

  Serial.print("Final Time: ");
  Serial.print(finalTime);
  Serial.println(" us");

  Serial.println("------Plunge Complete------");

  while (!digitalRead(buttonPin)) {} // Wait for activation button to be pressed to reset
  // Reset sensor read values
  //depoSensorVal = 1023;
  //LN2SensorVal = 1023;
}

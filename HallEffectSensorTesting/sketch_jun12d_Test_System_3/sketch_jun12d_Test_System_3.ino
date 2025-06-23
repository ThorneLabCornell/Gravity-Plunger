/*
  DMW – 2025-06-12  UPDATED 2025-06-13
  Gravity Plunger Program w/ Hall-Effect Sensors
*/

// ───────── libraries ───────────────────────────
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ───────── constants ───────────────────────────
const int      waitTime        = 0;      // pause (ms)
const uint8_t  depoSensorPin   = A0;
const uint8_t  LN2SensorPin    = A1;
const uint8_t  buttonPin       = A3;
const uint8_t  servoPin        = 9;
const uint16_t samplingTime_US = 500;    // 2 kSa s⁻¹
const float    LN2SensorThresholdDistance = 0.022;  // m
const unsigned long plungeTimeoutUS = 3000000UL;      // 3 000 ms

/* ─────── NEW: error thresholds ──────────────── */
const uint16_t TIME_ERR_THRESHOLD_MS = 250;  // NEW
const float    VEL_ERR_THRESHOLD     = 3.5f; // NEW

/* one-line knob for window width ─────────────── */
const uint8_t  noMagMargin = 2;

// ───────── dynamic “no-magnet” windows ─────────
uint16_t depoSensorMinNoMagnet = 0;
uint16_t depoSensorMaxNoMagnet = 0;
uint16_t LN2SensorMinNoMagnet  = 0;
uint16_t LN2SensorMaxNoMagnet  = 0;

// ───────── globals / state ─────────────────────
uint16_t depoSensorVal = 0, LN2SensorVal = 0;

Servo Servo1;
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ---------- 3-point moving-average state ------ */
const uint8_t FILT_N = 3;
uint16_t buf1[FILT_N] = {0}, buf2[FILT_N] = {0};
uint32_t tot1 = 0,        tot2 = 0;
uint8_t  idx  = 0;
float    depoSensorFiltered = 0,        LN2SensorFiltered = 0;

/* ------------- helpers ------------------------ */
uint8_t idx1 = 0, idx2 = 0;

inline void updateMovingAvg1(uint16_t s) {
  tot1 -= buf1[idx1];
  buf1[idx1] = s; tot1 += s;
  idx1 = (idx1 + 1) % FILT_N;
  depoSensorFiltered = (float)tot1 / FILT_N;
}

inline void updateMovingAvg2(uint16_t s) {
  tot2 -= buf2[idx2];
  buf2[idx2] = s; tot2 += s;
  idx2 = (idx2 + 1) % FILT_N;
  LN2SensorFiltered = (float)tot2 / FILT_N;
}

void wait(unsigned long ms) {
  unsigned long us = ms * 1000UL;
  while (us) {
    unsigned long chunk = (us > 16383UL) ? 16383UL : us;
    delayMicroseconds(chunk);  us -= chunk;
  }
}

void writePos(int angle) {
  Servo1.writeMicroseconds(map(angle, 0, 270, 500, 2500));
}

/*  Re-prime filters and quiet bands before EVERY plunge  */
void primeQuietBands() {
  uint16_t s1 = analogRead(depoSensorPin);
  uint16_t s2 = analogRead(LN2SensorPin);

  depoSensorMinNoMagnet = (s1 >  noMagMargin)       ? s1 - noMagMargin : 0;
  depoSensorMaxNoMagnet = (s1 < 1023 - noMagMargin) ? s1 + noMagMargin : 1023;
  LN2SensorMinNoMagnet  = (s2 >  noMagMargin)       ? s2 - noMagMargin : 0;
  LN2SensorMaxNoMagnet  = (s2 < 1023 - noMagMargin) ? s2 + noMagMargin : 1023;

  idx1 = idx2 = 0;   tot1 = tot2 = 0;
  for (uint8_t i = 0; i < FILT_N; ++i) {
    buf1[i] = s1;  tot1 += s1;
    buf2[i] = s2;  tot2 += s2;
  }
  depoSensorFiltered = s1;  LN2SensorFiltered = s2;
}

// ───────── timeout handler ─────────────────────────
void handleTimeout() {
  writePos(90);                      // safe park
  lcd.clear(); wait(50); 
  lcd.setCursor(0,0); lcd.print("TIMEOUT");
  lcd.setCursor(0,1); lcd.print("PRESS BTN TO RST");
  Serial.println("------TIMEOUT------");
  while (!digitalRead(buttonPin)) {} // wait for reset
}

/* ------------ micros() reset helper ----------- */
extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;
void resetMicros() {
  cli();
  //TCNT0 = 0;
  timer0_overflow_count = 0;
  timer0_millis = 0;
  sei();
}

/* ------------ setup --------------------------- */
void setup() {
  Serial.begin(115200);  delay(100);
  #if defined(analogReadResolution)
    analogReadResolution(10);
  #endif

  Servo1.attach(servoPin);
  lcd.init();  lcd.backlight();
  pinMode(buttonPin, INPUT);
  pinMode(depoSensorPin, INPUT);
  pinMode(LN2SensorPin, INPUT);

  uint16_t s1 = analogRead(depoSensorPin);
  uint16_t s2 = analogRead(LN2SensorPin);

  depoSensorMinNoMagnet = (s1 >  noMagMargin)       ? s1 - noMagMargin : 0;
  depoSensorMaxNoMagnet = (s1 < 1023 - noMagMargin) ? s1 + noMagMargin : 1023;
  LN2SensorMinNoMagnet  = (s2 >  noMagMargin)       ? s2 - noMagMargin : 0;
  LN2SensorMaxNoMagnet  = (s2 < 1023 - noMagMargin) ? s2 + noMagMargin : 1023;

  for (uint8_t i = 0; i < FILT_N; ++i) {
    buf1[i] = s1; tot1 += s1;
    buf2[i] = s2; tot2 += s2;
  }
  depoSensorFiltered = s1;
  LN2SensorFiltered  = s2;
}

/* --------------- main loop -------------------- */
void loop() {
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last < samplingTime_US) return;
  last = now;

  unsigned long reactionStart = 0, reactionEnd = 0, finalTime = 0;
  int   reactionTime  = 0;      // ms
  float finalVelocity = 0.0;    // m/s

  resetMicros();

  /* operator prompt */
  writePos(90);  wait(1000);
  primeQuietBands();
  lcd.clear();   wait(50);
  lcd.setCursor(0,0); lcd.print("Ready");
  lcd.setCursor(0,1); lcd.print("Wait: "); lcd.print(waitTime); lcd.print("ms");
  while (!digitalRead(buttonPin)) {}

  /* plunge */
  lcd.clear(); wait(50); lcd.print("Plunging");
  writePos(waitTime ? 110 : 180);

  unsigned long plungeStartUS = micros();       // timeout baseline
  bool timeoutOccurred = false;

  /* A0 – first detect */
  while (depoSensorFiltered > depoSensorMinNoMagnet && depoSensorFiltered < depoSensorMaxNoMagnet) {
    depoSensorVal = analogRead(depoSensorPin); updateMovingAvg1(depoSensorVal);
    if (micros() - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }
  reactionStart = micros();

  /* optional pause */
  if (waitTime) { wait(waitTime); writePos(180); }

  /* A1 – detect & leave */
  while (LN2SensorFiltered > LN2SensorMinNoMagnet && LN2SensorFiltered < LN2SensorMaxNoMagnet) {
    LN2SensorVal = analogRead(LN2SensorPin); updateMovingAvg2(LN2SensorVal);
    if (micros() - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }
  reactionEnd = micros();

  while (LN2SensorFiltered <= LN2SensorMinNoMagnet || LN2SensorFiltered >= LN2SensorMaxNoMagnet) {
    LN2SensorVal = analogRead(LN2SensorPin); updateMovingAvg2(LN2SensorVal);
    if (micros() - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }
  finalTime = micros();

  /* results */
  reactionTime  = (reactionEnd - reactionStart) / 1000;                   // ms
  finalVelocity = (LN2SensorThresholdDistance / float(finalTime - reactionEnd)) * 1e6; // m/s

  /* ───── NEW: error detection ─────────────────── */
  bool timeErr = reactionTime  > TIME_ERR_THRESHOLD_MS;  // NEW
  bool velErr  = finalVelocity > VEL_ERR_THRESHOLD;      // NEW

  /* ───── LCD output ───────────────────────────── */
  lcd.clear(); wait(100);
  lcd.setCursor(0,0);
  lcd.print(reactionTime); lcd.print("ms");
  if (timeErr) { lcd.print(" ERR"); }                    // NEW

  lcd.setCursor(0,1);
  lcd.print(finalVelocity, 2); lcd.print("m/s");
  if (velErr) { lcd.print(" ERR"); }                     // NEW

  /* ───── Serial output ────────────────────────── */
  Serial.print("Reaction Time: ");  Serial.print(reactionTime);   Serial.println(" ms");
  if (timeErr) Serial.println("!! HIGH TIMEPOINT ERROR");    // NEW

  Serial.print("Final Velocity: "); Serial.print(finalVelocity, 3); Serial.println(" m/s");
  if (velErr)  Serial.println("!! HIGH VELOCITY ERROR");  // NEW

  Serial.println("------Plunge Complete------");

  while (!digitalRead(buttonPin)) {}
}

/*
  DMW – 2025-06-11
  Gravity Plunger Program w/ Hall-Effect Sensors
  ---------------------------------------------------
  • Peak-time latching for accurate reactionStart /
    reactionEnd / finalTime
  • 300 ms global timeout
  • NEW: ±3-count ‘no-magnet’ thresholds are set
    automatically from the first sample of each sensor
*/

// ───────── libraries ───────────────────────────────
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ───────── constants ───────────────────────────────
const int      waitTime        = 0;          // pause (ms)
const uint8_t  depoSensorPin   = A0;
const uint8_t  LN2SensorPin    = A1;
const uint8_t  buttonPin       = A3;
const uint8_t  servoPin        = 9;
const uint16_t samplingTime_US = 500;        // 2 kSa s-¹
const float    LN2SensorThresholdDistance  = 0.022;  // m
const unsigned long plungeTimeoutUS        = 3000000; // 3000 ms

// ───────── globals ─────────────────────────────────
// Dynamic “no-magnet” windows (filled in at setup)
uint16_t depoSensorMinNoMagnet = 0;
uint16_t depoSensorMaxNoMagnet = 0;
uint16_t LN2SensorMinNoMagnet  = 0;
uint16_t LN2SensorMaxNoMagnet  = 0;

uint16_t depoSensorVal = 0, LN2SensorVal = 0;

Servo Servo1;
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ---------- 3-point moving average --------------- */
const uint8_t FILT_N = 3;
uint16_t buf1[FILT_N] = {0}, buf2[FILT_N] = {0};
uint32_t tot1 = 0,        tot2 = 0;
uint8_t  idx  = 0;
float    avg1 = 0,        avg2 = 0;

/* ---------- extremum tracking -------------------- */
uint16_t depoSensorMax = 0,   depoSensorMin = 1023;
uint16_t LN2SensorMax  = 0,   LN2SensorMin  = 1023;
uint32_t depoSensorMaxTime = 0, depoSensorMinTime = 0;
uint32_t LN2SensorMaxTime  = 0, LN2SensorMinTime  = 0;

// ───────── helper functions (unchanged) ───────────
inline void updateMovingAvg1(uint16_t s1) {
  tot1 -= buf1[idx];  buf1[idx] = s1;  tot1 += s1;
  idx = (idx + 1) % FILT_N;
  avg1 = (float)tot1 / FILT_N;
}
inline void updateMovingAvg2(uint16_t s2) {
  tot2 -= buf2[idx];  buf2[idx] = s2;  tot2 += s2;
  idx = (idx + 1) % FILT_N;
  avg2 = (float)tot2 / FILT_N;
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

// ───────── timeout handler ─────────────────────────
void handleTimeout() {
  writePos(90);                      // safe park
  lcd.clear(); wait(50); lcd.print("TIMEOUT");
  Serial.println("------TIMEOUT------");
  while (!digitalRead(buttonPin)) {} // wait for reset
}

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

// ───────── setup ───────────────────────────────────
void setup() {
  Serial.begin(115200);  delay(100);
  #if defined(analogReadResolution)
    analogReadResolution(10);
  #endif

  Servo1.attach(servoPin);
  lcd.init(); lcd.backlight();

  pinMode(buttonPin, INPUT);
  pinMode(depoSensorPin, INPUT);
  pinMode(LN2SensorPin, INPUT);

  // First sample of each sensor ── sets ‘no-magnet’ windows
  uint16_t s1 = analogRead(depoSensorPin);
  uint16_t s2 = analogRead(LN2SensorPin);

  depoSensorMinNoMagnet = (s1 > 3)    ? s1 - 3 : 0;
  depoSensorMaxNoMagnet = (s1 < 1020) ? s1 + 3 : 1023;
  LN2SensorMinNoMagnet  = (s2 > 3)    ? s2 - 3 : 0;
  LN2SensorMaxNoMagnet  = (s2 < 1020) ? s2 + 3 : 1023;

  // Prime moving-average buffers with the same samples
  for (uint8_t i = 0; i < FILT_N; ++i) {
    buf1[i] = s1; tot1 += s1;
    buf2[i] = s2; tot2 += s2;
  }
  avg1 = depoSensorMax = depoSensorMin = s1;
  avg2 = LN2SensorMax  = LN2SensorMin  = s2;

  Serial.print(F("No-magnet windows set:  A0 "));
  Serial.print(depoSensorMinNoMagnet); Serial.print("-");
  Serial.print(depoSensorMaxNoMagnet); Serial.print(",  A1 ");
  Serial.print(LN2SensorMinNoMagnet);  Serial.print("-");
  Serial.println(LN2SensorMaxNoMagnet);
}

// ───────── main loop ───────────────────────────────
void loop() {
  unsigned long reactionStart = 0, reactionEnd = 0, finalTime = 0;
  float         finalVelocity = 0.0;
  int           reactionTime  = 0;

  resetMicros();

  /* prompt operator */
  writePos(90); wait(1000);
  lcd.clear(); wait(50);
  lcd.setCursor(0,0); lcd.print("Ready");
  lcd.setCursor(0,1); lcd.print("Wait: "); lcd.print(waitTime); lcd.print("ms");
  while (!digitalRead(buttonPin)) {}            // wait for start

  /* plunge begins */
  lcd.clear(); wait(50); lcd.print("Plunging");
  writePos(waitTime ? 110 : 180);

  unsigned long plungeStartUS = micros();       // timeout baseline
  bool timeoutOccurred = false;

  /* 1) deposition sensor (A0) ---------------------- */
  while (avg1 > depoSensorMinNoMagnet && avg1 < depoSensorMaxNoMagnet) {
    depoSensorVal = analogRead(depoSensorPin); updateMovingAvg1(depoSensorVal);
    if (micros() - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }

  depoSensorMax = 0; depoSensorMaxTime = 0;
  while (avg1 <= depoSensorMinNoMagnet || avg1 >= depoSensorMaxNoMagnet) {
    depoSensorVal = analogRead(depoSensorPin); updateMovingAvg1(depoSensorVal);
    uint32_t now = micros();
    if (depoSensorVal > depoSensorMax) { depoSensorMax = depoSensorVal; depoSensorMaxTime = now; }
    if (now - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }
  reactionStart = depoSensorMaxTime ? depoSensorMaxTime : micros();

  /* optional pause */
  if (waitTime) { wait(waitTime); writePos(180); }

  /* 2) LN2 sensor (A1) ----------------------------- */
  while (avg2 > LN2SensorMinNoMagnet && avg2 < LN2SensorMaxNoMagnet) {
    LN2SensorVal = analogRead(LN2SensorPin); updateMovingAvg2(LN2SensorVal);
    if (micros() - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }

  LN2SensorMax = 0; LN2SensorMaxTime = 0;
  uint32_t lastDetectTime = micros();
  while (avg2 <= LN2SensorMinNoMagnet || avg2 >= LN2SensorMaxNoMagnet) {
    LN2SensorVal = analogRead(LN2SensorPin); updateMovingAvg2(LN2SensorVal);
    uint32_t now = micros(); lastDetectTime = now;
    if (LN2SensorVal > LN2SensorMax) { LN2SensorMax = LN2SensorVal; LN2SensorMaxTime = now; }
    if (now - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }
  reactionEnd = LN2SensorMaxTime ? LN2SensorMaxTime : lastDetectTime;
  finalTime   = lastDetectTime;                 // last sample in window

  /* results */
  reactionTime  = (reactionEnd - reactionStart) / 1000;      // ms
  finalVelocity = (LN2SensorThresholdDistance / float(finalTime - reactionEnd)) * 1e6;

  lcd.clear(); wait(100);
  lcd.setCursor(0,0); lcd.print(reactionTime); lcd.print("ms");
  lcd.setCursor(0,1); lcd.print(finalVelocity); lcd.print("m/s");

  Serial.print("Reaction Start: "); Serial.print(reactionStart); Serial.println(" us");
  Serial.print("Reaction End:   "); Serial.print(reactionEnd);   Serial.println(" us");
  Serial.print("Final Time:     "); Serial.print(finalTime);     Serial.println(" us");
  Serial.println("------Plunge Complete------");

  /* reset gate */
  while (!digitalRead(buttonPin)) {}
}

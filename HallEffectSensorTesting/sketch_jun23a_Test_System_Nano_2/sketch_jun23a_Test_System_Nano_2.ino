/*
  DMW – 2025-06-23
  Gravity Plunger Program w/ Hall-Effect Sensors
  – Dual-button start on D7 & D8
*/

// ───────── libraries ───────────────────────────
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ───────── constants ───────────────────────────
const int      waitTime        = 0;          // pause (ms)
const uint8_t  depoSensorPin   = A0;
const uint8_t  LN2SensorPin    = A1;
const uint8_t  btnPin1         = 7;          // NEW  ← D7
const uint8_t  btnPin2         = 8;          // NEW  ← D8
const uint8_t  servoPin        = 12;
const uint16_t samplingTime_US = 500;        // 2 kSa s-¹
const float    LN2SensorThresholdDistance = 0.00033;  // m
const unsigned long plungeTimeoutUS = 10000000UL; // 10 s -> future: 10 min timeout

/* ─────── error thresholds ──────────────── */
const uint16_t TIME_ERR_THRESHOLD_MS = 250;
const float    VEL_ERR_THRESHOLD     = 3.5f;

/* one-line knob for window width ────────── */
const uint8_t  noMagMargin = 3;

// ───────── dynamic “no-magnet” windows ────
uint16_t depoSensorMinNoMagnet = 0;
uint16_t depoSensorMaxNoMagnet = 0;
uint16_t LN2SensorMinNoMagnet  = 0;
uint16_t LN2SensorMaxNoMagnet  = 0;

// ───────── globals / state ────────────────
uint16_t depoSensorVal = 0, LN2SensorVal = 0;

Servo Servo1;
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ---------- 3-point moving-average state -- */
const uint8_t FILT_N = 3;
uint16_t buf1[FILT_N] = {0}, buf2[FILT_N] = {0};
uint32_t tot1 = 0,        tot2 = 0;
uint8_t  idx1 = 0,        idx2 = 0;
float  depoSensorFiltered = 0,  LN2SensorFiltered = 0;

/* ------------- helpers -------------------- */
inline void updateMovingAvg1(uint16_t s) {
  tot1 -= buf1[idx1];
  buf1[idx1] = s;  tot1 += s;
  idx1 = (idx1 + 1) % FILT_N;
  depoSensorFiltered = (float)tot1 / FILT_N;
}

inline void updateMovingAvg2(uint16_t s) {
  tot2 -= buf2[idx2];
  buf2[idx2] = s;  tot2 += s;
  idx2 = (idx2 + 1) % FILT_N;
  LN2SensorFiltered = (float)tot2 / FILT_N;
}

inline void waitMs(unsigned long ms) { while (ms--) delay(1); }
inline void writePos(int angle)      { Servo1.writeMicroseconds(map(angle, 0, 270, 500, 2500)); }

/* dual-button helper : both LOW ⇒ pressed */
inline bool buttonsPressed() {
  return digitalRead(btnPin1) == LOW && digitalRead(btnPin2) == LOW;
}

/* micros “reset” via offset ------------ */
//static unsigned long microsZero = 0;
//inline void resetMicros()        { microsZero = micros(); }
inline unsigned long microsSinceReset() { return micros();}// - microsZero; }

/* timeout handler ---------------------- */
void handleTimeout() {
  writePos(90);
  lcd.clear(); waitMs(50);
  lcd.setCursor(0,0); lcd.print("TIMEOUT");
  lcd.setCursor(0,1); lcd.print("RELEASE BTNS");
  Serial.println("------TIMEOUT------");
  /* wait until BOTH buttons are released (HIGH) */
  while (digitalRead(btnPin1)==LOW || digitalRead(btnPin2)==LOW) {}
}

/* ----------------------------------------------
   Build quiet windows from a clean baseline
   ---------------------------------------------- */
void primeQuietBands()
{
  /* helper: return the average of N good samples on a given pin */
  auto avgADC = [](uint8_t pin, uint8_t N) -> uint16_t {
    analogRead(pin);               // throw away first reading
    uint32_t sum = 0;
    for (uint8_t i = 0; i < N; ++i) sum += analogRead(pin);
    return uint16_t(sum / N);
  };
  uint16_t s1 = analogRead(depoSensorPin);   // current baseline on A0
  uint16_t s2 = analogRead(LN2SensorPin);    // current baseline on A1

  depoSensorMinNoMagnet = (s1 >  noMagMargin)       ? s1 - noMagMargin : 0;
  depoSensorMaxNoMagnet = (s1 < 1023 - noMagMargin) ? s1 + noMagMargin : 1023;
  LN2SensorMinNoMagnet  = (s2 >  noMagMargin)       ? s2 - noMagMargin : 0;
  LN2SensorMaxNoMagnet  = (s2 < 1023 - noMagMargin) ? s2 + noMagMargin : 1023;

  idx1 = idx2 = 0;
  tot1 = tot2 = 0;
  for (uint8_t i = 0; i < FILT_N; ++i) {     // preload the moving-avg buffers
    buf1[i] = s1;  tot1 += s1;
    buf2[i] = s2;  tot2 += s2;
  }
  depoSensorVal = s1;
  LN2SensorVal = s2;
  //depoSensorFiltered = s1;
  //LN2SensorFiltered  = s2;

}

/* ------------ setup --------------------- */
void setup() {
  Serial.begin(115200); delay(100);

  #if defined(analogReadResolution)
    analogReadResolution(10);
  #endif

  Servo1.attach(servoPin);
  lcd.init();  lcd.backlight();

  pinMode(btnPin1, INPUT_PULLUP);      // NEW
  pinMode(btnPin2, INPUT_PULLUP);      // NEW
  pinMode(depoSensorPin, INPUT);
  pinMode(LN2SensorPin,  INPUT);

  primeQuietBands();

  
}

/* --------------- main loop -------------- */
void loop() {

  /* sampling throttle (2 kSa/s) */
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last < samplingTime_US) return;
  last = now;

  /* per-plunge vars */
  unsigned long reactionStart = 0, reactionEnd = 0, finalTime = 0;
  long   reactionTime  = 0;
  double finalVelocity = 0.0f;

  /* ---- operator prompt ---- */
  //resetMicros();
  writePos(90);  waitMs(1000);

  primeQuietBands();
  lcd.clear(); waitMs(50);
  lcd.setCursor(0,0); lcd.print("Ready: press");
  lcd.setCursor(0,1); lcd.print("both buttons");
  while (!buttonsPressed()) {}          // ← wait for simultaneous press

  /* ---------- plunge ---------- */
  lcd.clear(); waitMs(50); lcd.print("Plunging");
  writePos(waitTime ? 110 : 180);
  waitMs(500);

  const unsigned long plungeStartUS = microsSinceReset();
  bool timeoutOccurred = false;

  //depoSensorVal = analogRead(depoSensorPin);
  //LN2SensorVal = analogRead(LN2SensorPin);


  /* A0 detect */
  while (depoSensorVal > depoSensorMinNoMagnet && depoSensorVal < depoSensorMaxNoMagnet) {
    depoSensorVal = analogRead(depoSensorPin); //updateMovingAvg1(depoSensorVal);
    if (microsSinceReset() - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }
  reactionStart = microsSinceReset();

  if (waitTime) { waitMs(waitTime); writePos(180); }

  /* A1 detect & leave */
  while (LN2SensorVal > LN2SensorMinNoMagnet &&
         LN2SensorVal < LN2SensorMaxNoMagnet) {
    LN2SensorVal = analogRead(LN2SensorPin); //updateMovingAvg2(LN2SensorVal);
    if (microsSinceReset() - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }
  reactionEnd = microsSinceReset();

  while (LN2SensorVal <= LN2SensorMinNoMagnet ||
         LN2SensorVal >= LN2SensorMaxNoMagnet) {
    LN2SensorVal = analogRead(LN2SensorPin); //updateMovingAvg2(LN2SensorVal);
    if (microsSinceReset() - plungeStartUS > plungeTimeoutUS) { timeoutOccurred = true; break; }
  }
  if (timeoutOccurred) { handleTimeout(); return; }
  finalTime = microsSinceReset();

  /* results */
  reactionTime  = (reactionEnd - reactionStart) / 1000;
  finalVelocity = (LN2SensorThresholdDistance /
                   float(finalTime - reactionEnd)) * 1e6;

  const bool timeErr = reactionTime  > TIME_ERR_THRESHOLD_MS;
  const bool velErr  = finalVelocity > VEL_ERR_THRESHOLD;

  lcd.clear(); waitMs(100);
  lcd.setCursor(0,0); lcd.print(reactionTime); lcd.print("ms");
  if (timeErr) lcd.print(" ERR");
  lcd.setCursor(0,1); lcd.print(finalVelocity,2); lcd.print("m/s");
  if (velErr) lcd.print(" ERR");

  //Serial.print("Reaction Time: ");  Serial.print(reactionTime);   Serial.println(" ms");
  //if (timeErr) Serial.println("!! HIGH TIMEPOINT ERROR");
  Serial.print("Reaction Start: ");
  Serial.print(reactionStart / 1000000.0, 4);   // 4 decimals, seconds
  Serial.println(" s");

  Serial.print("Reaction End: ");
  Serial.print(reactionEnd / 1000000.0, 4);     // 4 decimals, seconds
  Serial.println(" s");

  Serial.print("Final Time: ");
  Serial.print(finalTime / 1000000.0, 4);       // 4 decimals, seconds
  Serial.println(" s");
  //Serial.print("Final Velocity: "); Serial.print(finalVelocity,3); Serial.println(" m/s");
  //if (velErr)  Serial.println("!! HIGH VELOCITY ERROR");
  Serial.println("------Plunge Complete------");

  /* wait until buttons are pressed again */
  while (!buttonsPressed()) {}
}
//DMW - 2025/05/23
#include <Arduino.h>

/* ------------ pins & constants ---------------------------------- */
const uint8_t  S1_PIN     = A0;      // Hall sensor 1
const uint8_t  S2_PIN     = A1;      // Hall sensor 2
const uint16_t SAMPLE_US  = 200;     // 500 us -> 2 kHz (thousand samples / second)
const uint32_t WINDOW_US  = 5000000UL; // 5 s
const float    VREF       = 5.0f;    // ADC reference
const int      threshold  = 875;     // not used for stats, but keep if needed

/* ------------ 16-point moving average --------------------------- */
const uint8_t  AVG_WIN    = 16;      // power-of-two
uint16_t buf1[AVG_WIN], buf2[AVG_WIN];
uint32_t sum1 = 0, sum2 = 0;
uint8_t  idx  = 0;

/* ------------ run-time stats ------------------------------------ */
uint32_t tot1 = 0, tot2 = 0;         // sums for average
uint32_t samples = 0;

uint16_t max1 = 0, min1 = 1023, max2 = 0, min2 = 1023;
uint32_t tMax1 = 0, tMin1 = 0, tMax2 = 0, tMin2 = 0;

/* ------------ state machine ------------------------------------- */
enum State { IDLE, RUN };
State state = IDLE;
uint32_t startUs = 0;

inline float adc2V(uint16_t a) { return a * VREF / 1023.0f; }

/* ---------------------------------------------------------------- */
void setup() {
  Serial.begin(115200);
#if defined(analogReadResolution)
  analogReadResolution(10);
#endif
}

/* ---------------------------------------------------------------- */
void primeFilter() {
  uint16_t s1 = analogRead(S1_PIN);
  uint16_t s2 = analogRead(S2_PIN);
  sum1 = (uint32_t)s1 * AVG_WIN;
  sum2 = (uint32_t)s2 * AVG_WIN;
  for (uint8_t i = 0; i < AVG_WIN; ++i) { buf1[i] = s1; buf2[i] = s2; }
  idx = 0;
}

void startRun() {
  primeFilter();
  tot1 = tot2 = samples = 0;

  max1 = max2 = analogRead(S1_PIN);     // init both to current
  min1 = min2 = max1;
  tMax1 = tMin1 = tMax2 = tMin2 = micros();

  startUs = micros();
  state = RUN;
  Serial.println(F("START"));
}

void finishRun() {
  float avg1V = (float)tot1 / samples * VREF / 1023.0f;
  float avg2V = (float)tot2 / samples * VREF / 1023.0f;

  /* choose extremum farthest from average */
  float dMax1 = fabsf(adc2V(max1) - avg1V), dMin1 = fabsf(adc2V(min1) - avg1V);
  float ext1V = (dMax1 >= dMin1) ? adc2V(max1) : adc2V(min1);
  uint32_t tExt1 = (dMax1 >= dMin1) ? tMax1 : tMin1;

  float dMax2 = fabsf(adc2V(max2) - avg2V), dMin2 = fabsf(adc2V(min2) - avg2V);
  float ext2V = (dMax2 >= dMin2) ? adc2V(max2) : adc2V(min2);
  uint32_t tExt2 = (dMax2 >= dMin2) ? tMax2 : tMin2;

  int32_t  dt_us = (int32_t)tExt2 - (int32_t)tExt1;   // signed subtraction
  float    dt_s  = fabsf((float)dt_us) * 1e-3f;       // |Δt| in milliseconds


  Serial.print(avg1V, 3); Serial.print(',');
  Serial.print(ext1V, 3); Serial.print(',');
  Serial.print(avg2V, 3); Serial.print(',');
  Serial.print(ext2V, 3); Serial.print(',');
  Serial.println(dt_s, 6);

  state = IDLE;
}

/* ---------------------------------------------------------------- */
void loop() {
  /* wait for 'S' to start a run ----------------------------------- */
  if (state == IDLE && Serial.available()) {
    char c = Serial.read();
    if (c == 'S' || c == 's') startRun();
  }

  if (state != RUN) return;

  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last < SAMPLE_US) return;
  last = now;

  uint16_t r1 = analogRead(S1_PIN);
  uint16_t r2 = analogRead(S2_PIN);

  sum1 = sum1 - buf1[idx] + r1;
  sum2 = sum2 - buf2[idx] + r2;
  buf1[idx] = r1; buf2[idx] = r2;
  idx = (idx + 1) & (AVG_WIN - 1);

  uint16_t a1 = sum1 / AVG_WIN;
  uint16_t a2 = sum2 / AVG_WIN;

  tot1 += a1;  tot2 += a2;  ++samples;

  if (a1 > max1) { max1 = a1; tMax1 = now; }
  if (a1 < min1) { min1 = a1; tMin1 = now; }
  if (a2 > max2) { max2 = a2; tMax2 = now; }
  if (a2 < min2) { min2 = a2; tMin2 = now; }

  if (now - startUs >= WINDOW_US) finishRun();
}

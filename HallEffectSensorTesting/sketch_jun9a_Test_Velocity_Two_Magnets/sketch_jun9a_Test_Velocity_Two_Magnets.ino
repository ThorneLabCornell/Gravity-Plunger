//  Plunger velocity tester – pole-agnostic version
//  DMW • 2025-06-09

#include <Arduino.h>

/* ───── user-tunable parameters ─────────────────────────────────── */
const uint8_t  HALL_PIN     = A0;        // Hall sensor
const float    DIST_MM      = 12.64f;    // magnet spacing
const uint16_t SAMPLE_US    = 100;       // 10 kSa/s
const uint32_t MAX_WAIT_US  = 5000000UL; // 5 s timeout
/*  Deviation thresholds (counts away from the quiet level)         */
const uint16_t DELTA_ON     = 25;        // ≥ triggers a hit
const uint16_t DELTA_OFF    = 15;        // < clears for next hit
/* ────────────────────────────────────────────────────────────────── */

enum State { IDLE, ARMED, GOT_FIRST } state = IDLE;

uint16_t baseline = 512;       // will be re-measured each run
uint32_t tStart   = 0;         // when run armed
uint32_t tFirst   = 0;         // first-magnet timestamp
bool     hitPrev  = false;     // edge detector

/* ———— quick baseline measurement ——————————————— */
uint16_t measureBaseline(uint8_t n = 32)
{
  uint32_t sum = 0;
  for (uint8_t i = 0; i < n; ++i) {
    sum += analogRead(HALL_PIN);
    delayMicroseconds(150);    // ≈ same speed as loop() sampling
  }
  return sum / n;
}

void setup()
{
  Serial.begin(115200);
#if defined(analogReadResolution)
  analogReadResolution(10);
#endif
  pinMode(HALL_PIN, INPUT);
  Serial.println(F("Ready – press 's' to sample"));
}

void loop()
{
  /* ——— arm the test ———————————————————————————————— */
  if (state == IDLE && Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      baseline = measureBaseline();      // fresh midpoint
      state    = ARMED;
      tStart   = micros();
      hitPrev  = false;
      Serial.print(F("START  (baseline="));
      Serial.print(baseline);
      Serial.println(F(")"));
    }
  }
  if (state == IDLE) return;

  /* ——— sample sensor ——————————————————————————————— */
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last < SAMPLE_US) return;
  last = now;

  uint16_t raw = analogRead(HALL_PIN);
  int16_t  diff = (int16_t)raw - (int16_t)baseline;
  uint16_t absDiff = diff >= 0 ? diff : -diff;

  bool hitNow = (absDiff >= DELTA_ON);         // inside magnet’s field?

  /* rising edge: quiet → hit */
  if (!hitPrev && hitNow) {
    if (state == ARMED) {                      // first magnet
      tFirst = now;
      state  = GOT_FIRST;
    } else if (state == GOT_FIRST) {           // second magnet
      uint32_t dt_us = now - tFirst;
      float dt_s     = dt_us * 1e-6f;
      float v_mps    = (DIST_MM / 1000.0f) / dt_s;
      float v_mmps   =  DIST_MM / dt_s;

      Serial.print(F("Δt="));   Serial.print(dt_us);
      Serial.print(F(" µs,  v=")); Serial.print(v_mps, 3);
      Serial.print(F(" m/s ("));   Serial.print(v_mmps, 1);
      Serial.println(F(" mm/s)"));

      state = IDLE;
    }
  }

  /* hysteresis: stay ‘hit’ until deviation drops below DELTA_OFF */
  hitPrev = (absDiff >= DELTA_OFF);

  /* ——— global timeout ———————————————————————————— */
  if (state != IDLE && (now - tStart > MAX_WAIT_US)) {
    Serial.println(F("TIMEOUT"));
    state = IDLE;
  }
}

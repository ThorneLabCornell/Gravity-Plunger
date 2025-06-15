/*
  DMW – 2025-06-12
  Hall-sensor streamer
  ────────────────────────────────────────────────
  • Waits for one byte 's' or 'S' from the host.
  • Then captures 2 s of data at 500-µs intervals:
        raw1,raw2,avg1,avg2
    where avg* are 3-point moving averages.
  • Prints START … data … DONE, then loops back
    ready for another trigger.
*/

const uint8_t  S1_PIN    = A0;
const uint8_t  S2_PIN    = A1;
const uint16_t SAMPLE_US = 200;      // 2 kSa/s
const uint32_t CAPTURE_MS = 2000;    // 2-second window

/* 3-point moving-average state */
const uint8_t  FILT_N = 3;
uint16_t buf1[FILT_N] = {0}, buf2[FILT_N] = {0};
uint32_t tot1 = 0, tot2 = 0;
uint8_t  idx  = 0;
float    avg1 = 0,  avg2 = 0;

/* capture state */
bool     capturing  = false;
uint32_t capStartMs = 0;

void setup()
{
  Serial.begin(115200);
  /* prime the filter with the first reading */
  uint16_t f1 = analogRead(S1_PIN);
  uint16_t f2 = analogRead(S2_PIN);
  for (uint8_t i = 0; i < FILT_N; ++i) {
    buf1[i] = f1;  tot1 += f1;
    buf2[i] = f2;  tot2 += f2;
  }
  avg1 = f1;
  avg2 = f2;
}

inline void updateMovingAvg(uint16_t s1, uint16_t s2)
{
  tot1 -= buf1[idx];  tot2 -= buf2[idx];
  buf1[idx] = s1;     buf2[idx] = s2;
  tot1 += s1;         tot2 += s2;
  idx = (idx + 1) % FILT_N;
  avg1 = (float)tot1 / FILT_N;
  avg2 = (float)tot2 / FILT_N;
}

void loop()
{
  /* ── 1. Check for trigger byte ───────────────────────────── */
  if (!capturing && Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      capturing  = true;
      capStartMs = millis();
      //Serial.println(F("START"));
    }
  }

  /* ── 2. Fixed-rate sampling ─────────────────────────────── */
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last < SAMPLE_US) return;
  last = now;

  /* take raw readings and keep filter “warm” even when idle */
  uint16_t raw1 = analogRead(S1_PIN);
  uint16_t raw2 = analogRead(S2_PIN);
  updateMovingAvg(raw1, raw2);

  /* ── 3. During active capture, stream CSV ───────────────── */
  if (capturing) {
    Serial.print(raw1);  Serial.print(',');
    Serial.print(raw2);  Serial.print(',');
    Serial.print(avg1, 2);  Serial.print(',');
    Serial.println(avg2, 2);

    if (millis() - capStartMs >= CAPTURE_MS) {
      capturing = false;
      Serial.println(F("DONE"));
    }
  }
}

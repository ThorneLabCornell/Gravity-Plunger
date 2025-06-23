/*
  DMW – 2025-06-11
  Hall-sensor streamer — raw + 3-point moving average
  ---------------------------------------------------
  • Reads A0 and A1 every SAMPLE_US microseconds
  • Prints: <raw0>,<raw1>,<avg0>,<avg1>
*/

const uint8_t  DEPO_PIN    = A0;
const uint8_t  LN2_PIN    = A1;
const uint16_t SAMPLE_US = 500;     // 2 kSa/s

/* ---------- 3-point moving-average state ---------- */
const uint8_t  FILT_N = 3;          // window length
uint16_t buf1[FILT_N] = {0};
uint16_t buf2[FILT_N] = {0};
uint32_t tot1 = 0, tot2 = 0;        // running sums
uint8_t  idx  = 0;                  // circular index
float    avg1 = 0,  avg2 = 0;

/* ---------- absolute min / max (optional) --------- */
uint16_t absMinS1 = 1023, absMaxS1 = 0;
uint16_t absMinS2 = 1023, absMaxS2 = 0;

void setup()
{
  Serial.begin(115200);
  delay(100);                       // USB settle

  /* prime the filter with the first sample so start-up values are reasonable */
  uint16_t s1 = analogRead(DEPO_PIN);
  uint16_t s2 = analogRead(LN2_PIN);
  for (uint8_t i = 0; i < FILT_N; ++i) {
    buf1[i] = s1;  tot1 += s1;
    buf2[i] = s2;  tot2 += s2;
  }
  avg1 = s1;
  avg2 = s2;
}

/* update running 3-point average */
inline void updateMovingAvg(uint16_t s1, uint16_t s2)
{
  /* remove oldest sample from totals */
  tot1 -= buf1[idx];
  tot2 -= buf2[idx];

  /* store new samples */
  buf1[idx] = s1;
  buf2[idx] = s2;

  /* add new samples to totals */
  tot1 += s1;
  tot2 += s2;

  /* advance circular index */
  idx = (idx + 1) % FILT_N;

  /* compute current averages */
  avg1 = (float)tot1 / FILT_N;
  avg2 = (float)tot2 / FILT_N;
}

void loop()
{
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last < SAMPLE_US) return;
  last = now;

  /* raw readings */
  uint16_t raw1 = analogRead(DEPO_PIN);
  uint16_t raw2 = analogRead(LN2_PIN);

  /* update 3-point averages */
  updateMovingAvg(raw1, raw2);

  /* optional min / max tracking */
  absMinS1 = min(absMinS1, raw1);
  absMaxS1 = max(absMaxS1, raw1);
  absMinS2 = min(absMinS2, raw2);
  absMaxS2 = max(absMaxS2, raw2);

  /* CSV line: raw0,raw1,avg0,avg1 */
  Serial.print(raw1);  Serial.print(',');
  Serial.print(raw2);  Serial.print(',');
  Serial.print(avg1, 2);  Serial.print(',');   // two decimals is plenty
  Serial.println(avg2, 2);
}

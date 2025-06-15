/* 50-ms burst after 1-s delay – dual-channel, 10 kSa/s  */

const uint8_t  S1_PIN      = A0;
const uint8_t  S2_PIN      = A1;
const uint16_t SAMPLE_US   = 100;        // 100 µs → 10 kSa/s
const uint8_t  AVG_WIN     = 16;
const float    VREF        = 5.0f;

/* timing */
const uint32_t START_DELAY_MS  = 1000;   // hold-off
const uint32_t WINDOW_MS       = 50;     // capture length

/* avg buffers */
uint16_t buf0[AVG_WIN] = {0}, buf1[AVG_WIN] = {0};
uint32_t sum0 = 0,            sum1 = 0;
uint8_t  idx  = 0;

/* state */
bool      sampling     = false;
uint32_t  t_start_us   = 0;              // µs of burst start
uint32_t  lastSampleUs = 0;

void setup()                       // ────────────────────────────
{
  Serial.begin(500000);
  delay(100);                      // let USB enumerate

#if defined(analogReadResolution)
  analogReadResolution(10);        // 0-1023
#endif
  analogRead(S1_PIN);              // discard first conversions
  analogRead(S2_PIN);
}                                   // ←—————— end of setup()

void loop()                        // ────────────────────────────
{
  /* ---------- arming phase ---------- */
  if (!sampling)
  {
    if (millis() < START_DELAY_MS) return;   // still in 1-s hold-off
    sampling   = true;
    t_start_us = micros();
    lastSampleUs = t_start_us;
  }

  /* ---------- finished? ---------- */
  if (micros() - t_start_us >= WINDOW_MS * 1000UL)
    return;                              // burst complete → idle

  /* ---------- fixed-rate sampler ---------- */
  uint32_t now = micros();
  if (now - lastSampleUs < SAMPLE_US) return;
  lastSampleUs += SAMPLE_US;             // keep pace even if late

  uint16_t raw0 = analogRead(S1_PIN);
  uint16_t raw1 = analogRead(S2_PIN);

  /* update running average */
  sum0 = sum0 - buf0[idx] + raw0;
  sum1 = sum1 - buf1[idx] + raw1;
  buf0[idx] = raw0;
  buf1[idx] = raw1;
  idx = (idx + 1) & (AVG_WIN - 1);

  float raw0_V = raw0 * VREF / 1023.0f;
  float raw1_V = raw1 * VREF / 1023.0f;
  float avg0_V = (sum0 / AVG_WIN) * VREF / 1023.0f;
  float avg1_V = (sum1 / AVG_WIN) * VREF / 1023.0f;

  Serial.print(raw0_V, 3);  Serial.print(',');
  Serial.print(raw1_V, 3);  Serial.print(',');
  Serial.print(avg0_V, 3);  Serial.print(',');
  Serial.println(avg1_V, 3);
}                                   // ←—————— end of loop()

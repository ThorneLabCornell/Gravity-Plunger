/*
  DMW - 2025/06/02
  Continuous Hall-sensor voltage streamer
  ---------------------------------------
  • Reads A0 and A1 every SAMPLE_US microseconds.
  • Prints:  <count>,<raw0>,<raw1>,<avg0>,<avg1>   (volts, 3 dec)
*/

const uint8_t  S1_PIN    = A0;
const uint8_t  S2_PIN    = A1;
const uint16_t SAMPLE_US = 500;      // 500 µs → 2 kSa s⁻¹
const uint8_t  AVG_WIN   = 16;       // power-of-two window
const float    VREF      = 5.0f;     // ADC reference (Uno/Nano default)

/* ───── running-average buffers ───── */
uint16_t buf0[AVG_WIN] = {0};
uint16_t buf1[AVG_WIN] = {0};
uint32_t sum0 = 0, sum1 = 0;
uint8_t  idx  = 0;

/* global sample counter */
uint32_t sampleCnt = 0;

void setup()
{
  Serial.begin(115200);
  delay(100);                         // let USB settle

#if defined(analogReadResolution)     // e.g. SAMD, ESP32
  analogReadResolution(10);           // 0–1023
#endif

  /* throw away first bogus conversions */
  analogRead(S1_PIN);
  analogRead(S2_PIN);
}

void loop()
{
  static uint32_t lastSample = 0;
  uint32_t now = micros();
  if (now - lastSample < SAMPLE_US) return;
  lastSample = now;

  /* 1️⃣  read raw ADCs */
  uint16_t raw0 = analogRead(S1_PIN);
  uint16_t raw1 = analogRead(S2_PIN);

  /* 2️⃣  update running sums */
  sum0 = sum0 - buf0[idx] + raw0;
  sum1 = sum1 - buf1[idx] + raw1;
  buf0[idx] = raw0;
  buf1[idx] = raw1;
  idx = (idx + 1) & (AVG_WIN - 1);    // faster than %

  /* 3️⃣  compute averages */
  uint16_t avg0 = sum0 >> 4;          // divide by 16
  uint16_t avg1 = sum1 >> 4;

  /* 4️⃣  convert to volts (10-bit ADC) */
  const float scale = VREF / 1023.0f;
  float raw0_V = raw0 * scale;
  float raw1_V = raw1 * scale;
  float avg0_V = avg0 * scale;
  float avg1_V = avg1 * scale;

  /* 5️⃣  output: count,raw0,raw1,avg0,avg1 */
  Serial.print(sampleCnt++);  Serial.print(',');
  Serial.print(raw0_V, 3);    Serial.print(',');
  Serial.print(raw1_V, 3);    Serial.print(',');
  Serial.print(avg0_V, 3);    Serial.print(',');
  Serial.println(avg1_V, 3);
}

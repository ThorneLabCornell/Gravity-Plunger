

/*
  DRV5053 dual-channel – smoothed output
  • Sensors wired to A0 and A1
  • Internal sample rate: 1 kHz (1 ms cadence)
  • Console update rate: 20 Hz (every 50 ms)
*/

const uint8_t  PIN_SENS[2] = {A0, A1};
const uint32_t SAMPLE_US   = 1000;     // 1 kHz internal sampler
const uint32_t PRINT_US    = 50000;    // 20 Hz human-friendly stream
const float    FC_HZ       = 50.0;     // 50 Hz – gentle, visible smoothing

static float alpha;                    // IIR gain
static uint16_t raw[2];
static float    filt[2];

void setup()
{
  Serial.begin(115200);
  while (!Serial) ;                    // native-USB boards only

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_SAMD)
  analogReadResolution(12);            // wider ADC where available
#endif

  /* α = dt / (RCs + dt)   with RCs = 1 / (2π fc) */
  const float dt  = SAMPLE_US / 1e6;               // seconds
  const float RCs = 1.0f / (2.0f * PI * FC_HZ);
  alpha = dt / (RCs + dt);

  for (uint8_t i = 0; i < 2; ++i) {
    raw[i]  = analogRead(PIN_SENS[i]);
    filt[i] = raw[i];                              // prime filter
  }
}

void loop()
{
  static uint32_t tNextSample = micros();
  static uint32_t tNextPrint  = micros();

  /* ---------- fast sampler ---------- */
  if ((int32_t)(micros() - tNextSample) >= 0) {
    tNextSample += SAMPLE_US;
    for (uint8_t i = 0; i < 2; ++i) {
      raw[i]  = analogRead(PIN_SENS[i]);
      filt[i] = filt[i] + alpha * ((float)raw[i] - filt[i]);
    }
  }

  /* ---------- slow console stream ---------- */
  if ((int32_t)(micros() - tNextPrint) >= 0) {
    tNextPrint += PRINT_US;

    /* Format for Arduino Serial Plotter: one line, comma-separated */
    Serial.print(millis());       Serial.print(',');
    Serial.print((int)filt[0]);   Serial.print(',');
    Serial.println((int)filt[1]);
  }
}

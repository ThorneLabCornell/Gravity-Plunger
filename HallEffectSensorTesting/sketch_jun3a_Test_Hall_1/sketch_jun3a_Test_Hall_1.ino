/*
  DRV5053 dual-channel FIR smoothing
  • Sensors : A0 and A1
  • ADC     : 1 kHz internal sample rate
  • Filter  : 31-tap Hamming-windowed low-pass, fc ≈ 50 Hz
  • Output  : 20 Hz stream for Serial Plotter
*/

const uint8_t  PIN_SENS[2] = {A0, A1};
const uint32_t SAMPLE_US   = 1000;      // 1 kHz sampler
const uint32_t PRINT_US    = 5000;     // 20 Hz human-readable stream

/* ---------- FIR design (cut-off 50 Hz @ 1 kHz fs) ---------- */
const uint8_t  NTAPS = 31;
const float FIR[NTAPS] = {
  -0.0017327264, -0.0019874830, -0.0024215325, -0.0026711341, -0.0021192540,
   0.0000000000,  0.0044380569,  0.0117422605,  0.0220815504,  0.0351289228,
   0.0500324748,  0.0654894059,  0.0799158416,  0.0916844398,  0.0993861724,
   0.1020660098,  0.0993861724,  0.0916844398,  0.0799158416,  0.0654894059,
   0.0500324748,  0.0351289228,  0.0220815504,  0.0117422605,  0.0044380569,
   0.0000000000, -0.0021192540, -0.0026711341, -0.0024215325, -0.0019874830,
  -0.0017327264
};

/* ---------- per-channel circular buffers ---------- */
static float buf[2][NTAPS] = {{0}};
static uint8_t idx = 0;                 // write pointer (shared)

void setup()
{
  Serial.begin(115200);
  while (!Serial) ;                     // wait for USB on native-USB boards

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_SAMD)
  analogReadResolution(12);             // wider ADC where available
#endif
}

void loop()
{
  static uint32_t tNextSample = micros();
  static uint32_t tNextPrint  = micros();
  static float    filtVal[2]  = {0, 0};

  /* ----------- 1 kHz sampler ----------- */
  if ((int32_t)(micros() - tNextSample) >= 0) {
    tNextSample += SAMPLE_US;

    for (uint8_t ch = 0; ch < 2; ++ch) {
      buf[ch][idx] = analogRead(PIN_SENS[ch]);   // store new sample

      /* convolution: tap 0 ↔ newest sample (buf[][idx]) */
      float acc = 0;
      uint8_t tap = 0;
      uint8_t j   = idx;
      for (; tap < NTAPS; ++tap) {
        acc += FIR[tap] * buf[ch][j];
        if (j == 0) j = NTAPS;                   // wrap once
        --j;
      }
      filtVal[ch] = acc;                         // save latest output
    }

    /* advance ring-buffer pointer */
    idx++;
    if (idx == NTAPS) idx = 0;
  }

  /* ----------- 20 Hz console stream ----------- */
  if ((int32_t)(micros() - tNextPrint) >= 0) {
    tNextPrint += PRINT_US;

    /* Format: ms, filt0, filt1 — tidy in Serial Plotter */
    Serial.print(millis());  Serial.print(',');
    Serial.print((int)filtVal[0]);  Serial.print(',');
    Serial.println((int)filtVal[1]);
  }
}

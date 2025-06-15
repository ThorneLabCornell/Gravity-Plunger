/*
  DMW - 2025/06/02
  Continuous deposition Hall-sensor voltage streamer
  --------------------------------------------------------
  • Reads A0 every SAMPLE_US microseconds.
*/

const uint8_t  S1_PIN     = A0;
const uint16_t SAMPLE_US  = 500;    // 500 µs → 2 kHz (thousand samples / second)

void setup() {
  Serial.begin(115200);
  delay(100);                         // let USB settle
#if defined(analogReadResolution)
  analogReadResolution(10);           // 0–1023
#endif
  /* throw away first bogus conversions */
  analogRead(S1_PIN);
}

void loop() {
  //static uint32_t last = 0;
  //if (micros() - last < SAMPLE_US) return;   // keep the 2-kHz ADC cadence
  //last = micros();
  static uint32_t lastSample = 0;
  uint32_t now = micros();
  if (now - lastSample < SAMPLE_US) return;
  lastSample = now;
  

  uint16_t raw0 = analogRead(S1_PIN);
  // convert to volts in Python, so just send the integer:
  //Serial.print(micros());   Serial.print(',');   // 32-bit time stamp
  Serial.println(raw0);                          // ADC counts
}

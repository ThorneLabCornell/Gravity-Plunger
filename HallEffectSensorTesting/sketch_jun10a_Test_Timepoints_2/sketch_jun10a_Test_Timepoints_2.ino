// DMW - 2025/06/10
#include <Arduino.h>

/* ------------ pins & constants ---------------------------------- */
const uint8_t  S2_PIN     = A1;      // Hall sensor 2
const uint16_t SAMPLE_US  = 500;     // 500 us -> 2 kHz (thousand samples / second)
const uint32_t WINDOW_US  = 5000000UL; // 5 s
const int      threshold  = 510;     // analog threshold

/* ------------ arrays & data ------------------------------------- */
int curvePoints[25];

/* ------------ 16-point moving average --------------------------- */
const uint8_t  AVG_WIN    = 16;      // power-of-two
uint16_t buf1[AVG_WIN], buf2[AVG_WIN];
uint32_t sum1 = 0, sum2 = 0;
uint8_t  idx  = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
#if defined(analogReadResolution)
  analogReadResolution(10);
#endif
}

void startRun() {
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int rawValue = analogRead(S2_PIN);        // Read raw ADC value (0–1023)
}

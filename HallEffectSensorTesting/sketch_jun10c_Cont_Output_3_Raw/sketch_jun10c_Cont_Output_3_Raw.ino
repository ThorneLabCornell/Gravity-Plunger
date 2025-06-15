/*
  DMW - 2025/06/10
  Hall-sensor streamer — raw output
  -------------------------------------
  • Reads A0 and A1 every SAMPLE_US microseconds.
  • Prints: <count>,<raw0>,<raw1>
      where raw* are 0…1023 (10-bit ADC counts).
*/

const uint8_t  S1_PIN    = A0;
const uint8_t  S2_PIN    = A1;
const uint16_t SAMPLE_US = 1000;          // 1000 us  → 1 kSa s-¹

void setup()
{
  Serial.begin(115200);                  // baud rate
  delay(100);                            // USB settle
  analogRead(S1_PIN);                    // throw away the first bogus read
  analogRead(S2_PIN);
}

void loop()
{
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last < SAMPLE_US) return;
  last = now;

  uint16_t raw0 = analogRead(S1_PIN);
  uint16_t raw1 = analogRead(S2_PIN);

  Serial.print(raw0);       Serial.print(',');
  Serial.print(raw1);       Serial.println(' ');
}

/*
  Print averaged A1 readings every 50ms
  Arduino Nano (ATmega328P)
*/

const uint8_t HALL_PIN = A0;
const uint16_t INTERVAL_MS = 50;

void setup() {
  Serial.begin(115200);
}

void loop() {
  uint32_t startTime = millis();
  uint32_t sum = 0;
  uint16_t count = 0;

  // Collect as many samples as possible for 50ms
  while (millis() - startTime < INTERVAL_MS) {
    sum += analogRead(HALL_PIN);
    count++;
  }

  // Print the average value
  if (count > 0) {
    uint16_t avg = sum / count;
    Serial.println(avg);
  }

  // Wait for the remainder of the 50ms interval, if any
  uint32_t elapsed = millis() - startTime;
  if (elapsed < INTERVAL_MS) {
    delay(INTERVAL_MS - elapsed);
  }
}
#define ANALOG_PIN A0
volatile int count;
#define THRESHOLD_LOW 40
#define THRESHOLD_HIGH 42

void setup() {
  Serial.begin(19200);
  pinMode(ANALOG_PIN, INPUT);
}

void loop() {
  float val = analogRead(ANALOG_PIN); //read adc value (0-1023)
  updateCount(val);
  Serial.println(count);
}

void updateCount(int value) {
    static bool belowLow = true; // Track if the value was previously below THRESHOLD_LOW

    if (belowLow && value > THRESHOLD_HIGH) {
        count++;
        belowLow = false;
    } else if (!belowLow && value < THRESHOLD_LOW) {
        belowLow = true; 
    }
}

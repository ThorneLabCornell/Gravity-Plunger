/*
  Analog Hall Effect Sensor Reader
  ---------------------------------
  • Reads analog voltage from 55100-AP Hall Effect sensor
  • Converts ADC value to voltage (based on 5V reference)
  • Prints result to Serial Monitor every 200 ms
*/

// Pin Definitions
const uint8_t HALL_SENSOR_PIN = A0;    // Blue wire connected to A0
const float   VREF            = 5.0;   // Reference voltage for ADC (adjust if using 3.3V logic)

void setup() {
  Serial.begin(9600);                  // Start serial communication
  pinMode(HALL_SENSOR_PIN, INPUT);     // Set analog pin as input
}

void loop() {
  int rawValue = analogRead(HALL_SENSOR_PIN);         // Read raw ADC value (0–1023)
  float voltage = rawValue * (VREF / 1023.0);          // Convert to voltage

  Serial.print("Analog Value: ");
  Serial.print(rawValue);
  Serial.print("   Voltage: ");
  Serial.print(voltage, 3);                            // Print with 3 decimal places
  Serial.println(" V");

  delay(200);                                          // Wait 200 milliseconds
}

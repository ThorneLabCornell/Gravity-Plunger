#define ANALOG_PIN A1
void setup() {
  Serial.begin(9600);         
  pinMode(ANALOG_PIN, INPUT);  
  analogReference(DEFAULT);    // Set the default reference voltage (important for 3.3V boards)
}

void loop() {
  int rawValue = analogRead(ANALOG_PIN);
  
  Serial.println(rawValue);

}
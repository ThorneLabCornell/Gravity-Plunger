#define BUTTON_PIN A3
const int solenoid = A2; // Pin for the solenoid
const int waitTime = 1000;
void setup() {
  Serial.begin(9600);         
  pinMode(solenoid, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);  
}


void loop() {
  while(!digitalRead(BUTTON_PIN)) {
  }
  digitalWrite(solenoid, HIGH);
  delay(30);
  digitalWrite(solenoid, LOW);
  delay(waitTime);
  digitalWrite(solenoid, HIGH);
  delay(100);
  digitalWrite(solenoid, LOW);
}
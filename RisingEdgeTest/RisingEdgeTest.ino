const byte interruptPin = 2;
volatile int count = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // Initialize serial communication for printing
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), inc, RISING);
  
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(count);
}

void inc() {
  count++;
}
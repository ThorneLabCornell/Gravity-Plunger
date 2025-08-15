#include <Servo.h>

Servo myServo;
bool atOffset = false;   // track toggle state

void setup() {
  Serial.begin(115200);   // ← now 115200 baud
  myServo.attach(9);      // signal on D9 (PWM)
  myServo.write(0);       // start at 0°
  Serial.println("Servo at 0°. Send 's' to toggle.");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 's') {
      atOffset = !atOffset;
      if (atOffset) {
        myServo.write(30);    // move to +20°
        Serial.println("Moved to +30°");
      } else {
        myServo.write(0);     // back to 0°
        Serial.println("Returned to 0°");
      }
    }
  }
}

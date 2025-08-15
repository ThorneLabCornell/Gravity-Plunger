#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 9;
const int ANGLE_A = 0;      // start angle
const int ANGLE_B = 20;     // offset angle
const unsigned long MOVE_DURATION_MS = 600; // total move time per hop
const unsigned long UPDATE_PERIOD_MS = 15;  // how often to update the command

bool atOffset = false;      // toggles 0° <-> 20°
bool moving = false;

float startAngle = 0.0;
float targetAngle = 0.0;
float currentAngle = 0.0;

unsigned long moveStartMs = 0;
unsigned long lastUpdateMs = 0;

static inline float easeOutQuad(float x) {
  // x in [0,1] -> ease-out curve (fast then decelerate)
  return 1.0 - (1.0 - x) * (1.0 - x);
}

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);
  currentAngle = ANGLE_A;
  myServo.write((int)(currentAngle + 0.5));
  Serial.println("Ready. Press 's' to toggle 0° <-> 20° with deceleration.");
}

void startMove(float toAngle) {
  startAngle = currentAngle;
  targetAngle = toAngle;
  moveStartMs = millis();
  lastUpdateMs = 0;
  moving = true;
}

void loop() {
  // Handle keypress
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 's') {
      atOffset = !atOffset;
      float nextTarget = atOffset ? ANGLE_B : ANGLE_A;
      startMove(nextTarget);                 // retarget immediately from current position
      Serial.print("Moving to ");
      Serial.print(nextTarget);
      Serial.println("° with deceleration...");
    }
  }

  // Drive motion with easing
  if (moving) {
    unsigned long now = millis();
    if (now - lastUpdateMs >= UPDATE_PERIOD_MS) {
      lastUpdateMs = now;

      float t = (float)(now - moveStartMs) / (float)MOVE_DURATION_MS;
      if (t >= 1.0) {
        currentAngle = targetAngle;
        myServo.write((int)(currentAngle + 0.5));
        moving = false;
        Serial.print("Arrived at ");
        Serial.print(currentAngle);
        Serial.println("°");
      } else {
        float eased = startAngle + (targetAngle - startAngle) * easeOutQuad(t);
        currentAngle = eased;
        myServo.write((int)(currentAngle + 0.5));
      }
    }
  }
}

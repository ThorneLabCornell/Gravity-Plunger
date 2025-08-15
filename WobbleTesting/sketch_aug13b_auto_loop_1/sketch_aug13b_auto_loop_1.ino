#include <Servo.h>

// --- Timing (shared) ---
const unsigned long MOVE_DURATION_MS  = 600; // total move time per hop
const unsigned long UPDATE_PERIOD_MS  = 15;  // how often to update the command

// --- Servo 1 (D9) angles ---
const int SERVO1_PIN = 9;
const int ANGLE1_A   = 0;    // start angle
const int ANGLE1_B   = 20;   // offset angle

// --- Servo 2 (D8) angles (centered so we can go ±90) ---
const int SERVO2_PIN     = 8;
const int ANGLE2_CENTER  = 75;
const int ANGLE2_NEG90   = 0;    // 90° to one side of center
const int ANGLE2_POS90   = 165;  // 90° to the other side of center

static inline float easeOutQuad(float x) {
  // x in [0,1] -> ease-out curve (fast then decelerate)
  return 1.0 - (1.0 - x) * (1.0 - x);
}

struct EasedServo {
  Servo servo;
  int pin = -1;

  float startAngle = 0.0;
  float targetAngle = 0.0;
  float currentAngle = 0.0;

  unsigned long moveStartMs = 0;
  unsigned long lastUpdateMs = 0;

  bool moving = false;

  void attach(int p, float initialAngle) {
    pin = p;
    servo.attach(pin);
    currentAngle = initialAngle;
    servo.write((int)(currentAngle + 0.5));
    moving = false;
    lastUpdateMs = 0;
  }

  void startMove(float toAngle) {
    if (toAngle < 0) toAngle = 0;
    if (toAngle > 180) toAngle = 180;

    startAngle = currentAngle;
    targetAngle = toAngle;
    moveStartMs = millis();
    lastUpdateMs = 0;
    moving = true;
  }

  void update() {
    if (!moving) return;

    unsigned long now = millis();
    if (now - lastUpdateMs >= UPDATE_PERIOD_MS) {
      lastUpdateMs = now;

      float t = (float)(now - moveStartMs) / (float)MOVE_DURATION_MS;
      if (t >= 1.0) {
        currentAngle = targetAngle;
        servo.write((int)(currentAngle + 0.5));
        moving = false;
        Serial.print("Arrived (D");
        Serial.print(pin);
        Serial.print(") at ");
        Serial.print(currentAngle);
        Serial.println("°");
      } else {
        float eased = startAngle + (targetAngle - startAngle) * easeOutQuad(t);
        currentAngle = eased;
        servo.write((int)(currentAngle + 0.5));
      }
    }
  }
};

EasedServo s1; // D9: 0 <-> 20°
EasedServo s2; // D8: 90 <-> (0 or 180)

bool s1AtOffset = false;

static inline bool near(float a, float b, float eps = 1.0) {
  return fabs(a - b) <= eps;
}

void setup() {
  Serial.begin(115200);

  s1.attach(SERVO1_PIN, ANGLE1_A);
  s2.attach(SERVO2_PIN, ANGLE2_CENTER); // start centered at 90°

  Serial.println("Ready.");
  Serial.println("s: D9 toggle 0° <-> 20° (eased)");
  Serial.println("d: D8 toggle 90° <-> 180° (eased)");
  Serial.println("a: D8 toggle 90° <-> 0° (eased)");
}

void loop() {
  // Handle keypresses
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 's') {
      s1AtOffset = !s1AtOffset;
      float nextTarget = s1AtOffset ? ANGLE1_B : ANGLE1_A;
      s1.startMove(nextTarget);
      Serial.print("Moving (D");
      Serial.print(SERVO1_PIN);
      Serial.print(") to ");
      Serial.print(nextTarget);
      Serial.println("° with deceleration...");
    }

    // Right side (center <-> +90 => 180°)
    if (c == 'd') {
      float nextTarget = near(s2.currentAngle, ANGLE2_POS90) ? ANGLE2_CENTER : ANGLE2_POS90;
      s2.startMove(nextTarget);
      Serial.print("Moving (D");
      Serial.print(SERVO2_PIN);
      Serial.print(") to ");
      Serial.print(nextTarget);
      Serial.println("° with deceleration...");
    }

    // Left side (center <-> -90 => 0°)
    if (c == 'a') {
      float nextTarget = near(s2.currentAngle, ANGLE2_NEG90) ? ANGLE2_CENTER : ANGLE2_NEG90;
      s2.startMove(nextTarget);
      Serial.print("Moving (D");
      Serial.print(SERVO2_PIN);
      Serial.print(") to ");
      Serial.print(nextTarget);
      Serial.println("° with deceleration...");
    }
  }

  // Drive both servos (they can move simultaneously)
  s1.update();
  s2.update();
}

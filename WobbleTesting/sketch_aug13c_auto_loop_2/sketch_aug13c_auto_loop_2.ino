#include <Servo.h>
#include <math.h>

// --- Timing (shared) ---
const unsigned long MOVE_DURATION_MS = 600; // ramp time per change
const unsigned long UPDATE_PERIOD_MS = 15;  // update cadence

// --- Pulse settings (adjust for your servos) ---
const int NEUTRAL_US        = 1500; // true stop varies by servo; calibrate!
const int SERVO1_DELTA_US   = 300;  // mild speed for D9 (forward)
const int SERVO2_DELTA_US   = 600;  // stronger speed for D8 (± direction)

// --- Pins ---
const int SERVO1_PIN = 9; // D9
const int SERVO2_PIN = 8; // D8

static inline float easeOutQuad(float x) {
  // x in [0,1] -> ease-out curve (fast then decelerate)
  return 1.0 - (1.0 - x) * (1.0 - x);
}

struct EasedServoPulse {
  Servo servo;
  int pin = -1;

  int startUs = NEUTRAL_US;
  int targetUs = NEUTRAL_US;
  int currentUs = NEUTRAL_US;

  unsigned long moveStartMs = 0;
  unsigned long lastUpdateMs = 0;

  bool moving = false;

  void attach(int p, int initialUs) {
    pin = p;
    servo.attach(pin);
    currentUs = initialUs;
    targetUs = initialUs;
    servo.writeMicroseconds(currentUs);
    moving = false;
    lastUpdateMs = 0;
  }

  void startMove(int toUs) {
    if (toUs < 1000) toUs = 1000;
    if (toUs > 2000) toUs = 2000;

    startUs = currentUs;
    targetUs = toUs;
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
        currentUs = targetUs;
        servo.writeMicroseconds(currentUs);
        moving = false;
        Serial.print("Arrived (D");
        Serial.print(pin);
        Serial.print(") at ");
        Serial.print(currentUs);
        Serial.println(" us");
      } else {
        int eased = (int)(startUs + (targetUs - startUs) * easeOutQuad(t) + 0.5f);
        currentUs = eased;
        servo.writeMicroseconds(currentUs);
      }
    }
  }
};

static inline bool nearUs(int a, int b, int eps = 8) {
  return abs(a - b) <= eps;
}

EasedServoPulse s1; // D9: neutral <-> forward (mild)
EasedServoPulse s2; // D8: neutral <-> forward/reverse

bool s1AtForward = false; // toggles neutral <-> forward

void setup() {
  Serial.begin(115200);

  s1.attach(SERVO1_PIN, NEUTRAL_US);
  s2.attach(SERVO2_PIN, NEUTRAL_US);

  Serial.println("Ready (continuous rotation, pulse-width control).");
  Serial.println("s: D9 toggle NEUTRAL <-> FORWARD(mild)");
  Serial.println("d: D8 toggle NEUTRAL <-> FORWARD");
  Serial.println("a: D8 toggle NEUTRAL <-> REVERSE");
}

void loop() {
  // Handle keypresses
  if (Serial.available() > 0) {
    char c = Serial.read();

    // D9: neutral <-> mild forward
    if (c == 's') {
      s1AtForward = !s1AtForward;
      int nextUs = s1AtForward ? (NEUTRAL_US + SERVO1_DELTA_US) : NEUTRAL_US;
      s1.startMove(nextUs);
      Serial.print("Moving (D");
      Serial.print(SERVO1_PIN);
      Serial.print(") to ");
      Serial.print(nextUs);
      Serial.println(" us (eased)...");
    }

    // D8: neutral <-> forward
    if (c == 'd') {
      int forwardUs = NEUTRAL_US + SERVO2_DELTA_US;
      int nextUs = nearUs(s2.targetUs, forwardUs) ? NEUTRAL_US : forwardUs;
      s2.startMove(nextUs);
      Serial.print("Moving (D");
      Serial.print(SERVO2_PIN);
      Serial.print(") to ");
      Serial.print(nextUs);
      Serial.println(" us (eased)...");
    }

    // D8: neutral <-> reverse
    if (c == 'a') {
      int reverseUs = NEUTRAL_US - SERVO2_DELTA_US;
      int nextUs = nearUs(s2.targetUs, reverseUs) ? NEUTRAL_US : reverseUs;
      s2.startMove(nextUs);
      Serial.print("Moving (D");
      Serial.print(SERVO2_PIN);
      Serial.print(") to ");
      Serial.print(nextUs);
      Serial.println(" us (eased)...");
    }
  }

  // Drive both servos simultaneously
  s1.update();
  s2.update();
}

/*
  DMW – 2025-06-12
  Hall-sensor streamer + 3-point moving average  +  serial-triggered plunge
  ------------------------------------------------------------------------
  • Reads A0 and A1 every SAMPLE_US microseconds
  • CSV output:  <raw0>,<raw1>,<avg0>,<avg1>
  • When the PC script writes the single byte  's'
      → servo moves from parkDeg to plungeDeg,
        holds for holdMs, then returns.
  • Non-blocking state-machine: sensor streaming never pauses.
*/

#include <Arduino.h>
#include <Servo.h>

/* ───────── Hall-sensor settings ───────────────────────────────── */
const uint8_t  S1_PIN    = A0;
const uint8_t  S2_PIN    = A1;
const uint16_t SAMPLE_US = 250;          // 4 kSa/s

/* 3-point moving average */
const uint8_t  FILT_N = 3;
uint16_t buf1[FILT_N] = {0}, buf2[FILT_N] = {0};
uint32_t tot1 = 0, tot2 = 0;
uint8_t  idx  = 0;
float    avg1 = 0,  avg2 = 0;

/* optional min / max */
uint16_t absMinS1 = 1023, absMaxS1 = 0;
uint16_t absMinS2 = 1023, absMaxS2 = 0;

/* ───────── Servo-plunger settings ─────────────────────────────── */
const uint8_t  SERVO_PIN   = 9;     // PWM pin
const int      parkDeg     = 90;    // idle position
const int      plungeDeg   = 180;   // plunge position
const uint16_t servoSettle = 800;   // ms to let servo reach target
const uint16_t holdMs      = 5000;  // time plunger stays down
Servo servoArm;

/* map [0,270]° → [500,2500] µs and write ------------------------ */
inline void servoWriteDeg(int deg)
{
  servoArm.writeMicroseconds(map(constrain(deg, 0, 270), 0, 270, 500, 2500));
}

/* plunge state machine ------------------------------------------ */
enum PlungeState : uint8_t { READY, DESCEND, HOLD, ASCEND };
PlungeState plungeState = READY;
uint32_t    stateT0     = 0;        // ms timestamp for state changes

inline void startPlunge()
{
  servoWriteDeg(plungeDeg);
  plungeState = DESCEND;
  stateT0     = millis();
  Serial.println(F("START"));       // optional flag for the PC script
}

inline void updatePlungeSM()
{
  switch (plungeState)
  {
    case DESCEND:
      if (millis() - stateT0 >= servoSettle) {
        plungeState = HOLD;
        stateT0     = millis();
      }
      break;

    case HOLD:
      if (millis() - stateT0 >= holdMs) {
        servoWriteDeg(parkDeg);
        plungeState = ASCEND;
        stateT0     = millis();
      }
      break;

    case ASCEND:
      if (millis() - stateT0 >= servoSettle) {
        plungeState = READY;
        Serial.println(F("DONE"));  // signal that the cycle is finished
      }
      break;

    default: break; // READY does nothing
  }
}

/* ───────── Moving-average helper ──────────────────────────────── */
inline void updateMovingAvg(uint16_t s1, uint16_t s2)
{
  tot1 -= buf1[idx];  tot2 -= buf2[idx];
  buf1[idx] = s1;     buf2[idx] = s2;
  tot1 += s1;         tot2 += s2;
  idx   = (idx + 1) % FILT_N;
  avg1  = (float)tot1 / FILT_N;
  avg2  = (float)tot2 / FILT_N;
}

/* ───────── Setup ──────────────────────────────────────────────── */
void setup()
{
  Serial.begin(115200);
  delay(100);                          // USB settle

  // prime moving-average buffers
  uint16_t s1 = analogRead(S1_PIN);
  uint16_t s2 = analogRead(S2_PIN);
  for (uint8_t i = 0; i < FILT_N; ++i) {
    buf1[i] = s1;  tot1 += s1;
    buf2[i] = s2;  tot2 += s2;
  }
  avg1 = s1;  avg2 = s2;

  // servo
  servoArm.attach(SERVO_PIN);
  servoWriteDeg(parkDeg);
}

/* ───────── Main loop ──────────────────────────────────────────── */
void loop()
{
  /* ---------- 0.  handle incoming serial command --------------- */
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' && plungeState == READY) {
      startPlunge();
    }
    // flush any extra bytes so they don’t pile up
    while (Serial.available()) Serial.read();
  }

  /* ---------- 1.  sensor sampling every 250 µs ------------------ */
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last >= SAMPLE_US) {
    last = now;

    // raw readings
    uint16_t raw1 = analogRead(S1_PIN);
    uint16_t raw2 = analogRead(S2_PIN);

    // moving average
    updateMovingAvg(raw1, raw2);

    // min / max (optional)
    absMinS1 = min(absMinS1, raw1);   absMaxS1 = max(absMaxS1, raw1);
    absMinS2 = min(absMinS2, raw2);   absMaxS2 = max(absMaxS2, raw2);

    // CSV output
    Serial.print(raw1);  Serial.print(',');
    Serial.print(raw2);  Serial.print(',');
    Serial.print(avg1, 2);  Serial.print(',');
    Serial.println(avg2, 2);
  }

  /* ---------- 2.  update servo state machine ------------------- */
  updatePlungeSM();
}

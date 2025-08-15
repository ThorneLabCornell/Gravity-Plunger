/* ----------------------------------------------------------------
   Gravity-Plunger V2 – Nano (ATmega328P)
------------------------------------------------------------------ */

#include <Servo.h>
#include <LiquidCrystal_I2C.h>

/* ───────── pin map ───────── */
#define ENCODER_CLK 2
#define ENCODER_DT 3
#define ENCODER_SW 4
#define FAN_PIN 5
#define BUZZER_PIN 6
#define BTN1_PIN 7
#define BTN2_PIN 8
#define SERVO_PIN 12
#define DEP_SENSOR_PIN A0
#define LN2_SENSOR_PIN A1

/* ───────── constants ───────── */
const uint8_t SERVO_IDLE_DEG = 0;
const uint8_t SERVO_FIRE_DEG = 60;
const uint8_t SERVO_CONTACT_DEG = SERVO_IDLE_DEG + 15;
const uint8_t SERVO_PULSE_WINDOW_MS = 100;  // burst-PWM time

const uint8_t QUIET_MARGIN = 15;
const uint32_t TIMEOUT_US = 10000000UL;  // 10 s watchdog
const uint16_t BEEP_FREQ = 4000;         // countdown tone
const float LN2_WIDTH_M = 0.016f;        // 16 mm window

/* ───────── Jingle Bells (note ≈ 120 BPM) ───────── */
#define NOTE_E5 659
#define NOTE_G5 784
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_F5 698

const int jbMelody[] = {
  NOTE_E5, NOTE_E5, NOTE_E5,
  NOTE_E5, NOTE_E5, NOTE_E5,
  NOTE_E5, NOTE_G5, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_F5, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5,
  NOTE_E5, NOTE_D5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_G5
};
const int jbDur[] = {  // 4 = quarter, 8 = eighth …
  8, 8, 4, 8, 8, 4,
  8, 8, 8, 8, 2,
  8, 8, 8, 8, 8, 8, 8, 4, 4,
  8, 8, 8, 8, 2
};


/* ───────── Quiet-band helper ───────── */
struct QuietBand {
  uint16_t lo, hi;
  void prime(uint8_t pin) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      analogRead(pin);
      sum += analogRead(pin);
    }
    uint16_t s = sum >> 3;
    lo = (s > QUIET_MARGIN) ? s - QUIET_MARGIN : 0;
    hi = (s < 1023 - QUIET_MARGIN) ? s + QUIET_MARGIN : 1023;
  }
  bool inBand(uint16_t v) const {
    return v >= lo && v <= hi;
  }
};

void playJingleBells() {
  int notes = sizeof(jbMelody) / sizeof(jbMelody[0]);
  for (int i = 0; i < notes; ++i) {
    int dur = 2000 / jbDur[i];
    tone(BUZZER_PIN, jbMelody[i], dur * 0.9);
    delay(dur);
    noTone(BUZZER_PIN);
    delay(10);
  }
  
}


/* ───────── globals ───────── */
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo plunger;
uint8_t servoPosDeg = SERVO_IDLE_DEG;

void burstMove(uint8_t deg) {
  plunger.attach(SERVO_PIN);
  plunger.write(deg);
  delay(SERVO_PULSE_WINDOW_MS);
  plunger.detach();
  servoPosDeg = deg;
}

/* ───────── modes ───────── */
enum FireMode : uint8_t { MODE_STRAIGHT,
                          MODE_PAUSE,
                          MODE_XMAS };
FireMode fireMode = MODE_STRAIGHT;
uint32_t pauseDelayMs = 0;

/* ───────── prototypes ───────── */
int8_t readEncoder();
bool waitForEdge(uint8_t pin, const QuietBand& q, bool startIn, uint32_t t0, uint32_t to, uint32_t& tEdge);
void updateMenu(uint8_t sel);
void adjustPause(int8_t dir);
void beepCountdown();
void runPlunge();
void displayResults(float dt_ms, float vel_ms);

/* ───────── encoder ───────── */
int8_t readEncoder() {
  static bool prevCLK = HIGH;
  bool curr = digitalRead(ENCODER_CLK);
  int8_t dir = 0;
  if (prevCLK && !curr) dir = digitalRead(ENCODER_DT) ? -1 : +1;
  prevCLK = curr;
  return dir;
}

/* ───────── edge wait ───────── */
bool waitForEdge(uint8_t pin, const QuietBand& q, bool startIn, uint32_t t0, uint32_t tout, uint32_t& edge_us) {
  uint8_t ok = 0;
  while (micros() - t0 < tout) {
    bool nowIn = q.inBand(analogRead(pin));
    if (nowIn == startIn) ok = 0;
    else if (++ok >= 4) {
      edge_us = micros();
      return true;
    }
  }
  return false;
}

/* ───────── setup ───────── */
void setup() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  burstMove(SERVO_IDLE_DEG);
  updateMenu(0);
}

/* ───────── main loop ───────── */
void loop() {
  static uint8_t sel = 0;  // 0=mode 1=delay
  if (int8_t d = readEncoder()) {
    if (sel == 0) {
      fireMode = (FireMode)(((uint8_t)fireMode + (d > 0 ? 1 : 2)) % 3);  // cycle 0-1-2
    } else {
      adjustPause(-d);
    }
    updateMenu(sel);
  }

  if (!digitalRead(ENCODER_SW)) {
    while (!digitalRead(ENCODER_SW))
      ;
    sel ^= 1;
    updateMenu(sel);
  }

  if (!digitalRead(BTN1_PIN) && !digitalRead(BTN2_PIN)) {
    delay(30);
    if (!digitalRead(BTN1_PIN) && !digitalRead(BTN2_PIN)) {
      if (fireMode == MODE_XMAS) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Playing Jingle");
        lcd.setCursor(0, 1);
        lcd.print("Bells...      ");
        playJingleBells();
        noTone(BUZZER_PIN);
        updateMenu(sel);
      } else {
        runPlunge();
        updateMenu(sel);
      }
    }
  }
}

/* ───────── menu ───────── */
void updateMenu(uint8_t sel) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Mode: ");
  if (fireMode == MODE_STRAIGHT) lcd.print("Straight");
  else if (fireMode == MODE_PAUSE) lcd.print("Pause   ");
  else lcd.print("Xmas    ");
  if (sel == 0) lcd.print(" <");

  lcd.setCursor(0, 1);
  lcd.print("Delay: ");
  if (pauseDelayMs < 1000) {
    lcd.print(pauseDelayMs);
    lcd.print(" ms ");
  } else {
    lcd.print(pauseDelayMs / 1000.0f, 1);
    lcd.print(" s ");
  }
  if (sel == 1) lcd.print(" <");
}

void adjustPause(int8_t dir) {
  int32_t v = pauseDelayMs;
  int32_t step = (v < 500) ? 10 : (v < 1000) ? 25
                                             : 200;
  v += dir * step;
  if (v < 0) v = 0;
  if (v > 15000) v = 15000;
  pauseDelayMs = v;
}

/* ───────── countdown ───────── */
void beepCountdown() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Plunge Start");
  lcd.setCursor(0, 1);
  lcd.print("Keep Clear");
  for (int i = 0; i < 20; ++i) {  // 20 bursts ≈ 2 s
    tone(BUZZER_PIN, BEEP_FREQ, 30);
    delay(70);
  }
  noTone(BUZZER_PIN);
}

void lcdCountdown(const char* line1, uint32_t ms, uint8_t width, float offsetSec = 0.0f) {
  char buf[8];
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    float remain = (ms - (millis() - t0)) / 1000.0f + offsetSec;
    dtostrf(remain, width, 1, buf);
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print("T-  ");
    lcd.print(buf);
    lcd.print(" s        ");
    delay(100);
  }
}



/* ───────── plunge routine with on-screen countdown ───────── */
void runPlunge() {
  const uint32_t fanLeadMs = 3000;

  QuietBand depQ, ln2Q;
  depQ.prime(DEP_SENSOR_PIN);
  ln2Q.prime(LN2_SENSOR_PIN);

  beepCountdown();

  if (fireMode == MODE_STRAIGHT) {
    lcd.clear();
    digitalWrite(FAN_PIN, HIGH);
    lcdCountdown("Fan ON         ", fanLeadMs, 5);
    lcd.setCursor(0, 1);
    lcd.print("Plunging... ");
    burstMove(SERVO_FIRE_DEG);
  }
  else { // MODE_PAUSE
    uint32_t pauseMs = pauseDelayMs;
    // If pause is greater than the 3s fan spin
    if (pauseMs >= fanLeadMs) {
      burstMove(SERVO_CONTACT_DEG);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Contact        ");

      // Quiet part of pause
      lcdCountdown("Contact        ", pauseMs - fanLeadMs, 5, fanLeadMs / 1000.0f);

      // Fan ON for last 3s
      digitalWrite(FAN_PIN, HIGH);
      lcdCountdown("Fan ON         ", fanLeadMs, 3);
    }
    else {
      // Fan pre-spin
      digitalWrite(FAN_PIN, HIGH);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Fan ON         ");
      lcdCountdown("Fan ON         ", fanLeadMs - pauseMs, 5, pauseMs / 1000.0f);

      // Move to contact
      burstMove(SERVO_CONTACT_DEG);
      lcd.setCursor(0, 0);
      lcd.print("Contact        ");
      lcdCountdown("Contact        ", pauseMs, 3);
    }

    // Fire
    lcd.setCursor(0, 0);
    lcd.print("Plunging...    ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    burstMove(SERVO_FIRE_DEG);
  }



  /* ---- timing capture ---- */
  const uint32_t t0 = micros();
  uint32_t t_dep = 0, t_in = 0, t_out = 0;

  if (!waitForEdge(DEP_SENSOR_PIN, depQ, true, t0, TIMEOUT_US, t_dep)) {
    Serial.println("TIMEOUT DEP");
    goto finish;
  }
  Serial.println("A0 above threshold");
  if (!waitForEdge(LN2_SENSOR_PIN, ln2Q, true, t0, TIMEOUT_US, t_in)) {
    Serial.println("TIMEOUT IN");
    goto finish;
  }
  Serial.println("A1 above threshold");
  if (!waitForEdge(LN2_SENSOR_PIN, ln2Q, false, t0, TIMEOUT_US, t_out)) {
    Serial.println("TIMEOUT OUT");
    goto finish;
  }

  float dtTrav_us = float(t_in - t_dep);
  float dtWin_us = float(t_out - t_in);
  float vel = LN2_WIDTH_M / (dtWin_us * 1e-6f);
  float dtDisp_ms = dtTrav_us * 1e-3f + (fireMode == MODE_PAUSE ? pauseDelayMs : 0);

  displayResults(dtDisp_ms, vel);
  Serial.print("Time=");
  Serial.println(dtDisp_ms, 2);
  Serial.print("v=");
  Serial.println(vel, 2);

finish:
  burstMove(SERVO_IDLE_DEG);
  delay(800);  // fan coast-down
  digitalWrite(FAN_PIN, LOW);
  while (digitalRead(ENCODER_SW))
    ;  // wait release
  while (!digitalRead(ENCODER_SW))
    ;  // wait press
}




/* ───────── results ───────── */
void displayResults(float dt_ms, float vel_ms) {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (dt_ms < 1000) {
    lcd.print("t: ");
    lcd.print(dt_ms, 1);
    lcd.print(" ms ");
  } else {
    lcd.print("t: ");
    lcd.print(dt_ms / 1000.0f, 2);
    lcd.print(" s        ");
  }
  lcd.setCursor(0, 1);
  lcd.print("v: ");
  lcd.print(vel_ms, 2);
  lcd.print(" m/s");
}
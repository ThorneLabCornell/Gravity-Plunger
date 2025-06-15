
//DMW - 2025/05/21
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ─── user settings ───────────────────────────────────────────── */
const int buttonPin   = A3;     // push-button (active HIGH)
const int servoPin    = 9;      // servo PWM
const int parkDeg     = 90;     // servo angle when idle
const int plungeDeg   = 180;    // servo angle for plunge
const uint16_t servoSettle = 800;   // ms to let servo reach position
const uint16_t readyDelay  = 5000;  // ms before system is ready again
/* ─────────────────────────────────────────────────────────────── */

Servo servoArm;
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* map [0,270]° → [500,2500] µs and write ------------------------ */
inline void servoWriteDeg(int deg) {
  servoArm.writeMicroseconds(map(deg, 0, 270, 500, 2500));
}

/* show “Ready” screen ------------------------------------------- */
inline void showReady() {
  lcd.clear();
  lcd.print("Ready");
}

void setup() {
  servoArm.attach(servoPin);

  lcd.init();
  lcd.backlight();

  pinMode(buttonPin, INPUT);

  servoWriteDeg(parkDeg);
  showReady();
}

void loop() {
  /* wait for button press ---------------------------------------- */
  if (!digitalRead(buttonPin)) return;          // idle until pressed
  while (digitalRead(buttonPin)) {}             // simple debounce

  /* plunge sequence --------------------------------------------- */
  lcd.clear();
  lcd.print("Plunging...");
  servoWriteDeg(plungeDeg);
  delay(servoSettle);                           // allow motion

  /* keep plunger down for 5 s ----------------------------------- */
  delay(readyDelay);

  /* reset and show ready ---------------------------------------- */
  servoWriteDeg(parkDeg);
  showReady();
}

/*
  Nano Every – Rotary Encoder + Two Buttons + Servo + Buzzer + I²C LCD
  -------------------------------------------------------------------
  • Rotary Encoder (CLK, DT, SW) on D2, D3, D4:
      ◦ Rotate knob → moves servo on D9 to corresponding angle (0–180°).
      ◦ Press knob (SW) → displays current servo angle on the LCD.
  • Two discrete buttons on D7 and D8:
      ◦ Press **both at once** → plays a beep on a passive transducer (AT-1224) at D6.
  • I²C LCD on A4 (SDA) / A5 (SCL) shows messages upon encoder-button presses.
  • Servo signal → D9; servo powered from 5 V/GND.
  • Passive transducer (AT-1224) “+” → D6; “–” → GND. `tone()` drives it at ~2400 Hz.

  Pin assignments:
    ENCODER_CLK → D2   (quadrature A)
    ENCODER_DT  → D3   (quadrature B)
    ENCODER_SW  → D4   (pushbutton on encoder)
    BTN1        → D7   (discrete button 1)
    BTN2        → D8   (discrete button 2)
    BUZZER_PIN  → D6   (drives passive transducer with tone())
    SERVO_PIN   → D9   (servo PWM signal)
    LCD SDA     → A4
    LCD SCL     → A5
    All buttons/encoder SW use `INPUT_PULLUP` (idle = HIGH, pressed = LOW).

  Behavior summary:
    • Turning the encoder knob increments/decrements `servoPos` (clamped 0–180).
      The servo immediately moves to that angle.
    • Pressing the encoder’s pushbutton (D4) clears the LCD and prints:
         “Servo Angle:”  
         “XXX°”
    • Pressing **both** D7 and D8 simultaneously plays a 200 ms 2400 Hz tone on the AT-1224.
    • Built-in pull-ups remove need for external resistors. Debounce is handled by simple delays.

  ─────────────────────────────────────────────────────────────────────────────
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

//
// ─── PIN CONSTANTS ──────────────────────────────────────────────────────────
//
const uint8_t ENCODER_CLK  = 2;   // Quadrature A
const uint8_t ENCODER_DT   = 3;   // Quadrature B
const uint8_t ENCODER_SW   = 4;   // Encoder pushbutton
const uint8_t BTN1_PIN     = 7;   // Discrete button 1
const uint8_t BTN2_PIN     = 8;   // Discrete button 2
const uint8_t BUZZER_PIN   = 6;   // Passive transducer drive via tone()
const uint8_t SERVO_PIN    = 9;   // Servo control PWM

//
// ─── TIMING CONSTANTS ─────────────────────────────────────────────────────────
//
const uint16_t BEEP_MS     = 200; // Duration of buzzer tone
const uint16_t DEBOUNCE_MS = 250; // Debounce delay for buttons

//
// ─── GLOBAL OBJECTS & VARIABLES ──────────────────────────────────────────────
//
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adjust address to 0x3F if needed
Servo myServo;

volatile int servoPos       = 0;    // Current servo position (0–180°)
int lastEncoderCLKState     = HIGH; // Remember the last CLK state
bool encoderSwPressed       = false;

//
// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  // Serial.begin(115200); // Uncomment if you want debug prints

  // Configure encoder pins
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);
  pinMode(ENCODER_SW,  INPUT_PULLUP);

  // Configure discrete buttons
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);

  // Configure buzzer pin (tone output)
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Ensure no tone at startup

  // Attach servo and set initial position to 0°
  myServo.attach(SERVO_PIN);
  myServo.write(servoPos);

  // Initialize I²C LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Encoder + Buttons");
  lcd.setCursor(0, 1);
  lcd.print("Servo Demo");
}

//
// ─── LOOP ────────────────────────────────────────────────────────────────────
void loop() {
  handleEncoderRotation();   // Read quadrature, update servoPos
  handleEncoderButton();     // If encoder SW pressed → display angle
  handleDualButtons();       // If both BTN1 & BTN2 pressed → beep
}

//
// ─── HANDLE ROTARY ENCODER ROTATION ───────────────────────────────────────────
// Reads CLK pin; on a change, checks DT to determine rotation direction.
// Increments/decrements servoPos by 1°, clamps 0–180, and moves servo.
//
void handleEncoderRotation() {
  int currentCLK = digitalRead(ENCODER_CLK);

  // Detect a change (rising or falling) on CLK
  if (currentCLK != lastEncoderCLKState) {
    // Determine direction via relative state of DT
    // If DT != currentCLK, we turned clockwise; else counterclockwise.
    if (digitalRead(ENCODER_DT) != currentCLK) {
      // Clockwise → increase position
      servoPos++;
      if (servoPos > 180) {
        servoPos = 180;
      }
    } else {
      // Counterclockwise → decrease position
      servoPos--;
      if (servoPos < 0) {
        servoPos = 0;
      }
    }
    myServo.write(servoPos);  // Immediately move servo to new angle
    // Optionally uncomment below to debug:
    // Serial.print("servoPos = "); Serial.println(servoPos);
  }

  lastEncoderCLKState = currentCLK;
}

//
// ─── HANDLE ENCODER PUSHBUTTON ─────────────────────────────────────────────────
// When pressed, prints the current servoPos on the LCD. Debounces with a delay.
//
void handleEncoderButton() {
  if (digitalRead(ENCODER_SW) == LOW && !encoderSwPressed) {
    encoderSwPressed = true;

    // Clear LCD and print servo angle
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Servo Angle:");
    lcd.setCursor(0, 1);
    lcd.print(servoPos);
    lcd.print(" deg");

    delay(DEBOUNCE_MS);          // crude debounce
  }
  // Wait for release
  if (digitalRead(ENCODER_SW) == HIGH) {
    encoderSwPressed = false;
  }
}

//
// ─── HANDLE DUAL BUTTON PRESS FOR BUZZER ──────────────────────────────────────
// If both BTN1_PIN and BTN2_PIN are held LOW simultaneously → play tone.
//
void handleDualButtons() {
  if (digitalRead(BTN1_PIN) == LOW && digitalRead(BTN2_PIN) == LOW) {
    // Play a 2.4 kHz tone for BEEP_MS
    tone(BUZZER_PIN, 2400);
    delay(BEEP_MS);
    noTone(BUZZER_PIN);

    // Debounce delay so you have time to release both buttons
    delay(DEBOUNCE_MS);
  }
}

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int photoInterrupter1 = A0; // Pin for photo interrupter 1
const int photoInterrupter2 = A1; // Pin for photo interrupter 2
unsigned long time1 = 0;          // Time when interrupter 1 was activated
unsigned long time2 = 0;          // Time when interrupter 2 was activated
boolean activated1 = false;       // Flag to indicate if interrupter 1 was activated
boolean activated2 = false;       // Flag to indicate if interrupter 2 was activated

LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
}

void loop() {
  int sensorValue1 = analogRead(photoInterrupter1);
  int sensorValue2 = analogRead(photoInterrupter2);

  // Check if interrupter 1 is activated
  if (sensorValue1 < 100 && !activated1) {
    time1 = millis(); // Record the time when interrupter 1 is activated
    activated1 = true; // Set flag to indicate interrupter 1 is activated
  }
  
  // Check if interrupter 2 is activated
  if (sensorValue2 < 100 && !activated2) {
    time2 = millis(); // Record the time when interrupter 2 is activated
    activated2 = true; // Set flag to indicate interrupter 2 is activated
  }
  
  // If both interrupters are activated, calculate time difference and print once
  if (activated1 && activated2) {
    unsigned long timeDifference = time2 - time1; // Calculate time difference
    
    // Filter out time differences that are too large
    if (timeDifference > 10 && timeDifference < 10000) {
      lcd.clear(); // Clear the LCD screen
      lcd.setCursor(0, 0); // Set cursor to the first column of the first row
      lcd.print("Time difference:"); // Display label
      lcd.setCursor(0, 1); // Set cursor to the first column of the second row
      lcd.print(timeDifference); // Display time difference on the LCD
      lcd.print(" ms");
      delay(2000); // Delay for 2 seconds to display the result before clearing the screen
    }
    
    // Reset flags and time points for next calculation
    activated1 = false;
    activated2 = false;
    time1 = 0;
    time2 = 0;
  }
}

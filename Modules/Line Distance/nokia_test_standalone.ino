#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Nokia 5110 (PCD8544) wiring (hardware SPI lines + 3 control pins)
// ESP32 SCK=18  -> 5110 Clk
// ESP32 MOSI=23 -> 5110 Dir
// Control pins re-used from the previous 1602 wiring:
// GPIO14 -> DC
// GPIO27 -> CE
// GPIO26 -> RST
static const int LCD5110_SCLK = 18;
static const int LCD5110_DIN  = 23;
static const int LCD5110_DC   = 14;
static const int LCD5110_CS   = 27;
static const int LCD5110_RST  = 26;

Adafruit_PCD8544 display(LCD5110_SCLK, LCD5110_DIN, LCD5110_DC, LCD5110_CS, LCD5110_RST);

static void drawTestScreen(int counter) {
  display.clearDisplay();
  display.setTextColor(BLACK);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Nokia 5110 OK");

  display.setCursor(0, 10);
  display.print("Count: ");
  display.print(counter);

  display.setCursor(0, 20);
  display.print("ASCII:");
  display.setCursor(0, 30);
  display.print("0123456789");

  // simple bar
  int barW = (counter % 85);
  display.drawRect(0, 40, 84, 7, BLACK);
  display.fillRect(1, 41, barW > 82 ? 82 : barW, 5, BLACK);

  display.display();
}

void setup() {
  Serial.begin(115200);

  display.begin();
  display.setContrast(55);   // adjust 40-70 for contrast
  display.clearDisplay();
  display.display();

  Serial.println("Nokia 5110 test: updating screen once per second.");
}

void loop() {
  static int counter = 0;

  drawTestScreen(counter);
  counter++;

  delay(1000);
}
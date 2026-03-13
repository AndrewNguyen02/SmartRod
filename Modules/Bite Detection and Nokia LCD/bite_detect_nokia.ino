#include <Arduino.h>
#include <math.h>
#include "Adafruit_BNO08x_RVC.h"

// Nokia 5110 (PCD8544)
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// ==================== State Machine ====================
// ARMED    : IMU enabled; detect cast start
// CASTING  : IMU enabled; track peak force for power bar; piezo ignored
// WAIT_BITE: IMU disabled; piezo enabled; wait for bite event
enum RodState : uint8_t { ARMED = 0, CASTING = 1, WAIT_BITE = 2 };
RodState state = ARMED;

// -------------------- Hardware Pins --------------------

// BNO085 UART-RVC (RVC output -> ESP32 RX2)
static const int BNO_UART_RX = 19;
static const int BNO_UART_TX = -1;
static const uint32_t BNO_UART_BAUD = 115200;

HardwareSerial BNO_Serial(2);
Adafruit_BNO08x_RVC rvc;

// Nokia 5110 pins
// CLK  -> GPIO18 (SCK)
// DIN  -> GPIO23 (MOSI)
// DC   -> GPIO14
// CS   -> GPIO33
// RST  -> GPIO26
static const int LCD5110_SCLK = 18;
static const int LCD5110_DIN  = 23;
static const int LCD5110_DC   = 14;
static const int LCD5110_CS   = 33;
static const int LCD5110_RST  = 26;

Adafruit_PCD8544 display(LCD5110_SCLK, LCD5110_DIN, LCD5110_DC, LCD5110_CS, LCD5110_RST);

// -------------------- Timing --------------------
const unsigned long lcdMs = 120;
unsigned long lastLcdMs = 0;

// -------------------- IMU Power Meter --------------------
const float MASS_KG = 0.05f;
const float MAX_FORCE_EXPECTED = 1.0f;

static float amagLP = 0.0f;
const float LP_ALPHA = 0.995f;

float forceNewtons = 0.0f;
float forceHold = 0.0f;
bool  forceUpdated = false;

const unsigned long HOLD_MS = 3000;
unsigned long holdUntilMs = 0;

const float HOLD_DECAY_PER_UPDATE = 0.96f;
const float HOLD_FLOOR = 0.01f;

float imuDynAccel = 0.0f;

// -------------------- Piezo Bite Detection --------------------
static const int PIEZO_PIN = 34;

int piezoBaseline = 0;
float piezoAlpha = 0.02f;
int piezoSpike = 0;

int BITE_THRESH[3] = { 250, 350, 500 };  // HIGH, MED, LOW (ADC counts)
uint8_t sensIdx = 1;                     // default MED

unsigned long biteLockoutUntil = 0;
const unsigned long BITE_LOCKOUT_MS = 600;

unsigned long biteBannerUntil = 0;
const unsigned long BITE_BANNER_MS = 5000;

// -------------------- Cast Detection Thresholds --------------------
const float CAST_START_DYN = 1.20f;
const float CAST_END_DYN   = 0.35f;
const unsigned long CAST_END_QUIET_MS = 450;
const unsigned long CAST_CONFIRM_MS = 40;

unsigned long castCandidateStartMs = 0;
unsigned long castQuietStartMs = 0;

// -------------------- Post-cast Settling --------------------
unsigned long biteEnableAtMs = 0;
const unsigned long POST_CAST_SETTLE_MS = 900;

// ==================== State Entry Helpers ====================
static void enterArmed(unsigned long now) {
  state = ARMED;
  castCandidateStartMs = 0;
  castQuietStartMs = 0;

  biteBannerUntil = 0;
  biteLockoutUntil = 0;

  biteEnableAtMs = now + 300;
}

static void enterCasting(unsigned long now) {
  state = CASTING;
  castCandidateStartMs = 0;
  castQuietStartMs = 0;

  forceNewtons = 0.0f;
  forceHold = 0.0f;
  forceUpdated = false;
  holdUntilMs = 0;

  biteEnableAtMs = now + 9999999UL;
}

static void enterWaitBite(unsigned long now) {
  state = WAIT_BITE;
  castCandidateStartMs = 0;
  castQuietStartMs = 0;

  biteEnableAtMs = now + POST_CAST_SETTLE_MS;
}

// ==================== State Machine Update ====================
static void updateStateMachine(unsigned long now) {
  if (state == ARMED) {
    if (imuDynAccel >= CAST_START_DYN) {
      if (castCandidateStartMs == 0) castCandidateStartMs = now;
      if (now - castCandidateStartMs >= CAST_CONFIRM_MS) {
        enterCasting(now);
      }
    } else {
      castCandidateStartMs = 0;
    }
    return;
  }

  if (state == CASTING) {
    if (imuDynAccel <= CAST_END_DYN) {
      if (castQuietStartMs == 0) castQuietStartMs = now;
      if (now - castQuietStartMs >= CAST_END_QUIET_MS) {
        enterWaitBite(now);
      }
    } else {
      castQuietStartMs = 0;
    }
    return;
  }

  // WAIT_BITE: remains active until manual re-arm/reset
}

// -------------------- Nokia UI --------------------
static const char* stateLabel(RodState st) {
  switch (st) {
    case ARMED:     return "ARM";
    case CASTING:   return "CAST";
    case WAIT_BITE: return "WAIT";
    default:        return "?";
  }
}

static void drawNokiaUI(float forceHeldN, RodState st, bool biteBanner) {
  display.clearDisplay();
  display.setTextColor(BLACK);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("SmartRod ");
  display.print(biteBanner ? "BITE!" : stateLabel(st));

  display.setCursor(0, 10);
  display.print("PWR:");

  float clamped = forceHeldN;
  if (clamped < 0.0f) clamped = 0.0f;
  if (clamped > MAX_FORCE_EXPECTED) clamped = MAX_FORCE_EXPECTED;

  const int barX = 0;
  const int barY = 22;
  const int barW = 84;
  const int barH = 10;

  display.drawRect(barX, barY, barW, barH, BLACK);

  int fillW = (int)lroundf((clamped / MAX_FORCE_EXPECTED) * (float)(barW - 2));
  if (fillW < 0) fillW = 0;
  if (fillW > (barW - 2)) fillW = (barW - 2);
  if (fillW > 0) display.fillRect(barX + 1, barY + 1, fillW, barH - 2, BLACK);

  display.setCursor(0, 36);
  display.print(forceHeldN, 2);
  display.print(" N");

  display.display();
}

// -------------------- Reset --------------------
static void resetModel() {
  forceNewtons = 0.0f;
  forceHold = 0.0f;
  imuDynAccel = 0.0f;
  amagLP = 0.0f;
  forceUpdated = false;
  holdUntilMs = 0;

  piezoBaseline = 0;
  piezoSpike = 0;
  biteLockoutUntil = 0;
  biteBannerUntil = 0;

  enterArmed(millis());

  display.clearDisplay();
  display.setTextColor(BLACK);
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("Reset");
  display.setCursor(0, 10); display.print("...");
  display.display();
}

void setup() {
  Serial.begin(115200);

  // Display init
  display.begin();
  display.setContrast(55);
  display.setRotation(2);   // 180° flip
  display.clearDisplay();
  display.setTextColor(BLACK);
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("Smart Rod");
  display.setCursor(0, 10); display.print("Init...");
  display.display();

  // IMU init
  BNO_Serial.begin(BNO_UART_BAUD, SERIAL_8N1, BNO_UART_RX, BNO_UART_TX);
  delay(1200);

  if (!rvc.begin(&BNO_Serial)) {
    Serial.println("ERROR: BNO085 UART-RVC init failed.");
    display.clearDisplay();
    display.setCursor(0, 0);  display.print("BNO085 UART");
    display.setCursor(0, 10); display.print("INIT FAIL");
    display.display();
    while (true) delay(100);
  }

  // ADC init
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Baseline priming
  delay(200);
  int sum = 0;
  for (int i = 0; i < 50; i++) { sum += analogRead(PIEZO_PIN); delay(2); }
  piezoBaseline = sum / 50;

  enterArmed(millis());

  display.clearDisplay();
  display.setCursor(0, 0);  display.print("Ready");
  display.setCursor(0, 10); display.print("ARM to cast");
  display.display();
  delay(400);

  Serial.println("Prototype v2: IMU cast power + piezo bite; states ARM/CAST/WAIT.");
  Serial.println("Serial: r=reset, a=arm, 1/2/3=sens(H/M/L).");
}

void loop() {
  unsigned long now = millis();

  // -------------------- IMU Read (disabled in WAIT_BITE) --------------------
  if (state != WAIT_BITE) {
    BNO08x_RVC_Data data;
    bool gotPacket = false;

    while (rvc.read(&data)) { gotPacket = true; }

    if (gotPacket) {
      float ax = data.x_accel;
      float ay = data.y_accel;
      float az = data.z_accel;

      float amag = sqrtf(ax * ax + ay * ay + az * az);

      if (amagLP == 0.0f) amagLP = amag;
      amagLP = LP_ALPHA * amagLP + (1.0f - LP_ALPHA) * amag;

      imuDynAccel = fabsf(amag - amagLP);

      forceNewtons = MASS_KG * imuDynAccel;
      forceUpdated = true;
    }
  }

  // -------------------- State Machine --------------------
  updateStateMachine(now);

  // -------------------- Piezo Read + Bite Detect --------------------
  {
    int v = analogRead(PIEZO_PIN);

    if (piezoBaseline == 0) piezoBaseline = v;
    piezoBaseline = (int)((1.0f - piezoAlpha) * piezoBaseline + piezoAlpha * v);
    piezoSpike = abs(v - piezoBaseline);

    if (state == WAIT_BITE && now >= biteEnableAtMs && now >= biteLockoutUntil) {
      if (piezoSpike > BITE_THRESH[sensIdx]) {
        biteBannerUntil = now + BITE_BANNER_MS;
        biteLockoutUntil = now + BITE_LOCKOUT_MS;
        Serial.println("BITE!");
      }
    }
  }

  // -------------------- Peak Hold / Decay --------------------
  if (forceUpdated) {
    if (state == CASTING) {
      if (forceNewtons > forceHold) {
        forceHold = forceNewtons;
        holdUntilMs = now + HOLD_MS;
      }
    }
    forceUpdated = false;
  }

  if (now >= holdUntilMs) {
    forceHold *= HOLD_DECAY_PER_UPDATE;
    if (forceHold < HOLD_FLOOR) forceHold = 0.0f;
  }

  // -------------------- Display --------------------
  if (now - lastLcdMs >= lcdMs) {
    lastLcdMs = now;
    bool biteBanner = (now < biteBannerUntil);
    drawNokiaUI(forceHold, state, biteBanner);
  }

  // -------------------- Serial Commands --------------------
  if (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == 'r' || ch == 'R') {
      resetModel();
      Serial.println("Reset.");
    } else if (ch == 'a' || ch == 'A') {
      enterArmed(millis());
      Serial.println("Re-armed.");
    } else if (ch == '1') {
      sensIdx = 0;
      Serial.println("Piezo sensitivity: HIGH");
    } else if (ch == '2') {
      sensIdx = 1;
      Serial.println("Piezo sensitivity: MED");
    } else if (ch == '3') {
      sensIdx = 2;
      Serial.println("Piezo sensitivity: LOW");
    }
  }
}

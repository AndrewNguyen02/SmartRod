#include <Arduino.h>
#include <math.h>
#include "Adafruit_BNO08x_RVC.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include "BluetoothSerial.h" // ADDED: Bluetooth Library

// ==================== Bluetooth Config ====================
BluetoothSerial SerialBT; // ADDED: Bluetooth Object
unsigned long lastBtTxMs = 0;
const unsigned long BT_TX_INTERVAL = 250; // Transmit every 250ms

// ==================== State Machine ====================
// ARMED    : IMU active, waiting for cast onset
// CASTING  : IMU active, peak cast force tracked for power bar
// WAIT_BITE: IMU ignored, piezo monitored for bite detection
enum RodState : uint8_t { ARMED = 0, CASTING = 1, WAIT_BITE = 2 };
RodState state = ARMED;

// -------------------- Hardware Pins --------------------

// BNO085 UART-RVC interface
static const int BNO_UART_RX = 19;
static const int BNO_UART_TX = -1;
static const uint32_t BNO_UART_BAUD = 115200;

HardwareSerial BNO_Serial(2);
Adafruit_BNO08x_RVC rvc;

// Hall-effect line counter
static const int HALL_PIN = 27;

// Piezo bite sensor
static const int PIEZO_PIN = 34;

// Active buzzer output
static const int BUZZER_PIN = 25;

// ESP32 DevKit onboard BOOT button (active LOW)
static const int BUTTON_PIN = 0;

// Nokia 5110 (PCD8544)
static const int LCD5110_SCLK = 18;
static const int LCD5110_DIN  = 23;
static const int LCD5110_DC   = 14;
static const int LCD5110_CS   = 33;
static const int LCD5110_RST  = 26;

Adafruit_PCD8544 display(LCD5110_SCLK, LCD5110_DIN, LCD5110_DC, LCD5110_CS, LCD5110_RST);

// -------------------- Timing --------------------

const unsigned long lcdMs = 120;
unsigned long lastLcdMs = 0;

// -------------------- Hall Distance --------------------

const float R_CORE        = 0.02413f;
const float R_FULL        = 0.03048f;
const float AVG_RADIUS    = (R_CORE + R_FULL) / 2.0f;
const float CIRCUMFERENCE = TWO_PI * AVG_RADIUS;

const float MAGNETS_PER_REVOLUTION = 2.0f;
const float PULSES_TO_REVOLUTIONS  = 1.0f / MAGNETS_PER_REVOLUTION;

const float CALIBRATION_FACTOR = 1.00f;
const float METERS_TO_FEET     = 3.28084f;

volatile int pulseCount = 0;
volatile unsigned long lastTriggerTime = 0;

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

int piezoBaseline = 0;
float piezoAlpha = 0.02f;
int piezoSpike = 0;
static const bool AUTO_REARM_AFTER_BITE = true; // dev toggle: allows auto reset to ARM state after bite detected (for demo purposes)

// HIGH, MED, LOW sensitivity thresholds
int BITE_THRESH[3] = { 500, 1500, 3000 };
uint8_t sensIdx = 1;

unsigned long biteLockoutUntil = 0;
const unsigned long BITE_LOCKOUT_MS = 600;

unsigned long biteBannerUntil = 0;
const unsigned long BITE_BANNER_MS = 5000;

// -------------------- Button / Sensitivity Toggle --------------------

bool lastButtonReading = HIGH;
bool buttonStableState = HIGH;
unsigned long lastButtonEdgeMs = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 40;

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

// -------------------- Helpers --------------------

static const char* stateLabel(RodState st, bool biteBanner) {
  if (biteBanner) return "BITE";

  switch (st) {
    case ARMED:     return "ARM";
    case CASTING:   return "CAST";
    case WAIT_BITE: return "WAIT";
    default:        return "?";
  }
}

static const char* sensLabel(uint8_t idx) {
  switch (idx) {
    case 0: return "HIGH";
    case 1: return "MED";
    case 2: return "LOW";
    default: return "?";
  }
}

// Hall pulse ISR with debounce.
void IRAM_ATTR hall_ISR() {
  unsigned long currentTime = millis();

  if (currentTime - lastTriggerTime > 200) {
    pulseCount++;
    lastTriggerTime = currentTime;
  }
}

// Active buzzer drive helpers.
static void buzzerOff() {
  digitalWrite(BUZZER_PIN, LOW);
}

static void buzzerOn() {
  digitalWrite(BUZZER_PIN, HIGH);
}

static void buzzerPulse(unsigned long onMs, unsigned long offMs) {
  buzzerOn();
  delay(onMs);
  buzzerOff();
  delay(offMs);
}

// Short blocking alert sequence used only on bite events.
static void playBiteAlert() {
  buzzerPulse(120, 80);
  buzzerPulse(120, 80);
  buzzerPulse(180, 0);
}

static void cycleSensitivity() {
  sensIdx++;
  if (sensIdx > 2) sensIdx = 0;

  Serial.print("Piezo sensitivity: ");
  Serial.println(sensLabel(sensIdx));
}

static void updateButton(unsigned long now) {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastButtonEdgeMs = now;
    lastButtonReading = reading;
  }

  if ((now - lastButtonEdgeMs) >= BUTTON_DEBOUNCE_MS) {
    if (reading != buttonStableState) {
      buttonStableState = reading;

      // Trigger on button press (HIGH -> LOW)
      if (buttonStableState == LOW) {
        cycleSensitivity();
      }
    }
  }
}

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

  // WAIT_BITE persists until explicit re-arm/reset.
}

// -------------------- Nokia UI --------------------

static void drawNokiaUI(float distance_m, float distance_ft, float forceHeldN,
                        RodState st, bool biteBanner, uint8_t sensitivity) {
  display.clearDisplay();
  display.setTextColor(BLACK);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("State: ");
  display.print(stateLabel(st, biteBanner));

  display.setCursor(0, 10);
  display.print("Line:");
  display.print(distance_m, 1);
  display.print("m");

  display.setCursor(0, 20);
  display.print(distance_ft, 1);
  display.print("ft");

  display.setCursor(0, 30);
  display.print("Sens:");
  display.print(sensLabel(sensitivity));

  float clamped = forceHeldN;
  if (clamped < 0.0f) clamped = 0.0f;
  if (clamped > MAX_FORCE_EXPECTED) clamped = MAX_FORCE_EXPECTED;

  const int barX = 0;
  const int barY = 40;
  const int barW = 84;
  const int barH = 8;

  display.drawRect(barX, barY, barW, barH, BLACK);

  int fillW = (int)lroundf((clamped / MAX_FORCE_EXPECTED) * (float)(barW - 2));
  if (fillW < 0) fillW = 0;
  if (fillW > (barW - 2)) fillW = (barW - 2);

  if (fillW > 0) {
    display.fillRect(barX + 1, barY + 1, fillW, barH - 2, BLACK);
  }

  display.display();
}

// -------------------- Reset --------------------

static void resetModel() {
  noInterrupts();
  pulseCount = 0;
  lastTriggerTime = 0;
  interrupts();

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

  buzzerOff();
  enterArmed(millis());

  display.clearDisplay();
  display.setTextColor(BLACK);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Reset");
  display.display();
}

void setup() {
  Serial.begin(115200);
  
  // ADDED: Bluetooth Init
  SerialBT.begin("CyberFish_Rod"); 

  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hall_ISR, FALLING);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  display.begin();
  display.setContrast(55);
  display.setRotation(2);
  display.clearDisplay();
  display.display();

  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();

  BNO_Serial.begin(BNO_UART_BAUD, SERIAL_8N1, BNO_UART_RX, BNO_UART_TX);
  delay(1200);

  if (!rvc.begin(&BNO_Serial)) {
    Serial.println("ERROR: BNO085 UART-RVC init failed.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("BNO085 FAIL");
    display.display();

    while (true) delay(100);
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  delay(200);
  int sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(PIEZO_PIN);
    delay(2);
  }
  piezoBaseline = sum / 50;

  enterArmed(millis());
}

void loop() {
  unsigned long now = millis();

  // -------------------- Button Update --------------------
  updateButton(now);

  // -------------------- IMU Read --------------------
  if (state != WAIT_BITE) {
    BNO08x_RVC_Data data;
    bool gotPacket = false;

    while (rvc.read(&data)) {
      gotPacket = true;
    }

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
        unsigned long bannerUntil = now + BITE_BANNER_MS;
        unsigned long lockoutUntil = now + BITE_LOCKOUT_MS;

        Serial.println("BITE!");
        playBiteAlert();

        biteBannerUntil = bannerUntil;
        biteLockoutUntil = lockoutUntil;

        if (AUTO_REARM_AFTER_BITE) {
          enterArmed(millis());
          biteBannerUntil = bannerUntil;
          biteLockoutUntil = lockoutUntil;
        }
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

  // -------------------- Distance --------------------
  int localPulseCount;
  noInterrupts();
  localPulseCount = pulseCount;
  interrupts();

  float revolutions = localPulseCount * PULSES_TO_REVOLUTIONS;
  float distance_m  = revolutions * CIRCUMFERENCE * CALIBRATION_FACTOR;
  float distance_ft = distance_m * METERS_TO_FEET;

  // -------------------- Display --------------------
  if (now - lastLcdMs >= lcdMs) {
    lastLcdMs = now;
    bool biteBanner = (now < biteBannerUntil);
    drawNokiaUI(distance_m, distance_ft, forceHold, state, biteBanner, sensIdx);
  }

  // ADDED: Bluetooth Transmission Block
  if (now - lastBtTxMs >= BT_TX_INTERVAL) {
    lastBtTxMs = now;
    if (SerialBT.hasClient()) {
      SerialBT.print(distance_m, 2);
      SerialBT.print(",");
      SerialBT.print(forceHold, 2);
      SerialBT.print(",");
      SerialBT.println(stateLabel(state, (now < biteBannerUntil)));
    }
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

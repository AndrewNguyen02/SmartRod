#include <Arduino.h>

// Define the pin connected to the US5881 OUT pin
#define HALL_PIN 27

// Volatile variables because they change inside the Interrupt Service Routine (ISR)
volatile int pulseCount = 0;
volatile unsigned long lastTriggerTime = 0;

// The Interrupt function: Runs immediately when the magnet is detected
void IRAM_ATTR hall_ISR() {
  unsigned long currentTime = millis();
  
  // 200ms debounce for manual hand-testing
  if (currentTime - lastTriggerTime > 200) {
    pulseCount++;
    lastTriggerTime = currentTime;
  }
}

void setup() {
  // Start the serial monitor at a fast baud rate
  Serial.begin(115200);

  // Set GPIO 27 as an INPUT. (Our physical 10k resistor acts as the pull-up)
  pinMode(HALL_PIN, INPUT_PULLUP);

  // Attach the interrupt to trigger on a FALLING edge (voltage dropping from 3.3V to 0V)
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hall_ISR, FALLING);

  Serial.println("Smart Rod - Hall Sensor Test Ready!");
  Serial.println("Bring the magnet close to the sensor...");
}

void loop() {
  // Static variable to keep track of the last printed count
  static int lastCount = -1;

  // Only print to the Serial Monitor when the count actually goes up
  if (pulseCount != lastCount) {
    Serial.print("Magnet detected! Total Pulses: ");
    Serial.println(pulseCount);
    lastCount = pulseCount;
  }

  delay(10); // Tiny delay to keep the loop running smoothly
}

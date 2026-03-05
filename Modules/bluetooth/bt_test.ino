#include "BluetoothSerial.h"

BluetoothSerial ESP_BT;

void setup() {
  Serial.begin(115200);
  
  // This MUST match what you look for in your MIT App
  ESP_BT.begin("Cyberfish_Rod"); 
  
  Serial.println(">>> Bluetooth Test Active <<<");
  Serial.println("1. Open Cyberfish_2777 App");
  Serial.println("2. Connect to 'Cyberfish_Rod'");
}

void loop() {
  // Sending dummy data: Force=1.23, Distance=5.5, Status=TESTING
  ESP_BT.print("1.23"); 
  ESP_BT.print(",");
  ESP_BT.print("5.5");
  ESP_BT.print(",");
  ESP_BT.println("TESTING");

  Serial.println("Sent: 1.23, 5.5, TESTING");
  
  delay(2000); // Send every 2 seconds
}

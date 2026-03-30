#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(3000); // Give the Serial Monitor time to open

  Serial.println("\n--- ESP32-S3 N16R8 HARDWARE CHECK ---");

  // 1. Check Flash Size
  uint32_t flashSize = ESP.getFlashChipSize() / (1024 * 1024);
  Serial.printf("Flash Size: %u MB\n", flashSize);

  // 2. Check PSRAM (The "R8" part)
  if (psramInit()) {
    uint32_t psramSize = ESP.getPsramSize() / (1024 * 1024);
    uint32_t freePsram = ESP.getFreePsram() / (1024 * 1024);
    Serial.printf("PSRAM Total: %u MB\n", psramSize);
    Serial.printf("PSRAM Free:  %u MB\n", freePsram);
  } else {
    Serial.println("PSRAM: Not detected! Check your platformio.ini.");
  }

  // 3. Check Internal RAM
  Serial.printf("Internal Free Heap: %u KB\n", ESP.getFreeHeap() / 1024);

  // 4. Core Identification
  Serial.printf("Setup is running on Core: %d\n", xPortGetCoreID());
  
  Serial.println("-------------------------------------\n");
}

void loop() {
  // Simple speed test
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    float val = 3.14159 * random(1, 100) / 2.718; // FPU Test
    Serial.printf("Loop Heartbeat - Core: %d | FPU Math: %.4f\n", xPortGetCoreID(), val);
    lastCheck = millis();
  }
}
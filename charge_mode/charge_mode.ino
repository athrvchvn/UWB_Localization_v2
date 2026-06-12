// ─────────────────────────────────────────────────────────
//  CHARGE MODE — UWB Module Battery Charger Sketch
// ─────────────────────────────────────────────────────────
//
//  Flash this to every module (anchor or tag) before putting
//  them on charge. It:
//    • Initialises NOTHING (no WiFi, no SPI, no DW1000)
//    • Disables WiFi & Bluetooth radios to cut idle current
//    • Puts the ESP32 into light sleep so the CPU draws < 1 mA
//    • Lets the onboard charging IC (TP4056 or similar) work
//      completely unobstructed through its dedicated charge path
//
//  The module will sleep in a 10-second cycle and print a
//  heartbeat on Serial so you can confirm it is running.
//  Simply unplug USB / disconnect power when done charging.
// ─────────────────────────────────────────────────────────

#include <Arduino.h>
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_bt.h"

#define SLEEP_INTERVAL_MS 10000   // wake briefly every 10 s to print heartbeat

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  UWB MODULE — CHARGE MODE");
  Serial.println("  All peripherals OFF. Battery charging.");
  Serial.println("========================================");

  // ── Disable radios completely ────────────────────────────
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_disable();

  // ── Configure light-sleep wakeup timer ──────────────────
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_MS * 1000ULL);

  Serial.println("Radios OFF. Entering light-sleep cycle…");
  Serial.flush();
}

void loop() {
  // Enter light-sleep — CPU halts, charger IC keeps running
  esp_light_sleep_start();

  // Woke up from timer — print heartbeat and go back to sleep
  Serial.printf("[CHARGE] Awake for heartbeat — %lu s elapsed\n",
                millis() / 1000UL);
  Serial.flush();

  // Re-arm the wakeup timer for next cycle
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_MS * 1000ULL);
}

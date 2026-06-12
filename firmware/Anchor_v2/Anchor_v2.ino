/*
 * Anchor_v2.ino — UWB RTLS anchor with WiFi/UDP calibration support.
 *
 * Uses the UwbRtls library (TwrEngine) as the ranging engine.
 * Adds WiFi connectivity + UDP command interface for automated
 * antenna delay calibration from the Python calibration server.
 *
 * Features:
 *   • TwrEngine as responder (standard anchor operation)
 *   • WiFi connection to the calibration network
 *   • UDP discovery beacon (broadcast) for auto-detection by PC
 *   • UDP command socket for SET_ADELAY / GET_ADELAY
 *   • NVS persistence for antenna delay (survives reboot)
 *   • OLED display for status
 *
 * Per-board configuration:
 *   Change ANCHOR_ID (0x01, 0x02, 0x03) and ANCHOR_NAME ("A1", "A2", "A3").
 *   Each board has its own antenna delay stored in NVS.
 *
 * Board: Makerfabs ESP32 UWB Pro with Display (DW1000).
 */

#define UWB_USE_OLED
#define UWB_HOSTLINK_SERIAL

#include <UwbRtls.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>

// >>>>>>>>>>>>>>>>>> SET PER BOARD <<<<<<<<<<<<<<<<<<<<<
static const uint8_t  ANCHOR_ID    = 0x01;     // unique: 0x01, 0x02, 0x03
static const char*    ANCHOR_NAME  = "A1";     // matches Python config
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// ── WiFi ──────────────────────────────────────────────
static const char* WIFI_SSID = "iitk";
static const char* WIFI_PASS = "";            // open network

// ── UDP Ports ─────────────────────────────────────────
#define CMD_PORT        4211   // incoming commands from PC
#define BEACON_PORT     4213   // outgoing discovery beacons

// ── Defaults ──────────────────────────────────────────
#define DEFAULT_ADELAY  16556  // fallback if NVS is empty

// ── Objects ───────────────────────────────────────────
TwrEngine   engine;
OledStatus  oled;
Preferences prefs;
WiFiUDP     cmdUDP;
WiFiUDP     beaconUDP;

uint16_t    Adelay         = DEFAULT_ADELAY;
bool        discovered     = false;
uint32_t    lastBeaconMs   = 0;
#define BEACON_INTERVAL_MS 5000

// ── Discovery beacon ──────────────────────────────────
void sendBeacon() {
  char pkt[128];
  snprintf(pkt, sizeof(pkt),
    "{\"beacon\":\"anchor\",\"id\":\"%s\",\"short\":\"0x%02X\",\"adelay\":%u}",
    ANCHOR_NAME, ANCHOR_ID, Adelay);
  beaconUDP.beginPacket(IPAddress(255,255,255,255), BEACON_PORT);
  beaconUDP.print(pkt);
  beaconUDP.endPacket();
  Serial.printf("[BEACON] %s\n", pkt);
}

// ── Command handler ───────────────────────────────────
void checkCommands() {
  int sz = cmdUDP.parsePacket();
  if (sz <= 0) return;

  char buf[128] = {0};
  int n = cmdUDP.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';
  Serial.printf("[CMD] received: %s\n", buf);

  // ── ACK from discovery server ───────────────────────
  if (strncmp(buf, "ACK:", 4) == 0) {
    discovered = true;
    Serial.println("[DISCOVERY] Acknowledged — stopping beacons.");
    return;
  }

  // ── SET_ADELAY:<value> ──────────────────────────────
  if (strncmp(buf, "SET_ADELAY:", 11) == 0) {
    uint32_t newDelay = (uint32_t)atol(buf + 11);
    if (newDelay > 0 && newDelay < 65535) {
      Adelay = (uint16_t)newDelay;
      engine.setAntennaDelay(Adelay);
      prefs.putUInt("adelay", Adelay);

      char reply[32];
      snprintf(reply, sizeof(reply), "OK:%u", Adelay);
      cmdUDP.beginPacket(cmdUDP.remoteIP(), cmdUDP.remotePort());
      cmdUDP.print(reply);
      cmdUDP.endPacket();
      Serial.printf("[CMD] Adelay updated → %u (persisted)\n", Adelay);
    } else {
      cmdUDP.beginPacket(cmdUDP.remoteIP(), cmdUDP.remotePort());
      cmdUDP.print("ERR:out_of_range");
      cmdUDP.endPacket();
    }
    return;
  }

  // ── GET_ADELAY ──────────────────────────────────────
  if (strcmp(buf, "GET_ADELAY") == 0) {
    char reply[32];
    snprintf(reply, sizeof(reply), "ADELAY:%u", Adelay);
    cmdUDP.beginPacket(cmdUDP.remoteIP(), cmdUDP.remotePort());
    cmdUDP.print(reply);
    cmdUDP.endPacket();
    return;
  }

  Serial.printf("[CMD] Unknown: %s\n", buf);
}

// ── Splash screen ─────────────────────────────────────
void showStatus(const char* extra = nullptr) {
  char title[12], l1[20], l2[20], l3[24];
  snprintf(title, sizeof(title), "ANCHOR %s", ANCHOR_NAME);
  snprintf(l1, sizeof(l1), "Delay: %u", Adelay);

  if (WiFi.status() == WL_CONNECTED) {
    snprintf(l2, sizeof(l2), "IP:%s", WiFi.localIP().toString().c_str());
  } else {
    strncpy(l2, "WiFi: connecting", sizeof(l2));
  }

  if (extra) {
    strncpy(l3, extra, sizeof(l3));
  } else if (engine.lastPeer() != UWB_ADDR_INVALID) {
    snprintf(l3, sizeof(l3), "Tag:0x%02X d=%.2fm",
             engine.lastPeer(), engine.lastDistance());
  } else {
    strncpy(l3, "Waiting for tag...", sizeof(l3));
  }

  oled.showSplash(title, l1, l2, l3);
}

// ── Setup ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  // OLED first (before WiFi to avoid I2C issues)
  oled.begin();
  showStatus("Starting...");

  // Load persistent delay from NVS
  prefs.begin("uwb", false);
  Adelay = (uint16_t)prefs.getUInt("adelay", DEFAULT_ADELAY);
  Serial.printf("Loaded Adelay from NVS: %u\n", Adelay);

  // WiFi
  if (strlen(WIFI_PASS) > 0) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  } else {
    WiFi.begin(WIFI_SSID);  // open network
  }
  Serial.print("WiFi connecting");
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected — IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi failed — continuing without network");
  }

  // Re-init OLED after WiFi (WiFi RF can glitch I2C)
  oled.begin();

  // UDP sockets
  cmdUDP.begin(CMD_PORT);
  beaconUDP.begin(BEACON_PORT);
  Serial.printf("CMD port: %d  |  BEACON port: %d\n", CMD_PORT, BEACON_PORT);

  // UWB engine (anchor/responder role)
  engine.begin(TWR_ANCHOR, ANCHOR_ID, Adelay);
  engine.printDeviceId();
  Serial.printf("Anchor %s (0x%02X) ready — delay %u\n",
                ANCHOR_NAME, ANCHOR_ID, Adelay);
  showStatus();
}

// ── Loop ──────────────────────────────────────────────
void loop() {
  // Ranging — service any incoming tag polls
  engine.serviceResponder();

  // Network commands
  checkCommands();

  // Discovery beacon (until server ACKs)
  if (!discovered && WiFi.status() == WL_CONNECTED) {
    uint32_t now = millis();
    if (now - lastBeaconMs >= BEACON_INTERVAL_MS) {
      lastBeaconMs = now;
      sendBeacon();
    }
  }

  // OLED update (every 500ms)
  static uint32_t lastOled = 0;
  if (millis() - lastOled >= 500) {
    lastOled = millis();
    showStatus();
  }
}

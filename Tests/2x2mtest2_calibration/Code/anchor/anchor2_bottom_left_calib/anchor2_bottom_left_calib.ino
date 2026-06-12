// ============================================================
// Anchor 2 — Equilateral Triangle Layout  (CALIBRATION)
// Position: A2 = (-1.7321, -1.000)  [Bottom left]
// Short address LSB: 0x85
//
// New vs. original anchor:
//   • Connects to WiFi
//   • Loads Adelay from NVS (Preferences); falls back to 16556
//   • Broadcasts a UDP discovery beacon every 5 s on port 4213
//     until Python server sends an ACK
//   • Listens on port 4211 for plain-text commands:
//       "SET_ADELAY:<uint>"  → update delay, persist to NVS, reply "OK:<val>"
//       "GET_ADELAY"        → reply "ADELAY:<val>"
// ============================================================

#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// ── Identity ──────────────────────────────────────────────────
#define ANCHOR_ID    "A2"
#define ANCHOR_SHORT 0x85
char anchor_addr[] = "85:00:5B:D5:A9:9A:E2:9C";  // A2
static const char* ANCHOR_POS_STR = "A2 (-1.7321, -1.000) bottom-left";

// ── WiFi ──────────────────────────────────────────────────────
const char* WIFI_SSID = "UWB";
const char* WIFI_PASS = "00000000";

// ── Ports ──────────────────────────────────────────────────────
#define CMD_PORT    4211
#define BEACON_PORT 4213

// ── DW1000 SPI pins ───────────────────────────────────────────
#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 21;

// ── State ─────────────────────────────────────────────────────
Preferences prefs;
WiFiUDP     cmdUDP;
WiFiUDP     beaconUDP;
uint16_t    Adelay         = 16556;
bool        discovered     = false;
uint32_t    lastBeaconMs   = 0;
#define BEACON_INTERVAL_MS 5000

void sendBeacon() {
  char pkt[128];
  snprintf(pkt, sizeof(pkt),
    "{\"beacon\":\"anchor\",\"id\":\"%s\",\"short\":\"0x%02X\",\"adelay\":%u}",
    ANCHOR_ID, ANCHOR_SHORT, Adelay);
  beaconUDP.beginPacket(IPAddress(255,255,255,255), BEACON_PORT);
  beaconUDP.print(pkt);
  beaconUDP.endPacket();
  Serial.printf("[BEACON] %s\n", pkt);
}

void checkCommands() {
  int sz = cmdUDP.parsePacket();
  if (sz <= 0) return;

  char buf[128] = {0};
  int n = cmdUDP.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';
  Serial.printf("[CMD] received: %s\n", buf);

  if (strncmp(buf, "ACK:", 4) == 0) {
    discovered = true;
    Serial.println("[DISCOVERY] Acknowledged by server — stopping beacons.");
    return;
  }

  if (strncmp(buf, "SET_ADELAY:", 11) == 0) {
    uint32_t newDelay = (uint32_t)atol(buf + 11);
    if (newDelay > 0 && newDelay < 65535) {
      Adelay = (uint16_t)newDelay;
      DW1000.setAntennaDelay(Adelay);
      prefs.putUInt("adelay", Adelay);
      char reply[32];
      snprintf(reply, sizeof(reply), "OK:%u", Adelay);
      cmdUDP.beginPacket(cmdUDP.remoteIP(), cmdUDP.remotePort());
      cmdUDP.print(reply);
      cmdUDP.endPacket();
      Serial.printf("[CMD] Adelay updated → %u (persisted to NVS)\n", Adelay);
    } else {
      cmdUDP.beginPacket(cmdUDP.remoteIP(), cmdUDP.remotePort());
      cmdUDP.print("ERR:out_of_range");
      cmdUDP.endPacket();
    }
    return;
  }

  if (strcmp(buf, "GET_ADELAY") == 0) {
    char reply[32];
    snprintf(reply, sizeof(reply), "ADELAY:%u", Adelay);
    cmdUDP.beginPacket(cmdUDP.remoteIP(), cmdUDP.remotePort());
    cmdUDP.print(reply);
    cmdUDP.endPacket();
    return;
  }

  Serial.printf("[CMD] Unknown command: %s\n", buf);
}

void newRange() {
  Serial.printf("0x%04X  %.3f m\n",
    DW1000Ranging.getDistantDevice()->getShortAddress(),
    DW1000Ranging.getDistantDevice()->getRange());
}
void newDevice(DW1000Device *d)   { Serial.printf("Added:   0x%04X\n", d->getShortAddress()); }
void inactiveDevice(DW1000Device *d) { Serial.printf("Removed: 0x%04X\n", d->getShortAddress()); }

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== UWB Anchor (Calibration) ===");
  Serial.printf("ID: %s\n", ANCHOR_POS_STR);

  prefs.begin("uwb", false);
  Adelay = (uint16_t)prefs.getUInt("adelay", 16556);
  Serial.printf("Loaded Adelay from NVS: %u\n", Adelay);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nConnected — IP: %s\n", WiFi.localIP().toString().c_str());

  cmdUDP.begin(CMD_PORT);
  beaconUDP.begin(BEACON_PORT);
  Serial.printf("CMD socket: :%d  |  BEACON socket: :%d\n", CMD_PORT, BEACON_PORT);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000.setAntennaDelay(Adelay);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  Serial.println("Anchor running. Beaconing for discovery...");
}

void loop() {
  DW1000Ranging.loop();
  checkCommands();

  if (!discovered) {
    uint32_t now = millis();
    if (now - lastBeaconMs >= BEACON_INTERVAL_MS) {
      lastBeaconMs = now;
      sendBeacon();
    }
  }
}

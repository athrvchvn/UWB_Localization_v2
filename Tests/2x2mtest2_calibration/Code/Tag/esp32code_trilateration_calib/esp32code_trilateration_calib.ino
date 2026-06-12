// ============================================================
// UWB Tag — Equilateral Triangle Anchor Geometry  (CALIBRATION)
// ============================================================
// Identical to 2x2mtest2/Code/Tag/esp32code_trilateration.ino
// with one addition: "ts" (millis timestamp) field in UDP packet
// to allow the Python calibration server to detect duplicate
// or stale samples during rapid capture.
// ============================================================

#include "DW1000.h"
#include "DW1000Ranging.h"
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// ── WiFi credentials ──────────────────────────────────────────
const char *WIFI_SSID = "UWB";
const char *WIFI_PASS = "00000000";

// ── UDP ───────────────────────────────────────────────────────
#define UDP_PORT 4210
#define UDP_BROADCAST_IP "255.255.255.255"
WiFiUDP udp;

// ── DW1000 SPI pins ───────────────────────────────────────────
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 21;

char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

// ── Anchor configuration ──────────────────────────────────────
#define N_ANCHORS 3
#define ANCHOR_STALE_MS 5000

static const float SQRT3 = 1.73205f;
float anchor_matrix[N_ANCHORS][3] = {
    {0.0000f, 2.0000f, 0.0f}, // A1  (0x84)
    {-SQRT3, -1.0000f, 0.0f}, // A2  (0x85)
    {SQRT3, -1.0000f, 0.0f},  // A3  (0x86)
};

#define WS_XMIN -1.0f
#define WS_XMAX 1.0f
#define WS_YMIN -1.0f
#define WS_YMAX 1.0f
#define MAX_RANGE_M 5.0f
#define MIN_RANGE_M 0.05f

uint32_t last_anchor_update[N_ANCHORS] = {0};
float last_anchor_distance[N_ANCHORS] = {0.0f};
float current_tag_position[2] = {0.0f, 0.0f};
float current_distance_rmse = 0.0f;

// ══════════════════════════════════════════════════════════════
//  LAYER 1 — Sliding-window Median Filter  (window = 7)
// ══════════════════════════════════════════════════════════════
#define MEDIAN_WINDOW 7
struct MedianFilter {
  float buf[MEDIAN_WINDOW];
  int cnt, idx;
  void init() {
    cnt = 0;
    idx = 0;
  }
  float update(float v) {
    buf[idx] = v;
    idx = (idx + 1) % MEDIAN_WINDOW;
    if (cnt < MEDIAN_WINDOW)
      cnt++;
    float s[MEDIAN_WINDOW];
    for (int i = 0; i < cnt; i++)
      s[i] = buf[i];
    for (int i = 1; i < cnt; i++) {
      float key = s[i];
      int j = i - 1;
      while (j >= 0 && s[j] > key) {
        s[j + 1] = s[j];
        j--;
      }
      s[j + 1] = key;
    }
    return s[cnt / 2];
  }
};
MedianFilter medFilt[N_ANCHORS];

// ══════════════════════════════════════════════════════════════
//  LAYER 2 — EMA + Symmetric Outlier Gate  (per anchor)
// ══════════════════════════════════════════════════════════════
#define EMA_ALPHA 0.25f
#define OUTLIER_GATE 0.50f
struct EMAFilter {
  float val;
  bool ready;
  void init() {
    val = 0.0f;
    ready = false;
  }
  float update(float m) {
    if (!ready) {
      val = m;
      ready = true;
      return val;
    }
    if (fabsf(m - val) > OUTLIER_GATE)
      return val;
    val = EMA_ALPHA * m + (1.0f - EMA_ALPHA) * val;
    return val;
  }
};
EMAFilter emaFilt[N_ANCHORS];

// ══════════════════════════════════════════════════════════════
//  LAYER 3 — Extended Kalman Filter  state = [x, y, vx, vy]
// ══════════════════════════════════════════════════════════════
#define EKF_QP 0.003f
#define EKF_QV 0.050f
#define EKF_R_BASE 0.025f
#define EKF_K_RMSE 15.0f
#define EKF_RMSE_GATE 0.80f

struct EKF2D {
  float x[4];
  float P[4][4];
  uint32_t tPrev;
  bool ok;
  void init() { ok = false; }
  void seed(float px, float py, uint32_t t) {
    x[0] = px;
    x[1] = py;
    x[2] = 0.0f;
    x[3] = 0.0f;
    memset(P, 0, sizeof(P));
    P[0][0] = 0.5f;
    P[1][1] = 0.5f;
    P[2][2] = 0.3f;
    P[3][3] = 0.3f;
    tPrev = t;
    ok = true;
  }
  void predict(uint32_t t) {
    float dt = (t - tPrev) / 1000.0f;
    tPrev = t;
    if (dt <= 0.0f)
      dt = 0.001f;
    if (dt > 1.0f)
      dt = 1.0f;
    x[0] += x[2] * dt;
    x[1] += x[3] * dt;
    float FP[4][4];
    for (int j = 0; j < 4; j++) {
      FP[0][j] = P[0][j] + dt * P[2][j];
      FP[1][j] = P[1][j] + dt * P[3][j];
      FP[2][j] = P[2][j];
      FP[3][j] = P[3][j];
    }
    for (int i = 0; i < 4; i++) {
      P[i][0] = FP[i][0] + FP[i][2] * dt;
      P[i][1] = FP[i][1] + FP[i][3] * dt;
      P[i][2] = FP[i][2];
      P[i][3] = FP[i][3];
    }
    P[0][0] += EKF_QP * dt;
    P[1][1] += EKF_QP * dt;
    P[2][2] += EKF_QV * dt;
    P[3][3] += EKF_QV * dt;
  }
  void update(float zx, float zy, float rmse) {
    float r = EKF_R_BASE * (1.0f + EKF_K_RMSE * rmse * rmse);
    float y0 = zx - x[0], y1 = zy - x[1];
    float S00 = P[0][0] + r, S01 = P[0][1];
    float S10 = P[1][0], S11 = P[1][1] + r;
    float det = S00 * S11 - S01 * S10;
    if (fabsf(det) < 1e-10f)
      return;
    float id = 1.0f / det;
    float Si00 = id * S11, Si01 = -id * S01;
    float Si10 = -id * S10, Si11 = id * S00;
    float K[4][2];
    for (int i = 0; i < 4; i++) {
      K[i][0] = P[i][0] * Si00 + P[i][1] * Si10;
      K[i][1] = P[i][0] * Si01 + P[i][1] * Si11;
    }
    for (int i = 0; i < 4; i++)
      x[i] += K[i][0] * y0 + K[i][1] * y1;
    float Pn[4][4];
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) {
        Pn[i][j] = 0.0f;
        for (int k = 0; k < 4; k++) {
          float IKH = (i == k ? 1.0f : 0.0f);
          if (k == 0)
            IKH -= K[i][0];
          if (k == 1)
            IKH -= K[i][1];
          Pn[i][j] += IKH * P[k][j];
        }
      }
    memcpy(P, Pn, sizeof(P));
  }
};
EKF2D ekf;
float ekf_position[2] = {0.0f, 0.0f};

// ══════════════════════════════════════════════════════════════
//  Trilateration — linear least-squares, 3-anchor
// ══════════════════════════════════════════════════════════════
static bool trilat_init = false;
static float Ainv[2][2];
static float k_anch[N_ANCHORS];

int trilat2D() {
  float d[N_ANCHORS];
  for (int i = 0; i < N_ANCHORS; i++)
    d[i] = last_anchor_distance[i];
  if (!trilat_init) {
    trilat_init = true;
    float A[2][2];
    for (int i = 0; i < N_ANCHORS; i++)
      k_anch[i] = anchor_matrix[i][0] * anchor_matrix[i][0] +
                  anchor_matrix[i][1] * anchor_matrix[i][1];
    for (int i = 1; i < N_ANCHORS; i++) {
      A[i - 1][0] = anchor_matrix[i][0] - anchor_matrix[0][0];
      A[i - 1][1] = anchor_matrix[i][1] - anchor_matrix[0][1];
    }
    float det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    if (fabsf(det) < 1.0e-4f) {
      while (1)
        delay(1);
    }
    float id = 1.0f / det;
    Ainv[0][0] = id * A[1][1];
    Ainv[0][1] = -id * A[0][1];
    Ainv[1][0] = -id * A[1][0];
    Ainv[1][1] = id * A[0][0];
  }
  float b[2];
  for (int i = 1; i < N_ANCHORS; i++)
    b[i - 1] = d[0] * d[0] - d[i] * d[i] + k_anch[i] - k_anch[0];
  current_tag_position[0] = 0.5f * (Ainv[0][0] * b[0] + Ainv[0][1] * b[1]);
  current_tag_position[1] = 0.5f * (Ainv[1][0] * b[0] + Ainv[1][1] * b[1]);
  float rmse = 0.0f;
  for (int i = 0; i < N_ANCHORS; i++) {
    float dx = current_tag_position[0] - anchor_matrix[i][0];
    float dy = current_tag_position[1] - anchor_matrix[i][1];
    float e = d[i] - sqrtf(dx * dx + dy * dy);
    rmse += e * e;
  }
  current_distance_rmse = sqrtf(rmse / (float)N_ANCHORS);
  return 1;
}

// ══════════════════════════════════════════════════════════════
//  UDP broadcast — ADDED: "ts" field for calibration timestamping
// ══════════════════════════════════════════════════════════════
void sendUDP() {
  char pkt[256];
  snprintf(pkt, sizeof(pkt),
           "{\"x\":%.4f,\"y\":%.4f,\"ex\":%.4f,\"ey\":%.4f,"
           "\"rmse\":%.4f,\"d0\":%.4f,\"d1\":%.4f,\"d2\":%.4f,\"ts\":%lu}",
           current_tag_position[0], current_tag_position[1], ekf_position[0],
           ekf_position[1], current_distance_rmse, last_anchor_distance[0],
           last_anchor_distance[1], last_anchor_distance[2],
           (unsigned long)millis());
  udp.beginPacket(UDP_BROADCAST_IP, UDP_PORT);
  udp.print(pkt);
  udp.endPacket();
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(
      "\n=== UWB Tag — Equilateral Triangle Geometry (CALIBRATION) ===");
  for (int i = 0; i < N_ANCHORS; i++) {
    medFilt[i].init();
    emaFilt[i].init();
  }
  ekf.init();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected — IP: %s\n", WiFi.localIP().toString().c_str());
  udp.begin(UDP_PORT);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER,
                           false);
}

void loop() { DW1000Ranging.loop(); }

// ── Range callback ────────────────────────────────────────────
void newRange() {
  uint16_t addr = DW1000Ranging.getDistantDevice()->getShortAddress();
  int ai = -1;
  if ((addr & 0xFF) == 0x84)
    ai = 0;
  if ((addr & 0xFF) == 0x85)
    ai = 1;
  if ((addr & 0xFF) == 0x86)
    ai = 2;
  if (ai >= 0) {
    float raw = DW1000Ranging.getDistantDevice()->getRange();
    if (raw < MIN_RANGE_M || raw > MAX_RANGE_M)
      return;
    last_anchor_update[ai] = millis();
    float med = medFilt[ai].update(raw);
    float flt = emaFilt[ai].update(med);
    last_anchor_distance[ai] = flt;
  }
  uint32_t now = millis();
  int fresh = 0;
  for (int i = 0; i < N_ANCHORS; i++) {
    if (last_anchor_update[i] > 0 &&
        (now - last_anchor_update[i]) < ANCHOR_STALE_MS)
      fresh++;
  }
  if (fresh == N_ANCHORS) {
    trilat2D();
    if (!ekf.ok) {
      ekf.seed(current_tag_position[0], current_tag_position[1], now);
    } else {
      ekf.predict(now);
      if (current_distance_rmse < EKF_RMSE_GATE)
        ekf.update(current_tag_position[0], current_tag_position[1],
                   current_distance_rmse);
    }
    ekf_position[0] = ekf.x[0];
    ekf_position[1] = ekf.x[1];
    sendUDP();
    Serial.printf(
        "RAW=(%.3f,%.3f) EKF=(%.3f,%.3f) RMSE=%.4f d=[%.3f,%.3f,%.3f] ts=%lu\n",
        current_tag_position[0], current_tag_position[1], ekf_position[0],
        ekf_position[1], current_distance_rmse, last_anchor_distance[0],
        last_anchor_distance[1], last_anchor_distance[2], (unsigned long)now);
  }
}

void newDevice(DW1000Device *device) {
  Serial.printf("Device added:   0x%04X\n", device->getShortAddress());
}
void inactiveDevice(DW1000Device *device) {
  uint16_t addr = device->getShortAddress();
  int ai = -1;
  if ((addr & 0xFF) == 0x84)
    ai = 0;
  if ((addr & 0xFF) == 0x85)
    ai = 1;
  if ((addr & 0xFF) == 0x86)
    ai = 2;
  if (ai >= 0) {
    last_anchor_update[ai] = 0;
    emaFilt[ai].init();
    medFilt[ai].init();
    Serial.printf("Anchor A%d gone inactive — reset filters\n", ai + 1);
  }
}

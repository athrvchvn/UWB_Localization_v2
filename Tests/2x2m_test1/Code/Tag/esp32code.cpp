// UWB Tag — 2D trilateration with 3-layer filtering + EKF + UDP broadcast
//
// Layer 1: Sliding-window median filter (per anchor, window=5)
//           → kills multipath / NLOS spikes
// Layer 2: Exponential moving average + outlier gate (per anchor)
//           → smooths residual jitter, rejects sustained outliers
// Layer 3: Extended Kalman Filter on (x, y, vx, vy)
//           → temporal fusion with constant-velocity model
//           → adaptive measurement noise scaled by trilateration RMSE
//
// Pipeline:  raw range → median → EMA+gate → trilateration → EKF → output

#include "DW1000.h"
#include "DW1000Ranging.h"
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <math.h>

// ── WiFi credentials ──────────────────────────────────────
const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ── UDP ───────────────────────────────────────────────────
#define UDP_PORT 4210
WiFiUDP udp;

// ── DW1000 pins ───────────────────────────────────────────
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 21;

char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

// ── Anchor configuration ──────────────────────────────────
#define N_ANCHORS 3
#define ANCHOR_DISTANCE_EXPIRED 5000 // ms

// 2 m × 2 m right-triangle layout
float anchor_matrix[N_ANCHORS][3] = {
    {0.0f, 0.0f, 0.0f}, // Anchor #1 (0x84) — origin
    {0.0f, 2.0f, 0.0f}, // Anchor #2 (0x85) — 2 m along Y
    {2.0f, 0.0f, 0.0f}, // Anchor #3 (0x86) — 2 m along X
};

uint32_t last_anchor_update[N_ANCHORS] = {0};
float last_anchor_distance[N_ANCHORS] = {0.0f}; // filtered ranges
float current_tag_position[2] = {0.0f, 0.0f};   // raw trilat output
float current_distance_rmse = 0.0f;

// ══════════════════════════════════════════════════════════
//  LAYER 1 — Median Filter  (per anchor, window = 5)
// ══════════════════════════════════════════════════════════
#define MEDIAN_WINDOW 5

struct MedianFilter {
  float buf[MEDIAN_WINDOW];
  int cnt;
  int idx;

  void init() {
    cnt = 0;
    idx = 0;
  }

  float update(float v) {
    buf[idx] = v;
    idx = (idx + 1) % MEDIAN_WINDOW;
    if (cnt < MEDIAN_WINDOW)
      cnt++;

    // copy + insertion-sort (5 elements — trivial cost)
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

// ══════════════════════════════════════════════════════════
//  LAYER 2 — EMA + Outlier Gate  (per anchor)
// ══════════════════════════════════════════════════════════
#define EMA_ALPHA 0.3f      // smoothing factor (0 = full smooth, 1 = no smooth)
#define OUTLIER_GATE_M 0.8f // reject if jump > this (meters)

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
    if (fabsf(m - val) > OUTLIER_GATE_M)
      return val; // reject
    val = EMA_ALPHA * m + (1.0f - EMA_ALPHA) * val;
    return val;
  }
};

EMAFilter emaFilt[N_ANCHORS];

// ══════════════════════════════════════════════════════════
//  LAYER 3 — Extended Kalman Filter   state = [x y vx vy]
// ══════════════════════════════════════════════════════════
//
//  F = [1 0 dt 0]   Q = diag(qp, qp, qv, qv)*dt
//      [0 1 0 dt]
//      [0 0 1  0]   H = [1 0 0 0]
//      [0 0 0  1]       [0 1 0 0]
//
//  Adaptive R_scalar = R_base * (1 + K * rmse^2)

#define EKF_QP 0.005f      // process noise position  (m²)
#define EKF_QV 0.08f       // process noise velocity  (m²/s²)
#define EKF_R_BASE 0.04f   // base measurement noise  (m²)
#define EKF_K_RMSE 20.0f   // RMSE→R scaling
#define EKF_RMSE_GATE 1.2f // skip update if RMSE > this (m)

struct EKF2D {
  float x[4];    // state
  float P[4][4]; // covariance
  uint32_t tPrev;
  bool ok; // initialized?

  void init() { ok = false; }

  // first fix — seed state from trilateration
  void seed(float px, float py, uint32_t t) {
    x[0] = px;
    x[1] = py;
    x[2] = 0.0f;
    x[3] = 0.0f;
    memset(P, 0, sizeof(P));
    P[0][0] = 1.0f;
    P[1][1] = 1.0f; // pos uncertainty
    P[2][2] = 0.5f;
    P[3][3] = 0.5f; // vel uncertainty
    tPrev = t;
    ok = true;
  }

  // ── predict ──────────────────────────────────────────
  void predict(uint32_t t) {
    float dt = (t - tPrev) / 1000.0f;
    tPrev = t;
    if (dt <= 0.0f)
      dt = 0.001f;
    if (dt > 1.0f)
      dt = 1.0f;

    // state prediction
    x[0] += x[2] * dt;
    x[1] += x[3] * dt;

    // P = F·P·F^T + Q   (exploit F structure)
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

  // ── measurement update ───────────────────────────────
  void update(float zx, float zy, float rmse) {
    float r = EKF_R_BASE * (1.0f + EKF_K_RMSE * rmse * rmse);

    // innovation
    float y0 = zx - x[0];
    float y1 = zy - x[1];

    // S = H·P·H^T + R   (2×2, top-left of P + r·I)
    float S00 = P[0][0] + r, S01 = P[0][1];
    float S10 = P[1][0], S11 = P[1][1] + r;

    float det = S00 * S11 - S01 * S10;
    if (fabsf(det) < 1e-10f)
      return; // singular — skip
    float id = 1.0f / det;

    // S^-1
    float Si00 = id * S11, Si01 = -id * S01;
    float Si10 = -id * S10, Si11 = id * S00;

    // K = P·H^T · S^-1   (4×2)
    float K[4][2];
    for (int i = 0; i < 4; i++) {
      K[i][0] = P[i][0] * Si00 + P[i][1] * Si10;
      K[i][1] = P[i][0] * Si01 + P[i][1] * Si11;
    }

    // state update
    for (int i = 0; i < 4; i++)
      x[i] += K[i][0] * y0 + K[i][1] * y1;

    // covariance update  P = (I − K·H) · P
    float Pn[4][4];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        // (I-KH)[i][k] = δ(i,k) − K[i][0]·(k==0) − K[i][1]·(k==1)
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
    }
    memcpy(P, Pn, sizeof(P));
  }
};

EKF2D ekf;
float ekf_position[2] = {0.0f, 0.0f};

// ══════════════════════════════════════════════════════════
//  UDP send
// ══════════════════════════════════════════════════════════
void sendUDP() {
  char pkt[200];
  snprintf(pkt, sizeof(pkt),
           "{\"x\":%.4f,\"y\":%.4f,\"ex\":%.4f,\"ey\":%.4f,"
           "\"rmse\":%.4f,\"d0\":%.4f,\"d1\":%.4f,\"d2\":%.4f}",
           current_tag_position[0], current_tag_position[1], ekf_position[0],
           ekf_position[1], current_distance_rmse, last_anchor_distance[0],
           last_anchor_distance[1], last_anchor_distance[2]);

  udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT);
  udp.print(pkt);
  udp.endPacket();
}

// ══════════════════════════════════════════════════════════
//  Setup
// ══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);

  for (int i = 0; i < N_ANCHORS; i++) {
    medFilt[i].init();
    emaFilt[i].init();
  }
  ekf.init();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.printf("Broadcasting UDP on port %d\n", UDP_PORT);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER,
                           false);
}

void loop() { DW1000Ranging.loop(); }

// ══════════════════════════════════════════════════════════
//  Range callback — the full filtering pipeline
// ══════════════════════════════════════════════════════════
void newRange() {
  uint16_t addr = DW1000Ranging.getDistantDevice()->getShortAddress();

  int index = 0;
  if (addr == 0x84)
    index = 1;
  if (addr == 0x85)
    index = 2;
  if (addr == 0x86)
    index = 3;

  if (index > 0) {
    float raw = DW1000Ranging.getDistantDevice()->getRange();

    // hard sanity gate — 2x2 m diagonal ≈ 2.83 m; 6 m gives headroom
    if (raw < 0.0f || raw > 6.0f)
      return;

    int ai = index - 1; // array index
    last_anchor_update[ai] = millis();

    float med = medFilt[ai].update(raw); // Layer 1
    float flt = emaFilt[ai].update(med); // Layer 2
    last_anchor_distance[ai] = flt;
  }

  // check freshness
  int detected = 0;
  for (int i = 0; i < N_ANCHORS; i++) {
    if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED)
      last_anchor_update[i] = 0;
    if (last_anchor_update[i] > 0)
      detected++;
  }

  if (detected == 3) {
    trilat2D_3A();

    uint32_t now = millis();

    // ── Layer 3: EKF ──
    if (!ekf.ok) {
      ekf.seed(current_tag_position[0], current_tag_position[1], now);
      ekf_position[0] = ekf.x[0];
      ekf_position[1] = ekf.x[1];
    } else {
      ekf.predict(now);
      if (current_distance_rmse < EKF_RMSE_GATE) {
        ekf.update(current_tag_position[0], current_tag_position[1],
                   current_distance_rmse);
      }
      // else: coast on prediction only — bad trilateration ignored
      ekf_position[0] = ekf.x[0];
      ekf_position[1] = ekf.x[1];
    }

    sendUDP();

    Serial.printf(
        "RAW=(%.3f,%.3f) EKF=(%.3f,%.3f) RMSE=%.4f d=[%.3f,%.3f,%.3f]\n",
        current_tag_position[0], current_tag_position[1], ekf_position[0],
        ekf_position[1], current_distance_rmse, last_anchor_distance[0],
        last_anchor_distance[1], last_anchor_distance[2]);
  }
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

// ══════════════════════════════════════════════════════════
//  Trilateration  (same least-squares algo, uses filtered d[])
// ══════════════════════════════════════════════════════════
int trilat2D_3A(void) {
  static bool first = true;
  float b[N_ANCHORS], d[N_ANCHORS];
  static float Ainv[2][2], k[N_ANCHORS];

  for (int i = 0; i < N_ANCHORS; i++)
    d[i] = last_anchor_distance[i];

  if (first) {
    first = false;
    float xc[N_ANCHORS], yc[N_ANCHORS], A[2][2];

    for (int i = 0; i < N_ANCHORS; i++) {
      xc[i] = anchor_matrix[i][0];
      yc[i] = anchor_matrix[i][1];
      k[i] = xc[i] * xc[i] + yc[i] * yc[i];
    }
    for (int i = 1; i < N_ANCHORS; i++) {
      A[i - 1][0] = xc[i] - xc[0];
      A[i - 1][1] = yc[i] - yc[0];
    }
    float det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    if (fabs(det) < 1.0E-4) {
      Serial.println("*** Singular matrix — check anchor coords ***");
      while (1)
        delay(1);
    }
    det = 1.0 / det;
    Ainv[0][0] = det * A[1][1];
    Ainv[0][1] = -det * A[0][1];
    Ainv[1][0] = -det * A[1][0];
    Ainv[1][1] = det * A[0][0];
  }

  for (int i = 1; i < N_ANCHORS; i++)
    b[i - 1] = d[0] * d[0] - d[i] * d[i] + k[i] - k[0];

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
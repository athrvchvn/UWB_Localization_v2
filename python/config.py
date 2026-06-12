# =============================================================
#  config.py — UWB v2 Calibration System Configuration
# =============================================================
#  Anchor IPs are auto-discovered at runtime via UDP beacons.
#  All calibration stages share this single config file.
# =============================================================

import math

# ── Ports ─────────────────────────────────────────────────────
ANCHOR_CMD_PORT = 4211   # anchors listen here for SET_ADELAY / GET_ADELAY
TAG_UDP_PORT    = 5005   # tag broadcasts RTLS sweep packets
WS_PORT         = 8765   # WebSocket server port (server → GUI)
BEACON_PORT     = 4213   # anchors broadcast discovery beacons here

# ── WiFi ──────────────────────────────────────────────────────
WIFI_SSID = "iitk"
WIFI_PASS = ""           # open network

# ── Anchor geometry (metres) — equilateral triangle ───────────
SQRT3 = math.sqrt(3)
ANCHOR_COORDS = {
    "A1": ( 0.0000,  2.0000),   # top centre
    "A2": (-SQRT3,  -1.0000),   # bottom left  (≈ -1.7321)
    "A3": ( SQRT3,  -1.0000),   # bottom right (≈  1.7321)
}

# DW1000 short-address LSB → anchor ID
ANCHOR_ADDR_MAP = {
    0x01: "A1",
    0x02: "A2",
    0x03: "A3",
}

# Anchor ID → DW1000 short-address LSB (reverse map)
ANCHOR_ID_TO_ADDR = {v: k for k, v in ANCHOR_ADDR_MAP.items()}

# ── Antenna delay defaults ────────────────────────────────────
DEFAULT_ADELAY   = 16556   # DW1000 units — anchor default
TAG_ADELAY       = 16473   # tag delay (fixed, compensated in Python)
DELAY_TO_METRES  = 0.4691e-3   # 1 delay unit ≈ 0.4691 mm

# ── Optimizer bounds ──────────────────────────────────────────
DELAY_BOUND      = 500     # ±500 units delay search range
COORD_BOUND_M    = 0.10    # ±10 cm anchor coordinate correction

# ── Calibration workspace (metres) ────────────────────────────
WORKSPACE = {"xmin": -1.0, "xmax": 1.0, "ymin": -1.0, "ymax": 1.0}

# 3×3 grid of calibration points (evenly spaced in workspace)
CALIB_POINTS = [
    (-1.0,  1.0), ( 0.0,  1.0), ( 1.0,  1.0),
    (-1.0,  0.0), ( 0.0,  0.0), ( 1.0,  0.0),
    (-1.0, -1.0), ( 0.0, -1.0), ( 1.0, -1.0),
]

# ── Capture settings ──────────────────────────────────────────
DEFAULT_CAPTURE_N  = 200   # samples per calibration point

# ── File paths ────────────────────────────────────────────────
CALIB_LOG_DIR              = "./calib_logs/"
DISCOVERED_ANCHORS_FILE    = "./discovered_anchors.json"
COORD_COMPENSATION_FILE    = "./coord_compensation.json"
ANCHORS_JSON_FILE          = "./anchors.json"

# ── Delay calibration mode ────────────────────────────────────
# "anchor_only" — fix tag delay, calibrate 3 anchor delays
# "joint"       — calibrate all 4 (anchors + tag) with zero-mean constraint
DELAY_CALIB_MODE = "anchor_only"

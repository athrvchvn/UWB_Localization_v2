# =============================================================
#  config.py — UWB Calibration System Configuration
# =============================================================
#  Anchor IPs are NOT stored here; they are auto-discovered at
#  runtime via UDP beacons on BEACON_PORT. Discovered IPs are
#  cached in-memory and optionally written to discovered_anchors.json.
# =============================================================

# ── Ports ─────────────────────────────────────────────────────
ANCHOR_CMD_PORT = 4211   # anchors listen here for SET_ADELAY / GET_ADELAY
TAG_UDP_PORT    = 4210   # tag broadcasts position packets
WS_PORT         = 8765   # WebSocket server port (server → GUI)
BEACON_PORT     = 4213   # anchors broadcast discovery beacons here

# ── WiFi (copied into each anchor .ino at compile time) ───────
WIFI_SSID = "UWB"
WIFI_PASS = "00000000"

# ── Anchor geometry (metres) — equilateral triangle ───────────
ANCHOR_COORDS = {
    "A1": ( 0.0000,  2.0000),   # top centre
    "A2": (-1.7321, -1.0000),   # bottom left
    "A3": ( 1.7321, -1.0000),   # bottom right
}

# DW1000 short-address LSB → anchor ID
ANCHOR_ADDR_MAP = {
    0x84: "A1",
    0x85: "A2",
    0x86: "A3",
}

# ── Default antenna delay ──────────────────────────────────────
DEFAULT_ADELAY = 16556   # DW1000 units (≈ 0.4691 mm per unit)

# ── Optimizer bounds ───────────────────────────────────────────
DELAY_BOUND          = 500     # ±500 units delay search range
COORD_BOUND_M        = 0.10    # ±10 cm anchor coordinate correction

# ── Calibration workspace (metres) ────────────────────────────
WORKSPACE = {"xmin": -1.0, "xmax": 1.0, "ymin": -1.0, "ymax": 1.0}

# 3×3 grid of calibration points (evenly spaced in workspace)
CALIB_POINTS = [
    (-1.0,  1.0), ( 0.0,  1.0), ( 1.0,  1.0),
    (-1.0,  0.0), ( 0.0,  0.0), ( 1.0,  0.0),
    (-1.0, -1.0), ( 0.0, -1.0), ( 1.0, -1.0),
]

# ── Capture settings ───────────────────────────────────────────
DEFAULT_CAPTURE_N  = 200   # samples per calibration point

# ── Logging ───────────────────────────────────────────────────
CALIB_LOG_DIR           = "./calib_logs/"
DISCOVERED_ANCHORS_FILE = "./discovered_anchors.json"

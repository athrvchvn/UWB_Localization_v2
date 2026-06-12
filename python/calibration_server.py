"""
calibration_server.py — Unified asyncio server for UWB v2 calibration.

Four-stage workflow:
  1. Live position tracking
  2. Anchor survey with Procrustes alignment (§1)
  3. Automatic antenna delay calibration (§2)
  4. Coordinate error compensation (§3)

Usage:
    python calibration_server.py
"""

from __future__ import annotations
import asyncio, json, logging, os, socket, sys, time, math
from collections import defaultdict
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Optional

import websockets
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import config
from survey_solver import SurveySolver, write_anchors_json
from delay_calibrator import DelayCalibrator
from coord_compensator import CoordCompensator
from rtls import FrameParser, AnchorConfig, Multilaterator, PositionEKF

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("v2_server")

# ── Tuning defaults ──────────────────────────────────────────────────────────
RANGE_FILT_N = 8; EKF_Q_ACCEL = 0.005; RANGE_SIGMA = 0.20
GATE_K = 2.5; EMA_ALPHA = 0.15; RANGE_BIAS_M = 0.04
AGE_GATE_SEC = 3.0; EKF_JUMP_THRESH = 2.0
from collections import deque

class RtlsPipeline:
    """Full signal pipeline: parse → median filter → multilat → EKF → EMA."""
    def __init__(self, cfg: AnchorConfig, compensator: CoordCompensator = None):
        self.cfg = cfg
        self.ml = Multilaterator(cfg.dim); self.ml.gate_k = GATE_K; self.ml.range_sigma = RANGE_SIGMA
        self.ekf = PositionEKF(cfg.dim); self.ekf.q_accel = EKF_Q_ACCEL
        self.compensator = compensator
        self.range_hist: dict[int, deque] = {}
        self.anchor_last_seen: dict[int, float] = {}
        self.last_pos = None; self.ema_pos = None; self.prev_t = None
        self.pkt_count = 0; self.ekf_resets = 0
        self.range_filt_n = RANGE_FILT_N; self.ema_alpha = EMA_ALPHA; self.range_bias_m = RANGE_BIAS_M

    def update_tuning(self, params: dict):
        if 'rangeFiltN' in params: self.range_filt_n = max(1, int(params['rangeFiltN']))
        if 'ekfQAccel' in params: self.ekf.q_accel = float(params['ekfQAccel'])
        if 'emaAlpha' in params: self.ema_alpha = float(params['emaAlpha'])
        if 'gateK' in params: self.ml.gate_k = float(params['gateK'])
        if 'rangeBiasM' in params: self.range_bias_m = float(params['rangeBiasM'])

    def process(self, line: str) -> dict | None:
        pkt = FrameParser.parse(line)
        if not pkt.valid: return None
        self.pkt_count += 1; now = time.monotonic()
        stale = [k for k, t in self.anchor_last_seen.items() if now - t > AGE_GATE_SEC]
        for k in stale: self.range_hist.pop(k, None); self.anchor_last_seen.pop(k, None)
        A, found = self.cfg.coords_for(pkt.ids)
        d_raw = np.array(pkt.dist); ids_arr = np.array(pkt.ids)
        mask = np.array(found, dtype=bool); A = A[mask]; d_raw = d_raw[mask]; active = ids_arr[mask]
        if len(d_raw) < self.cfg.dim + 1: return None
        d_filt = d_raw.copy()
        for k, aid in enumerate(active):
            aid_i = int(aid); self.anchor_last_seen[aid_i] = now
            if aid_i not in self.range_hist: self.range_hist[aid_i] = deque(maxlen=self.range_filt_n)
            buf = self.range_hist[aid_i]; buf.append(d_raw[k])
            while len(buf) > self.range_filt_n: buf.popleft()
            d_filt[k] = float(np.median(list(buf)))
        d_filt = np.maximum(0.05, d_filt - self.range_bias_m)
        pos, info = self.ml.solve(A, d_filt, self.last_pos)
        if not info.ok or not np.all(np.isfinite(pos)): return None
        self.last_pos = pos.copy()
        if self.prev_t is None: dt = 0.1
        else: dt = max((pkt.t_ms - self.prev_t) / 1000.0, 1e-3)
        self.prev_t = pkt.t_ms
        R = info.cov[:self.cfg.dim, :self.cfg.dim]
        if not np.all(np.isfinite(R)): R = None
        fpos, vel = self.ekf.step(dt, pos, R)
        ekf_jump = np.linalg.norm(fpos - pos)
        if ekf_jump > EKF_JUMP_THRESH or np.trace(self.ekf.P) > 25:
            self.ekf_resets += 1; self.ekf.initialize(pos); fpos = pos.copy(); self.ema_pos = None
        if self.ema_pos is None: self.ema_pos = fpos.copy()
        else: self.ema_pos = self.ema_alpha * fpos + (1 - self.ema_alpha) * self.ema_pos
        # Apply coordinate compensation if trained
        ema_out = self.ema_pos.copy()
        if self.compensator and self.compensator.trained and self.compensator.enabled:
            ema_out = self.compensator.correct(self.ema_pos)
        dists = {}
        for k, aid in enumerate(active): dists[f'd{int(aid)}'] = round(float(d_filt[k]), 4)
        return {
            't_ms': pkt.t_ms,
            'x': round(float(pos[0]), 4), 'y': round(float(pos[1]), 4),
            'ex': round(float(ema_out[0]), 4), 'ey': round(float(ema_out[1]), 4),
            'rmse': round(float(info.rms), 4), 'nUsed': int(np.sum(info.used)),
            'pktN': self.pkt_count, 'ekfR': self.ekf_resets, **dists,
        }

# ══════════════════════════════════════════════════════════════════════════════
#  Server State
# ══════════════════════════════════════════════════════════════════════════════
class ServerState:
    def __init__(self):
        self.anchor_ips: dict[str, str] = {}
        self.anchor_delays: dict[str, int] = {}
        self.clients: set = set()
        # Capture state (shared across tabs)
        self.capturing = False
        self.capture_mode = None  # "survey", "delay", "coord"
        self.capture_point = None
        self.capture_target = config.DEFAULT_CAPTURE_N
        self.capture_buffer: list[dict] = []
        # Survey (§1)
        self.survey_solver = SurveySolver(config.ANCHOR_COORDS)
        self.survey_result = None
        # Delay calibration (§2)
        self.delay_calibrator = DelayCalibrator(config.ANCHOR_COORDS, mode=config.DELAY_CALIB_MODE)
        self.delay_result = None
        # Coord compensation (§3)
        self.coord_compensator = CoordCompensator()
        self.coord_captures: list[dict] = []
        self.coord_result = None

    def all_anchors_discovered(self) -> bool:
        return all(k in self.anchor_ips for k in ["A1", "A2", "A3"])

    def status_msg(self) -> dict:
        return {
            "type": "status",
            "anchors": {k: {"ip": self.anchor_ips.get(k), "delay": self.anchor_delays.get(k, config.DEFAULT_ADELAY)} for k in ["A1", "A2", "A3"]},
            "capture": {"active": self.capturing, "mode": self.capture_mode, "point": self.capture_point, "n": len(self.capture_buffer), "target": self.capture_target},
            "survey": {"n_captures": len(self.survey_solver.captures)},
            "delay_cal": {"n_captures": len(self.delay_calibrator.captures)},
            "coord_cal": {"n_captures": len(self.coord_captures), "enabled": self.coord_compensator.enabled, "trained": self.coord_compensator.trained},
        }

state = ServerState()
pipeline: RtlsPipeline = None
cfg_global: AnchorConfig = None
log_fh = None

# ══════════════════════════════════════════════════════════════════════════════
#  Broadcast
# ══════════════════════════════════════════════════════════════════════════════
async def broadcast(msg: dict):
    if not state.clients: return
    data = json.dumps(msg)
    await asyncio.gather(*[ws.send(data) for ws in list(state.clients)], return_exceptions=True)

# ══════════════════════════════════════════════════════════════════════════════
#  Tag UDP relay
# ══════════════════════════════════════════════════════════════════════════════
class TagUDPProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data: bytes, addr):
        global pipeline
        if pipeline is None: return
        try: line = data.decode('utf-8', errors='replace').strip()
        except: return
        result = pipeline.process(line)
        if not result: return
        result["type"] = "position"
        asyncio.ensure_future(broadcast(result))
        if log_fh:
            log_fh.write(json.dumps(result) + '\n'); log_fh.flush()
        # Capture accumulation
        if state.capturing and state.capture_point is not None:
            state.capture_buffer.append(result)
            n = len(state.capture_buffer)
            asyncio.ensure_future(broadcast({"type": "capture_progress", "n": n, "target": state.capture_target, "mode": state.capture_mode, "point": state.capture_point}))
            if n >= state.capture_target:
                asyncio.ensure_future(_finish_capture())

async def _finish_capture():
    state.capturing = False
    buf = state.capture_buffer[:]
    state.capture_buffer.clear()
    if not buf: return
    pt = state.capture_point
    mode = state.capture_mode

    # Compute means
    xs = [p.get("x", 0) for p in buf]; ys = [p.get("y", 0) for p in buf]
    mean_x, mean_y = float(np.mean(xs)), float(np.mean(ys))
    # Per-anchor mean distances
    anchor_dists = {}
    for aid_int in [1, 2, 3]:
        key = f"d{aid_int}"
        vals = [p[key] for p in buf if key in p]
        if vals: anchor_dists[f"A{aid_int}"] = float(np.mean(vals))

    done_msg = {"type": "capture_done", "mode": mode, "point": pt, "n": len(buf),
                "mean_x": mean_x, "mean_y": mean_y, "anchor_dists": anchor_dists}

    if mode == "survey":
        dists_int = {int(k[1:]): v for k, v in anchor_dists.items()}
        state.survey_solver.add_capture(dists_int)
        done_msg["n_total"] = len(state.survey_solver.captures)
        log.info(f"Survey capture at ({pt['x']},{pt['y']}): {len(buf)} samples")
    elif mode == "delay":
        state.delay_calibrator.add_capture((pt["x"], pt["y"]), anchor_dists)
        done_msg["n_total"] = len(state.delay_calibrator.captures)
        log.info(f"Delay capture at ({pt['x']},{pt['y']}): {len(buf)} samples")
    elif mode == "coord":
        state.coord_captures.append({"expected": (pt["x"], pt["y"]), "measured": (mean_x, mean_y)})
        done_msg["n_total"] = len(state.coord_captures)
        log.info(f"Coord capture at ({pt['x']},{pt['y']}): measured ({mean_x:.3f},{mean_y:.3f})")

    await broadcast(done_msg)
    await broadcast(state.status_msg())

# ══════════════════════════════════════════════════════════════════════════════
#  Discovery UDP
# ══════════════════════════════════════════════════════════════════════════════
class DiscoveryProtocol(asyncio.DatagramProtocol):
    def __init__(self): self._transport = None
    def connection_made(self, transport): self._transport = transport
    def datagram_received(self, data: bytes, addr):
        try: pkt = json.loads(data.decode())
        except: return
        if pkt.get("beacon") != "anchor": return
        anchor_id = pkt.get("id")
        if anchor_id not in ("A1", "A2", "A3"): return
        ip = addr[0]
        if anchor_id not in state.anchor_ips:
            state.anchor_ips[anchor_id] = ip
            state.anchor_delays[anchor_id] = int(pkt.get("adelay", config.DEFAULT_ADELAY))
            log.info(f"Discovered {anchor_id} @ {ip} delay={state.anchor_delays[anchor_id]}")
            try:
                ack = f"ACK:{anchor_id}".encode()
                self._transport.sendto(ack, (ip, config.ANCHOR_CMD_PORT))
            except Exception as e: log.warning(f"ACK failed: {e}")
            asyncio.ensure_future(broadcast(state.status_msg()))
            if state.all_anchors_discovered():
                _save_discovered(); log.info("All 3 anchors discovered!")

def _save_discovered():
    try:
        with open(config.DISCOVERED_ANCHORS_FILE, "w") as f:
            json.dump({"timestamp": datetime.now().isoformat(), "anchors": {k: {"ip": state.anchor_ips[k], "delay": state.anchor_delays[k]} for k in state.anchor_ips}}, f, indent=2)
    except Exception as e: log.warning(f"Could not save discovered: {e}")

# ══════════════════════════════════════════════════════════════════════════════
#  UDP command helper
# ══════════════════════════════════════════════════════════════════════════════
async def _send_cmd_udp(ip: str, port: int, msg: str, timeout: float = 2.0) -> str:
    loop = asyncio.get_event_loop()
    class _P(asyncio.DatagramProtocol):
        def __init__(self):
            self.fut = loop.create_future()
        def datagram_received(self, data, addr):
            if not self.fut.done(): self.fut.set_result(data.decode())
        def error_received(self, exc):
            if not self.fut.done(): self.fut.set_exception(exc)
        def connection_lost(self, exc):
            if not self.fut.done(): self.fut.set_result("CONN_LOST")
    transport, proto = await loop.create_datagram_endpoint(_P, remote_addr=(ip, port))
    try:
        transport.sendto(msg.encode())
        return await asyncio.wait_for(proto.fut, timeout=timeout)
    finally:
        transport.close()

# ══════════════════════════════════════════════════════════════════════════════
#  WebSocket + command handler
# ══════════════════════════════════════════════════════════════════════════════
async def ws_handler(ws):
    state.clients.add(ws)
    log.info(f"GUI connected: {ws.remote_address}")
    await ws.send(json.dumps(state.status_msg()))
    try:
        async for raw in ws:
            try: msg = json.loads(raw)
            except: await ws.send(json.dumps({"type": "error", "msg": "Invalid JSON"})); continue
            await handle_cmd(msg, ws)
    except websockets.exceptions.ConnectionClosed: pass
    finally: state.clients.discard(ws); log.info("GUI disconnected")

async def handle_cmd(msg: dict, ws):
    cmd = msg.get("cmd")

    if cmd == "get_status":
        await ws.send(json.dumps(state.status_msg()))

    elif cmd == "start_capture":
        if state.capturing:
            await ws.send(json.dumps({"type": "error", "msg": "Already capturing"})); return
        pt = msg.get("point", {"x": 0.0, "y": 0.0})
        n = int(msg.get("n", config.DEFAULT_CAPTURE_N))
        mode = msg.get("mode", "survey")
        state.capture_point = pt; state.capture_target = n; state.capture_mode = mode
        state.capture_buffer.clear(); state.capturing = True
        log.info(f"Capture [{mode}] at ({pt['x']},{pt['y']}) × {n}")
        await broadcast(state.status_msg())

    elif cmd == "stop_capture":
        if state.capturing: state.capturing = False; await _finish_capture()
        await broadcast(state.status_msg())

    # ── Survey (§1) ─────────────────────────────────────────
    elif cmd == "survey_compute":
        try:
            result = state.survey_solver.solve(apply_scale=True)
            state.survey_result = result
            write_anchors_json(result.transformed_coords)
            # Reload pipeline
            global cfg_global, pipeline
            p = Path(config.ANCHORS_JSON_FILE)
            if p.exists():
                cfg_global = AnchorConfig.from_json(str(p))
                pipeline = RtlsPipeline(cfg_global, state.coord_compensator)
            out = {"type": "survey_result", "surveyed": result.surveyed_coords,
                   "transformed": result.transformed_coords, "reference": result.reference_coords,
                   "per_anchor_residual": result.per_anchor_residual, "rms_error": result.rms_error,
                   "rotation_deg": result.rotation_deg, "scale_factor": result.scale_factor,
                   "success": result.success}
            await broadcast(out)
            log.info(f"Survey done: RMS={result.rms_error*100:.2f} cm")
        except Exception as e:
            await ws.send(json.dumps({"type": "error", "msg": str(e)}))

    elif cmd == "survey_clear":
        state.survey_solver.clear(); state.survey_result = None
        await broadcast({"type": "survey_cleared"}); await broadcast(state.status_msg())

    # ── Delay calibration (§2) ──────────────────────────────
    elif cmd == "delay_optimize":
        try:
            initial = {k: state.anchor_delays.get(k, config.DEFAULT_ADELAY) for k in ["A1", "A2", "A3"]}
            result = state.delay_calibrator.solve(initial)
            state.delay_result = result
            out = {"type": "delay_result", "anchor_corrections": result.anchor_corrections,
                   "new_delays": result.new_delays, "initial_delays": result.initial_delays,
                   "rmse_before": result.rmse_before, "rmse_after": result.rmse_after,
                   "n_points": result.n_points, "success": result.success}
            await broadcast(out)
            log.info(f"Delay cal: RMSE {result.rmse_before*100:.2f}→{result.rmse_after*100:.2f} cm")
        except Exception as e:
            await ws.send(json.dumps({"type": "error", "msg": str(e)}))

    elif cmd == "delay_apply":
        delays = msg.get("delays", {})
        if not delays and state.delay_result: delays = state.delay_result.new_delays
        if not delays: await ws.send(json.dumps({"type": "error", "msg": "No delays"})); return
        responses = {}
        for aid, dval in delays.items():
            ip = state.anchor_ips.get(aid)
            if not ip: responses[aid] = "NOT_DISCOVERED"; continue
            try:
                resp = await _send_cmd_udp(ip, config.ANCHOR_CMD_PORT, f"SET_ADELAY:{dval}", timeout=2.0)
                responses[aid] = resp
                if resp.startswith("OK:"): state.anchor_delays[aid] = int(resp.split(":")[1])
                log.info(f"Applied {aid}: {resp}")
            except Exception as e: responses[aid] = f"ERROR:{e}"
        await broadcast({"type": "delay_apply_result", "responses": responses})
        await broadcast(state.status_msg())

    elif cmd == "delay_clear":
        state.delay_calibrator.clear(); state.delay_result = None
        await broadcast({"type": "delay_cleared"}); await broadcast(state.status_msg())

    # ── Coord compensation (§3) ─────────────────────────────
    elif cmd == "coord_train":
        if len(state.coord_captures) < 3:
            await ws.send(json.dumps({"type": "error", "msg": f"Need ≥3 points (have {len(state.coord_captures)})"})); return
        try:
            expected = np.array([c["expected"] for c in state.coord_captures])
            measured = np.array([c["measured"] for c in state.coord_captures])
            result = state.coord_compensator.train(expected, measured)
            state.coord_compensator.enabled = True
            state.coord_compensator.save()
            state.coord_result = result
            # Update pipeline
            if pipeline: pipeline.compensator = state.coord_compensator
            out = {"type": "coord_result", "n_points": result.n_control_points,
                   "rmse_before": result.rmse_before, "rmse_after": result.rmse_after,
                   "max_error_before": result.max_error_before,
                   "per_point_errors_before": result.per_point_errors_before,
                   "success": result.success, "enabled": True}
            await broadcast(out)
            log.info(f"Coord comp trained: RMSE {result.rmse_before*100:.2f}→{result.rmse_after*100:.4f} cm")
        except Exception as e:
            await ws.send(json.dumps({"type": "error", "msg": str(e)}))

    elif cmd == "coord_toggle":
        state.coord_compensator.enabled = bool(msg.get("enabled", False))
        await broadcast(state.status_msg())

    elif cmd == "coord_clear":
        state.coord_captures.clear(); state.coord_result = None
        state.coord_compensator = CoordCompensator()
        if pipeline: pipeline.compensator = state.coord_compensator
        await broadcast({"type": "coord_cleared"}); await broadcast(state.status_msg())

    elif cmd == "tune":
        if pipeline: pipeline.update_tuning(msg)

    elif cmd == "save_session":
        path = _save_session()
        await ws.send(json.dumps({"type": "session_saved", "path": path}))

    else:
        await ws.send(json.dumps({"type": "error", "msg": f"Unknown: {cmd}"}))

def _save_session() -> str:
    os.makedirs(config.CALIB_LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    path = os.path.join(config.CALIB_LOG_DIR, f"session_{ts}.json")
    session = {"timestamp": datetime.now().isoformat(),
               "anchor_ips": state.anchor_ips, "anchor_delays": state.anchor_delays,
               "survey_captures": len(state.survey_solver.captures),
               "delay_captures": len(state.delay_calibrator.captures),
               "coord_captures": len(state.coord_captures)}
    if state.survey_result:
        session["survey"] = {"transformed": state.survey_result.transformed_coords, "rms": state.survey_result.rms_error}
    if state.delay_result:
        session["delay"] = {"new_delays": state.delay_result.new_delays, "rmse_before": state.delay_result.rmse_before, "rmse_after": state.delay_result.rmse_after}
    with open(path, "w") as f: json.dump(session, f, indent=2)
    log.info(f"Session saved → {path}"); return path

# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════
SCRIPT_DIR = Path(__file__).resolve().parent

async def main():
    global pipeline, cfg_global, log_fh
    log.info("=== UWB v2 Calibration Server ===")

    # Load anchor config
    anchors_path = Path(config.ANCHORS_JSON_FILE)
    if anchors_path.exists():
        cfg_global = AnchorConfig.from_json(str(anchors_path))
        log.info(f"Loaded {len(cfg_global.ids)} anchors from {anchors_path}")
    else:
        cfg_global = AnchorConfig()
        log.info("Using built-in default anchors")

    # Load coord compensation if available
    comp_path = Path(config.COORD_COMPENSATION_FILE)
    if comp_path.exists():
        try:
            state.coord_compensator = CoordCompensator.load(str(comp_path))
        except Exception as e:
            log.warning(f"Could not load compensation: {e}")

    pipeline = RtlsPipeline(cfg_global, state.coord_compensator)

    # Load cached anchors
    try:
        with open(config.DISCOVERED_ANCHORS_FILE) as f:
            cached = json.load(f)
        for k, v in cached["anchors"].items():
            state.anchor_ips[k] = v["ip"]
            state.anchor_delays[k] = v.get("delay", config.DEFAULT_ADELAY)
        log.info("Loaded cached anchor IPs")
    except FileNotFoundError: log.info("No cached anchors — waiting for beacons")
    except Exception as e: log.warning(f"Cache load error: {e}")

    # Log file
    log_name = SCRIPT_DIR / f"rtls_log_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    log_fh = open(log_name, 'w')

    loop = asyncio.get_event_loop()
    tag_transport, _ = await loop.create_datagram_endpoint(TagUDPProtocol, local_addr=("0.0.0.0", config.TAG_UDP_PORT))
    disc_transport, _ = await loop.create_datagram_endpoint(DiscoveryProtocol, local_addr=("0.0.0.0", config.BEACON_PORT), allow_broadcast=True)
    ws_server = await websockets.serve(ws_handler, "0.0.0.0", config.WS_PORT)

    log.info(f"Tag UDP: {config.TAG_UDP_PORT}  Beacon: {config.BEACON_PORT}  WS: {config.WS_PORT}")
    log.info("Open gui.html in your browser")

    try: await asyncio.Future()
    finally: tag_transport.close(); disc_transport.close(); ws_server.close()

if __name__ == "__main__":
    asyncio.run(main())

"""
calibration_server.py — Asyncio-based UDP/WebSocket server for UWB calibration.

Usage
-----
    pip install websockets
    cd Tests/2x2mtest2_calibration/Code
    python calibration_server.py

What it does
------------
  1. TAG UDP relay    — listens :4210 for tag position packets, forwards to GUI via WS
  2. Discovery        — listens :4213 for anchor beacons, auto-discovers IPs, sends ACK
  3. WebSocket server — listens :8765 for GUI connections; dispatches GUI commands
  4. Command handler  — forwards SET_ADELAY commands to anchors via UDP :4211

GUI → server commands (JSON over WebSocket):
  {"cmd":"get_status"}
  {"cmd":"start_capture","point":{"x":0.0,"y":1.0},"n":200}
  {"cmd":"stop_capture"}
  {"cmd":"optimize","optimize_positions":false}
  {"cmd":"apply","delays":{"A1":16562,"A2":16551,"A3":16558}}
  {"cmd":"clear_session"}
  {"cmd":"save_session"}

Server → GUI messages (JSON over WebSocket):
  {"type":"position",  "x":..., "y":..., ...}
  {"type":"status",    "anchors":{...}, "capture":{...}, "session":{...}}
  {"type":"capture_progress","n":42,"target":200,"point":{...}}
  {"type":"optimize_result", ...OptimizationResult fields...}
  {"type":"apply_result",    "responses":{...}}
  {"type":"error",           "msg":"..."}
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import socket
import sys
import time
from collections import defaultdict
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Optional

import websockets

# ── path setup so config / optimizer can be imported directly ──
sys.path.insert(0, str(Path(__file__).parent))
import config
from calibration_optimizer import (
    CalibPoint,
    OptimizationResult,
    optimize_delays,
)
import numpy as np

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("calib_server")

# ══════════════════════════════════════════════════════════════
#  Shared state
# ══════════════════════════════════════════════════════════════
class ServerState:
    def __init__(self):
        # Discovery
        self.anchor_ips: dict[str, str]  = {}   # "A1" → "192.168.x.x"
        self.anchor_delays: dict[str, int] = {}  # "A1" → current delay (from beacon)

        # Capture
        self.capturing: bool = False
        self.capture_point: Optional[dict] = None   # {"x":..., "y":...}
        self.capture_target: int = config.DEFAULT_CAPTURE_N
        self.capture_buffer: list[dict] = []         # raw UDP packets

        # Session (all captured points)
        self.session_points: list[CalibPoint] = []
        self.last_result: Optional[OptimizationResult] = None

        # Connected GUI clients
        self.clients: set = set()

    def all_anchors_discovered(self) -> bool:
        return all(k in self.anchor_ips for k in ["A1", "A2", "A3"])

    def status_msg(self) -> dict:
        return {
            "type": "status",
            "anchors": {
                k: {"ip": self.anchor_ips.get(k, None),
                    "delay": self.anchor_delays.get(k, config.DEFAULT_ADELAY)}
                for k in ["A1", "A2", "A3"]
            },
            "capture": {
                "active": self.capturing,
                "point": self.capture_point,
                "n": len(self.capture_buffer),
                "target": self.capture_target,
            },
            "session": {
                "n_points": len(self.session_points),
            },
        }


state = ServerState()


# ══════════════════════════════════════════════════════════════
#  Broadcast helpers
# ══════════════════════════════════════════════════════════════
async def broadcast(msg: dict):
    if not state.clients:
        return
    data = json.dumps(msg)
    await asyncio.gather(
        *[ws.send(data) for ws in list(state.clients)],
        return_exceptions=True,
    )


# ══════════════════════════════════════════════════════════════
#  1. Tag UDP relay  (:4210)
# ══════════════════════════════════════════════════════════════
class TagUDPProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data: bytes, addr):
        try:
            pkt = json.loads(data.decode())
        except Exception:
            return

        pkt["type"] = "position"

        # ── Capture accumulation ───────────────────────────────
        if state.capturing and state.capture_point is not None:
            state.capture_buffer.append(pkt)
            n = len(state.capture_buffer)
            asyncio.ensure_future(broadcast({
                "type": "capture_progress",
                "n": n,
                "target": state.capture_target,
                "point": state.capture_point,
            }))
            if n >= state.capture_target:
                asyncio.ensure_future(_finish_capture())

        asyncio.ensure_future(broadcast(pkt))


async def _finish_capture():
    state.capturing = False
    buf = state.capture_buffer[:]
    state.capture_buffer.clear()

    if not buf:
        return

    pt = state.capture_point
    xs  = [p["x"]  for p in buf]
    ys  = [p["y"]  for p in buf]
    d0s = [p["d0"] for p in buf]
    d1s = [p["d1"] for p in buf]
    d2s = [p["d2"] for p in buf]

    cp = CalibPoint(
        x_true  = pt["x"],
        y_true  = pt["y"],
        n_samples = len(buf),
        mean_d0 = float(np.mean(d0s)), std_d0 = float(np.std(d0s)),
        mean_d1 = float(np.mean(d1s)), std_d1 = float(np.std(d1s)),
        mean_d2 = float(np.mean(d2s)), std_d2 = float(np.std(d2s)),
        mean_x  = float(np.mean(xs)),  std_x  = float(np.std(xs)),
        mean_y  = float(np.mean(ys)),  std_y  = float(np.std(ys)),
    )
    state.session_points.append(cp)
    log.info(f"Capture complete: point ({pt['x']}, {pt['y']}) "
             f"— {len(buf)} samples, mean d=[{cp.mean_d0:.3f},{cp.mean_d1:.3f},{cp.mean_d2:.3f}]")

    await broadcast({
        "type":    "capture_done",
        "point":   pt,
        "n":       len(buf),
        "mean_d0": cp.mean_d0, "std_d0": cp.std_d0,
        "mean_d1": cp.mean_d1, "std_d1": cp.std_d1,
        "mean_d2": cp.mean_d2, "std_d2": cp.std_d2,
        "mean_x":  cp.mean_x,  "std_x":  cp.std_x,
        "mean_y":  cp.mean_y,  "std_y":  cp.std_y,
    })
    await broadcast(state.status_msg())


# ══════════════════════════════════════════════════════════════
#  2. Discovery UDP  (:4213)
# ══════════════════════════════════════════════════════════════
class DiscoveryProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self._transport = None

    def connection_made(self, transport):
        self._transport = transport

    def datagram_received(self, data: bytes, addr):
        try:
            pkt = json.loads(data.decode())
        except Exception:
            return
        if pkt.get("beacon") != "anchor":
            return

        anchor_id = pkt.get("id")
        if anchor_id not in ("A1", "A2", "A3"):
            return

        ip = addr[0]
        if anchor_id not in state.anchor_ips:
            state.anchor_ips[anchor_id]    = ip
            state.anchor_delays[anchor_id] = int(pkt.get("adelay", config.DEFAULT_ADELAY))
            log.info(f"Discovered {anchor_id} @ {ip}  delay={state.anchor_delays[anchor_id]}")

            # Send ACK to stop beaconing
            ack_msg = f"ACK:{anchor_id}".encode()
            try:
                self._transport.sendto(ack_msg, (ip, config.ANCHOR_CMD_PORT))
            except Exception as e:
                log.warning(f"ACK send failed: {e}")

            asyncio.ensure_future(broadcast(state.status_msg()))

            if state.all_anchors_discovered():
                _save_discovered()
                log.info("All 3 anchors discovered!")


def _save_discovered():
    try:
        with open(config.DISCOVERED_ANCHORS_FILE, "w") as f:
            json.dump({
                "timestamp": datetime.now().isoformat(),
                "anchors": {k: {"ip": state.anchor_ips[k],
                                 "delay": state.anchor_delays[k]}
                            for k in state.anchor_ips},
            }, f, indent=2)
    except Exception as e:
        log.warning(f"Could not save discovered_anchors.json: {e}")


# ══════════════════════════════════════════════════════════════
#  3. WebSocket server  (:8765)
# ══════════════════════════════════════════════════════════════
async def ws_handler(ws):
    state.clients.add(ws)
    log.info(f"GUI connected: {ws.remote_address}")
    await ws.send(json.dumps(state.status_msg()))

    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except Exception:
                await ws.send(json.dumps({"type": "error", "msg": "Invalid JSON"}))
                continue
            await handle_gui_command(msg, ws)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        state.clients.discard(ws)
        log.info(f"GUI disconnected: {ws.remote_address}")


# ══════════════════════════════════════════════════════════════
#  4. GUI command handler
# ══════════════════════════════════════════════════════════════
async def handle_gui_command(msg: dict, ws):
    cmd = msg.get("cmd")

    # ── get_status ────────────────────────────────────────────
    if cmd == "get_status":
        await ws.send(json.dumps(state.status_msg()))

    # ── start_capture ─────────────────────────────────────────
    elif cmd == "start_capture":
        if state.capturing:
            await ws.send(json.dumps({"type": "error", "msg": "Already capturing"}))
            return
        pt = msg.get("point", {"x": 0.0, "y": 0.0})
        n  = int(msg.get("n", config.DEFAULT_CAPTURE_N))
        state.capture_point  = pt
        state.capture_target = n
        state.capture_buffer.clear()
        state.capturing = True
        log.info(f"Capture started: ({pt['x']}, {pt['y']}) × {n} samples")
        await broadcast(state.status_msg())

    # ── stop_capture ──────────────────────────────────────────
    elif cmd == "stop_capture":
        if state.capturing:
            state.capturing = False
            await _finish_capture()
        await broadcast(state.status_msg())

    # ── optimize ──────────────────────────────────────────────
    elif cmd == "optimize":
        if len(state.session_points) < 3:
            await ws.send(json.dumps({
                "type": "error",
                "msg": f"Need ≥3 calibration points (have {len(state.session_points)})"
            }))
            return
        opt_pos = bool(msg.get("optimize_positions", False))
        anchors = np.array([config.ANCHOR_COORDS[k] for k in ["A1", "A2", "A3"]])
        initial = {k: state.anchor_delays.get(k, config.DEFAULT_ADELAY)
                   for k in ["A1", "A2", "A3"]}
        log.info(f"Running optimizer ({len(state.session_points)} pts, "
                 f"opt_positions={opt_pos})")
        try:
            result = optimize_delays(
                state.session_points, anchors, initial,
                optimize_positions=opt_pos,
            )
            state.last_result = result
            out = {
                "type":                   "optimize_result",
                "new_delays":             result.new_delays,
                "anchor_corrections":     result.anchor_corrections,
                "rmse_before":            result.rmse_before,
                "rmse_after":             result.rmse_after,
                "max_error_before":       result.max_error_before,
                "max_error_after":        result.max_error_after,
                "per_point_errors_before":result.per_point_errors_before,
                "per_point_errors_after": result.per_point_errors_after,
                "optimizer_message":      result.optimizer_message,
                "success":                result.success,
                "nfev":                   result.nfev,
                "initial_delays":         initial,
            }
            await broadcast(out)
            log.info(f"Optimize done: RMSE {result.rmse_before*100:.2f} cm → "
                     f"{result.rmse_after*100:.2f} cm  success={result.success}")
        except Exception as e:
            await ws.send(json.dumps({"type": "error", "msg": str(e)}))

    # ── apply ─────────────────────────────────────────────────
    elif cmd == "apply":
        delays: dict[str, int] = msg.get("delays", {})
        if not delays:
            if state.last_result:
                delays = state.last_result.new_delays
            else:
                await ws.send(json.dumps({"type": "error",
                                           "msg": "No delays to apply"}))
                return

        responses = {}
        for anchor_id, delay_val in delays.items():
            ip = state.anchor_ips.get(anchor_id)
            if not ip:
                responses[anchor_id] = "NOT_DISCOVERED"
                continue
            try:
                resp = await _send_cmd_udp(ip, config.ANCHOR_CMD_PORT,
                                            f"SET_ADELAY:{delay_val}", timeout=2.0)
                responses[anchor_id] = resp
                if resp.startswith("OK:"):
                    state.anchor_delays[anchor_id] = int(resp.split(":")[1])
                log.info(f"Applied to {anchor_id} ({ip}): {resp}")
            except Exception as e:
                responses[anchor_id] = f"ERROR:{e}"
                log.warning(f"Apply failed for {anchor_id}: {e}")

        await broadcast({"type": "apply_result", "responses": responses})
        await broadcast(state.status_msg())

    # ── clear_session ─────────────────────────────────────────
    elif cmd == "clear_session":
        state.session_points.clear()
        state.last_result = None
        await broadcast(state.status_msg())

    # ── save_session ──────────────────────────────────────────
    elif cmd == "save_session":
        path = _save_session()
        await ws.send(json.dumps({"type": "session_saved", "path": path}))

    else:
        await ws.send(json.dumps({"type": "error", "msg": f"Unknown command: {cmd}"}))


# ══════════════════════════════════════════════════════════════
#  UDP command helper (send/receive with timeout)
# ══════════════════════════════════════════════════════════════
async def _send_cmd_udp(ip: str, port: int, msg: str,
                         timeout: float = 2.0) -> str:
    loop = asyncio.get_event_loop()

    class _Proto(asyncio.DatagramProtocol):
        def __init__(self):
            self.fut: asyncio.Future = loop.create_future()
        def datagram_received(self, data, addr):
            if not self.fut.done():
                self.fut.set_result(data.decode())
        def error_received(self, exc):
            if not self.fut.done():
                self.fut.set_exception(exc)
        def connection_lost(self, exc):
            if not self.fut.done():
                self.fut.set_result("CONN_LOST")

    transport, proto = await loop.create_datagram_endpoint(
        _Proto,
        remote_addr=(ip, port),
    )
    try:
        transport.sendto(msg.encode())
        return await asyncio.wait_for(proto.fut, timeout=timeout)
    finally:
        transport.close()


# ══════════════════════════════════════════════════════════════
#  Session save
# ══════════════════════════════════════════════════════════════
def _save_session() -> str:
    os.makedirs(config.CALIB_LOG_DIR, exist_ok=True)
    ts  = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    path = os.path.join(config.CALIB_LOG_DIR, f"calibration_{ts}.json")

    result = state.last_result
    session = {
        "session_id":       ts,
        "timestamp":        datetime.now().isoformat(),
        "geometry": {
            "anchors":    {k: list(v) for k, v in config.ANCHOR_COORDS.items()},
            "workspace":  config.WORKSPACE,
        },
        "initial_delays":   {k: config.DEFAULT_ADELAY for k in ["A1", "A2", "A3"]},
        "discovered_delays":state.anchor_delays,
        "optimized_delays": result.new_delays if result else None,
        "anchor_corrections":result.anchor_corrections if result else None,
        "rmse_before":      result.rmse_before if result else None,
        "rmse_after":       result.rmse_after  if result else None,
        "max_error_before": result.max_error_before if result else None,
        "max_error_after":  result.max_error_after  if result else None,
        "calibration_points": [
            {
                "x_true":  cp.x_true, "y_true": cp.y_true,
                "n_samples": cp.n_samples,
                "mean_x": cp.mean_x, "std_x": cp.std_x,
                "mean_y": cp.mean_y, "std_y": cp.std_y,
                "mean_d0": cp.mean_d0, "std_d0": cp.std_d0,
                "mean_d1": cp.mean_d1, "std_d1": cp.std_d1,
                "mean_d2": cp.mean_d2, "std_d2": cp.std_d2,
            }
            for cp in state.session_points
        ],
        "optimizer_info": {
            "method":  "least_squares (trf)",
            "success": result.success if result else None,
            "message": result.optimizer_message if result else None,
            "nfev":    result.nfev if result else None,
        } if result else None,
    }

    with open(path, "w") as f:
        json.dump(session, f, indent=2)
    log.info(f"Session saved → {path}")
    return path


# ══════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════
async def main():
    log.info("=== UWB Calibration Server ===")
    log.info(f"Tag UDP port:      {config.TAG_UDP_PORT}")
    log.info(f"Anchor CMD port:   {config.ANCHOR_CMD_PORT}")
    log.info(f"Beacon port:       {config.BEACON_PORT}")
    log.info(f"WebSocket port:    {config.WS_PORT}")

    # Try loading previously discovered anchors
    try:
        with open(config.DISCOVERED_ANCHORS_FILE) as f:
            cached = json.load(f)
        for k, v in cached["anchors"].items():
            state.anchor_ips[k]    = v["ip"]
            state.anchor_delays[k] = v.get("delay", config.DEFAULT_ADELAY)
        log.info(f"Loaded cached anchor IPs from {config.DISCOVERED_ANCHORS_FILE}")
    except FileNotFoundError:
        log.info("No cached anchor IPs — waiting for beacons...")
    except Exception as e:
        log.warning(f"Could not load cached anchors: {e}")

    loop = asyncio.get_event_loop()

    # Tag position UDP
    tag_transport, _ = await loop.create_datagram_endpoint(
        TagUDPProtocol,
        local_addr=("0.0.0.0", config.TAG_UDP_PORT),
    )

    # Discovery UDP
    disc_transport, _ = await loop.create_datagram_endpoint(
        DiscoveryProtocol,
        local_addr=("0.0.0.0", config.BEACON_PORT),
        allow_broadcast=True,
    )

    # WebSocket server
    ws_server = await websockets.serve(ws_handler, "0.0.0.0", config.WS_PORT)

    log.info("Server running. Open gui_calibration.html in a browser.")
    log.info("Waiting for anchor beacons on port %d ...", config.BEACON_PORT)

    try:
        await asyncio.Future()   # run forever
    finally:
        tag_transport.close()
        disc_transport.close()
        ws_server.close()


if __name__ == "__main__":
    asyncio.run(main())

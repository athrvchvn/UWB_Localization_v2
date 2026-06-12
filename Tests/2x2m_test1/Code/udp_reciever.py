"""
UWB UDP → WebSocket relay
Listens for UDP packets from the ESP32 tag and forwards them
to any connected browser clients over WebSocket.

ESP32 packet schema (JSON):
  x, y     — raw trilateration position (m)
  ex, ey   — EKF-filtered position (m)
  rmse     — trilateration RMSE (m)
  d0,d1,d2 — per-anchor filtered distances (m)

Install dependency:  pip install websockets
Run:                 python udp_reciever.py
Then open:          index.html in your browser
"""

import asyncio
import socket
import json
import websockets

connected_clients: set = set()

UDP_PORT = 4210
WS_PORT  = 8765

# Required keys in every ESP32 packet
EXPECTED_KEYS = {"x", "y", "ex", "ey", "rmse", "d0", "d1", "d2"}

async def ws_handler(websocket):
    connected_clients.add(websocket)
    print(f"[WS]  Browser connected  ({len(connected_clients)} total)")
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.discard(websocket)
        print(f"[WS]  Browser disconnected  ({len(connected_clients)} total)")

async def udp_reader():
    # Blocking socket read offloaded to a thread — no EAGAIN spam
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", UDP_PORT))
    # Keep socket BLOCKING — recv() waits until a packet arrives
    sock.setblocking(True)

    loop = asyncio.get_event_loop()
    print(f"[UDP] Listening on port {UDP_PORT}...")

    while True:
        try:
            # run_in_executor lets the blocking recv() wait in a thread
            # without freezing the asyncio event loop
            data, addr = await loop.run_in_executor(
                None, lambda: sock.recvfrom(256)
            )
            payload = data.decode().strip()
            parsed  = json.loads(payload)

            # Validate that all expected keys are present
            missing = EXPECTED_KEYS - parsed.keys()
            if missing:
                print(f"[UDP] Incomplete packet from {addr[0]}, missing: {missing}")
                continue

            # Log: raw trilateration | EKF output | RMSE | anchor distances
            print(
                f"[UDP] {addr[0]}"
                f"  RAW=({parsed['x']:.3f},{parsed['y']:.3f})"
                f"  EKF=({parsed['ex']:.3f},{parsed['ey']:.3f})"
                f"  RMSE={parsed['rmse']:.4f}"
                f"  d=[{parsed['d0']:.3f},{parsed['d1']:.3f},{parsed['d2']:.3f}]"
            )

            if connected_clients:
                await asyncio.gather(
                    *[c.send(payload) for c in list(connected_clients)],
                    return_exceptions=True
                )
        except json.JSONDecodeError:
            print("[UDP] Bad packet, skipping")
        except Exception as e:
            print(f"[UDP] Error: {e}")
            await asyncio.sleep(0.1)

async def main():
    print(f"[WS]  WebSocket server on ws://localhost:{WS_PORT}")
    print(f"      Open index.html in your browser\n")
    # websockets v14+ API
    async with websockets.serve(ws_handler, "localhost", WS_PORT):
        await udp_reader()

if __name__ == "__main__":
    asyncio.run(main())
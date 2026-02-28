#!/usr/bin/env python3
"""
FSM-Pilot Mock Operator Client
Simulates an operator connecting to the signaling server,
connecting to a vehicle, sending control commands, and receiving telemetry.

Usage:
    python3 mock_operator.py [signaling_url] [vehicle_id]

Defaults:
    signaling_url = ws://localhost:8080
    vehicle_id    = FSM-TEST-01
"""

import asyncio
import json
import math
import sys
import time

try:
    import websockets
except ImportError:
    print("ERROR: websockets package not installed. Run: pip3 install websockets")
    sys.exit(1)


SIGNALING_URL = sys.argv[1] if len(sys.argv) > 1 else "ws://localhost:8080"
VEHICLE_ID    = sys.argv[2] if len(sys.argv) > 2 else "FSM-TEST-01"

telemetry_received = 0
control_sent       = 0
connected_to_vehicle = False
vehicle_relay_ready  = False
latency_ms = 0.0
ping_seq = 0


async def run():
    global telemetry_received, control_sent, connected_to_vehicle
    global vehicle_relay_ready, latency_ms, ping_seq

    print(f"[OPERATOR] Connecting to {SIGNALING_URL}")
    async with websockets.connect(SIGNALING_URL) as ws:
        # Register as operator
        await ws.send(json.dumps({
            "type":        "register",
            "client_type": "operator",
        }))

        last_ctrl = time.time()
        last_ping = time.time()

        while True:
            try:
                msg_raw = await asyncio.wait_for(ws.recv(), timeout=0.02)
                msg = json.loads(msg_raw)
                mtype = msg.get("type", "")

                if mtype == "registered":
                    client_id = msg.get("client_id", "")
                    print(f"[OPERATOR] Registered: client_id={client_id}")
                    # Connect to vehicle
                    await ws.send(json.dumps({
                        "type":       "connect",
                        "vehicle_id": VEHICLE_ID,
                    }))
                    print(f"[OPERATOR] Requesting connection to {VEHICLE_ID}")
                    connected_to_vehicle = True

                elif mtype == "vehicle_relay_ready" or mtype == "relay_ready":
                    vehicle_relay_ready = True
                    print(f"[OPERATOR] Vehicle {VEHICLE_ID} relay ready!")

                elif mtype == "vehicle_online":
                    print(f"[OPERATOR] Vehicle online: {msg.get('vehicle_id')}")

                elif mtype == "telemetry":
                    telemetry_received += 1
                    d = msg.get("data", {})
                    if telemetry_received % 30 == 0:
                        print(f"[OPERATOR] Telemetry #{telemetry_received}: "
                              f"speed={d.get('speed_mps', 0):.1f}m/s "
                              f"battery={d.get('battery_pct', 0):.0f}% "
                              f"RTT={latency_ms:.1f}ms")

                elif mtype == "pong":
                    orig_t = msg.get("timestamp", 0)
                    now_ns = time.time_ns()
                    latency_ms = (now_ns - orig_t) / 1e6

                elif mtype == "peer_disconnected":
                    print(f"[OPERATOR] Vehicle disconnected")
                    connected_to_vehicle = False
                    vehicle_relay_ready  = False

                elif mtype == "error":
                    err = msg.get("message", "")
                    print(f"[OPERATOR] Error: {err}")
                    if "not found" in err.lower():
                        print(f"[OPERATOR] Vehicle {VEHICLE_ID} not online yet, waiting...")

            except asyncio.TimeoutError:
                pass

            now = time.time()

            # Send control command at 20 Hz when connected to vehicle
            if vehicle_relay_ready and now - last_ctrl >= 0.05:
                last_ctrl = now
                t = now
                cmd = {
                    "type":       "control",
                    "vehicle_id": VEHICLE_ID,
                    "data": {
                        "steering":     round(0.1 * math.sin(t * 0.3), 3),
                        "throttle":     0.4,
                        "brake":        0.0,
                        "gear":         3,
                        "turn_signal":  0,
                        "emergency":    False,
                        "timestamp_ns": time.time_ns(),
                        "sequence":     control_sent,
                    }
                }
                await ws.send(json.dumps(cmd))
                control_sent += 1

            # Send ping every 2 seconds for latency measurement
            if connected_to_vehicle and now - last_ping >= 2.0:
                last_ping = now
                ping_seq += 1
                await ws.send(json.dumps({
                    "type":       "ping",
                    "sequence":   ping_seq,
                    "timestamp":  time.time_ns(),
                }))


async def main():
    backoff = 1
    while True:
        try:
            await run()
        except (ConnectionRefusedError, OSError) as e:
            print(f"[OPERATOR] Connection failed: {e}. Retrying in {backoff}s...")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30)
        except KeyboardInterrupt:
            print("[OPERATOR] Stopping")
            break
        except Exception as e:
            print(f"[OPERATOR] Unexpected error: {e}")
            await asyncio.sleep(backoff)


if __name__ == "__main__":
    asyncio.run(main())

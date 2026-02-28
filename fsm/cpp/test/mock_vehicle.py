#!/usr/bin/env python3
"""
FSM-Pilot Mock Vehicle Client
Simulates a vehicle connecting to the signaling server and sending telemetry.

Usage:
    python3 mock_vehicle.py [signaling_url] [vehicle_id]

Defaults:
    signaling_url = ws://localhost:8080
    vehicle_id    = FSM-TEST-01
"""

import asyncio
import json
import math
import random
import sys
import time

try:
    import websockets
except ImportError:
    print("ERROR: websockets package not installed. Run: pip3 install websockets")
    sys.exit(1)


SIGNALING_URL = sys.argv[1] if len(sys.argv) > 1 else "ws://localhost:8080"
VEHICLE_ID    = sys.argv[2] if len(sys.argv) > 2 else "FSM-TEST-01"

# Simulated vehicle state
state = {
    "speed_mps":    0.0,
    "steering_rad": 0.0,
    "gear":         3,
    "battery_pct":  85,
    "latitude":     30.0000,
    "longitude":    120.0000,
    "heading_rad":  0.0,
    "t":            0.0,
}

last_control = None
telemetry_count = 0
control_count = 0


def simulate_step(dt: float) -> None:
    """Advance simulated vehicle state by dt seconds."""
    s = state
    s["t"] += dt

    # Sine-wave speed profile
    s["speed_mps"] = max(0.0, 5.0 + 3.0 * math.sin(s["t"] * 0.2))

    # Gentle steering oscillation
    s["steering_rad"] = 0.05 * math.sin(s["t"] * 0.5)

    # Integrate position
    s["latitude"]  += s["speed_mps"] * math.cos(s["heading_rad"]) * dt * 9e-6
    s["longitude"] += s["speed_mps"] * math.sin(s["heading_rad"]) * dt * 9e-6
    s["heading_rad"] = (s["heading_rad"] + s["steering_rad"] * dt) % (2 * math.pi)

    # Battery drain
    s["battery_pct"] = max(0, s["battery_pct"] - 0.0001 * dt)

    # Apply control commands if any
    if last_control:
        d = last_control.get("data", {})
        if d.get("emergency"):
            s["speed_mps"] = 0.0
        else:
            target = d.get("throttle", 0.5) * 10.0 - d.get("brake", 0.0) * 10.0
            s["speed_mps"] += (target - s["speed_mps"]) * 0.1
            s["speed_mps"] = max(0.0, min(s["speed_mps"], 15.0))


def build_telemetry() -> dict:
    now_ns = time.time_ns()
    return {
        "type":       "telemetry",
        "vehicle_id": VEHICLE_ID,
        "sequence":   telemetry_count,
        "data": {
            "speed_mps":    round(state["speed_mps"], 3),
            "steering_rad": round(state["steering_rad"], 4),
            "gear":         state["gear"],
            "battery_pct":  round(state["battery_pct"], 1),
            "latitude":     state["latitude"],
            "longitude":    state["longitude"],
            "heading_rad":  round(state["heading_rad"], 4),
            "latency_ms":   0.0,
            "timestamp_ns": now_ns,
        }
    }


async def run():
    global telemetry_count, control_count, last_control

    print(f"[VEHICLE] Connecting to {SIGNALING_URL} as {VEHICLE_ID}")
    async with websockets.connect(SIGNALING_URL) as ws:
        # Register
        await ws.send(json.dumps({
            "type":        "register",
            "client_type": "vehicle",
            "vehicle_id":  VEHICLE_ID,
        }))
        print(f"[VEHICLE] Registered")

        # Receive loop + telemetry loop
        last_telem = time.time()
        last_sim   = time.time()

        while True:
            # Try to receive a message (non-blocking poll)
            try:
                msg_raw = await asyncio.wait_for(ws.recv(), timeout=0.01)
                msg = json.loads(msg_raw)
                mtype = msg.get("type", "")

                if mtype == "registered":
                    print(f"[VEHICLE] Server assigned client_id={msg.get('client_id')}")
                elif mtype == "create_offer":
                    operator_id = msg.get("operator_id", "")
                    print(f"[VEHICLE] Operator connected: {operator_id}")
                    # Acknowledge relay mode
                    await ws.send(json.dumps({
                        "type":       "relay_ready",
                        "vehicle_id": VEHICLE_ID,
                    }))
                elif mtype == "control":
                    control_count += 1
                    last_control = msg
                    d = msg.get("data", {})
                    if control_count % 20 == 0:
                        print(f"[VEHICLE] Control #{control_count}: "
                              f"steer={d.get('steering', 0):.2f} "
                              f"throttle={d.get('throttle', 0):.2f} "
                              f"brake={d.get('brake', 0):.2f}")
                elif mtype == "ping":
                    # Respond with pong
                    await ws.send(json.dumps({
                        "type":      "pong",
                        "sequence":  msg.get("sequence", 0),
                        "timestamp": msg.get("timestamp", 0),
                    }))
                elif mtype == "peer_disconnected":
                    print("[VEHICLE] Operator disconnected")
                    last_control = None
                elif mtype == "error":
                    print(f"[VEHICLE] Error: {msg.get('message')}")

            except asyncio.TimeoutError:
                pass

            # Simulate physics
            now = time.time()
            dt  = now - last_sim
            last_sim = now
            simulate_step(dt)

            # Send telemetry at 10 Hz
            if now - last_telem >= 0.1:
                last_telem = now
                telem = build_telemetry()
                telemetry_count += 1
                await ws.send(json.dumps(telem))
                if telemetry_count % 50 == 0:
                    print(f"[VEHICLE] Sent {telemetry_count} telemetry frames, "
                          f"speed={state['speed_mps']:.1f}m/s "
                          f"battery={state['battery_pct']:.0f}%")


async def main():
    backoff = 1
    while True:
        try:
            await run()
        except (ConnectionRefusedError, OSError) as e:
            print(f"[VEHICLE] Connection failed: {e}. Retrying in {backoff}s...")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30)
        except KeyboardInterrupt:
            print("[VEHICLE] Stopping")
            break
        except Exception as e:
            print(f"[VEHICLE] Unexpected error: {e}")
            await asyncio.sleep(backoff)


if __name__ == "__main__":
    asyncio.run(main())

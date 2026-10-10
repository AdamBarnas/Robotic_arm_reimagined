"""Send targets to a running robot_assembly.py over UDP and print the joint state.

Usage:
    python arm_client.py base_yaw=0.5 shoulder=0.3 gripper=1
    python arm_client.py                       # only read the state

From Python (e.g. the notebook):
    from arm_client import send, move_linear
    state = send(elbow=-1.2, gripper=0)                       # jump to the target
    state = move_linear({"elbow": -1.2, "gripper": 1}, 2.0)   # get there in 2 s
"""
import json
import socket
import sys
import time

import numpy as np

ADDRESS = ("127.0.0.1", 5005)
COMMAND_NAMES = ("base_yaw", "shoulder", "elbow", "wrist", "gripper")


def send(address=ADDRESS, timeout=1.0, **targets):
    """Send targets (rad, gripper 0..1) and return the arm's joint state as a dict."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(timeout)
        sock.sendto(json.dumps(targets).encode(), address)
        data, _ = sock.recvfrom(4096)
    return json.loads(data)


def plan_linear_trajectory(start, end, duration, dt=0.02):
    """Linear interpolation in joint space from start to end in duration seconds.

    start, end: dicts {joint name: value} (rad, gripper 0..1); joints missing from
    end keep their start value. Every joint starts and finishes at the same time.
    Returns (times, waypoints): times from 0 to duration with step ~dt, and one
    dict of joint values per time.
    """
    names = [name for name in COMMAND_NAMES if name in start]
    q0 = np.array([start[name] for name in names], dtype=float)
    q1 = np.array([end.get(name, start[name]) for name in names], dtype=float)

    steps = max(1, int(np.ceil(duration / dt))) if duration > 0 else 1
    times = np.linspace(0.0, max(duration, 0.0), steps + 1)
    s = times / times[-1] if duration > 0 else np.ones_like(times)  # 0 -> 1
    q = q0 + np.outer(s, q1 - q0)
    return times, [dict(zip(names, row.tolist())) for row in q]


def move_linear(end, duration, dt=0.02, start=None, address=ADDRESS):
    """Move the arm along plan_linear_trajectory and return the final joint state.

    start defaults to the arm's current position, read from the simulation.
    """
    if start is None:
        start = send(address)
    times, waypoints = plan_linear_trajectory(start, end, duration, dt)
    t0 = time.monotonic()
    for t, waypoint in zip(times, waypoints):
        time.sleep(max(0.0, t0 + t - time.monotonic()))  # keep to the planned timing
        state = send(address, **waypoint)
    return state


if __name__ == "__main__":
    targets = {key: float(value) for key, value in (arg.split("=", 1) for arg in sys.argv[1:])}
    print(send(**targets))

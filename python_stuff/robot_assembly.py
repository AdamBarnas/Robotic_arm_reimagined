"""Compact robot arm assembled from the STL parts in ../stl.

4 revolute DOF (base yaw, shoulder, elbow, wrist) + gripper jaws (open/close).

All STL files share the Fusion assembly frame (mm, Y up), so every part is
loaded at its original coordinates and only re-expressed in the frame of the
link it belongs to. Joint pivots below were measured from the part outlines.

Usage:
    python python_stuff/robot_assembly.py          # sliders in the GUI
    python python_stuff/robot_assembly.py --demo   # scripted sine motion

External control: while running, the arm also listens for UDP packets with a JSON
object of targets (rad, gripper 0..1); every packet is answered with the joint state:
    python python_stuff/arm_client.py base_yaw=0.5 gripper=1
"""
import argparse
import json
import socket
import time
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

STL_DIR = Path(__file__).resolve().parents[1] / "stl"

# -----------------------------
# Fusion frame -> world frame
# -----------------------------
MESH_SCALE = 0.001                                   # meshes are in mm
FUSION_ORN = p.getQuaternionFromEuler([np.pi/2, 0, 0])  # Fusion Y (up) -> world Z
# Fusion point that becomes the world origin: base yaw axis, bottom of MB plates
FUSION_ORIGIN = np.array([-190.0, -76.0, -182.5])


def fusion_to_world(point_mm):
    """Fusion assembly point (mm) -> world position (m), at zero joint angles."""
    pos, _ = p.multiplyTransforms([0, 0, 0], FUSION_ORN,
                                  (np.asarray(point_mm) - FUSION_ORIGIN) * MESH_SCALE, [0, 0, 0, 1])
    return np.array(pos)


# Fusion +Z (the shoulder/elbow/wrist axis) in world coordinates = -Y
FUSION_Z_AXIS = [0, -1, 0]

# -----------------------------
# Robot description
# -----------------------------
# Joint pivots in Fusion coordinates (mm), measured from the STL outlines:
#   base   - centre of ring MB1R / base B1+B2, vertical axis
#   shoulder, elbow - centres of the round ends of M1/M2 (R = 20 mm)
#   wrist  - centre of the round end of F1/F2 and gripper body G1 (R = 15 mm)
#   jaws   - G3/G4 slide along Fusion Z, they touch at Z = -182.5 when closed
LINKS = [
    # name,      parent,     joint type,          pivot (mm),                axis,          parts,                 mass [kg]
    ("base_yaw", None,       p.JOINT_REVOLUTE,    [-190.0, -32.5, -182.5],   [0, 0, 1],     ["B1", "B2"],          0.15),
    ("shoulder", "base_yaw", p.JOINT_REVOLUTE,    [-190.0,   0.0, -182.5],   FUSION_Z_AXIS, ["M1", "M2"],          0.10),
    ("elbow",    "shoulder", p.JOINT_REVOLUTE,    [-100.0,   0.0, -182.5],   FUSION_Z_AXIS, ["F1", "F2", "F2C"],   0.10),
    ("wrist",    "elbow",    p.JOINT_REVOLUTE,    [-182.75, 35.62, -182.5],  FUSION_Z_AXIS, ["G1", "G2"],          0.04),
    ("jaw_left", "wrist",    p.JOINT_PRISMATIC,   [-206.0,  35.62, -182.5],  FUSION_Z_AXIS, ["G3"],                0.005),
    ("jaw_right", "wrist",   p.JOINT_PRISMATIC,   [-206.0,  35.62, -182.5],  [0, 1, 0],     ["G4"],                0.005),
]
# Static body: main box, ring, bottom plates, front panel, stabilising legs.
BASE_PARTS = ["MB1", "MB1R", "MB2", "MB3", "MB4", "S1", "NS1", "S2", "NS2"]
# S2/NS2 are byte-identical copies of S1/NS1 at the same place; the second leg
# mounts on the mirrored hole in MB1, so they are mirrored about the arm mid-plane.
MIRROR_PLANE_Z = -182.5
MIRRORED_PARTS = {"S2", "NS2"}

PART_COLORS = {
    "MB": [0.25, 0.25, 0.28, 1], "S": [0.6, 0.6, 0.6, 1], "NS": [0.6, 0.6, 0.6, 1],
    "B": [0.85, 0.45, 0.1, 1], "M": [0.2, 0.6, 0.85, 1], "F": [0.9, 0.75, 0.2, 1],
    "G": [0.3, 0.75, 0.35, 1],
}

# Joint limits (rad for revolute, m per jaw for prismatic); zero = pose from the STL files
# (forearm folded back over the upper arm). Adjust to the real servo ranges.
JOINT_LIMITS = {
    "base_yaw": (-np.pi/2, np.pi/2),
    "shoulder": (0, np.pi),
    "elbow":    (-np.pi, 0),
    "wrist":    (-np.pi/2, np.pi/2),
}
COMMAND_NAMES = ("base_yaw", "shoulder", "elbow", "wrist", "gripper")
JAW_TRAVEL = 0.010   # opening of each jaw at gripper = 1.0
JOINT_FORCE = 0.2    # N*m, roughly an MG996R
JAW_FORCE = 10.0     # N


def color_of(part):
    prefix = part.rstrip("0123456789CR")
    return PART_COLORS.get(prefix, [0.7, 0.7, 0.7, 1])


def mesh_shapes(parts, frame_pos_world):
    """Compound collision + visual shape of STL parts for a link whose frame sits at
    frame_pos_world (world orientation at zero joint angles)."""
    files = [str(STL_DIR / f"{part}.stl") for part in parts]
    # A mesh vertex v ends up at FUSION_ORN * (v - FUSION_ORIGIN) * scale, so the mesh
    # frame inside the link is that transform shifted by the link frame position.
    # Mirrored parts flip Fusion Z (v -> v with z' = 2*plane - z) via a negative scale.
    scales, offsets = [], []
    for part in parts:
        origin = FUSION_ORIGIN.copy()
        scale = [MESH_SCALE]*3
        if part in MIRRORED_PARTS:
            origin[2] -= 2 * MIRROR_PLANE_Z
            scale[2] = -MESH_SCALE
        mesh_pos, _ = p.multiplyTransforms([0, 0, 0], FUSION_ORN,
                                           -origin * MESH_SCALE, [0, 0, 0, 1])
        scales.append(scale)
        offsets.append(list(np.array(mesh_pos) - frame_pos_world))
    n = len(files)
    collision = p.createCollisionShapeArray(
        shapeTypes=[p.GEOM_MESH]*n,
        fileNames=files,
        meshScales=scales,
        collisionFramePositions=offsets,
        collisionFrameOrientations=[FUSION_ORN]*n,
    )
    visual = p.createVisualShapeArray(
        shapeTypes=[p.GEOM_MESH]*n,
        fileNames=files,
        meshScales=scales,
        visualFramePositions=offsets,
        visualFrameOrientations=[FUSION_ORN]*n,
        rgbaColors=[color_of(part) for part in parts],
    )
    return collision, visual


def build_robot():
    names = [link[0] for link in LINKS]
    pivots = {name: fusion_to_world(pivot) for name, _, _, pivot, *_ in LINKS}

    base_collision, base_visual = mesh_shapes(BASE_PARTS, np.zeros(3))

    kwargs = {key: [] for key in (
        "linkMasses", "linkCollisionShapeIndices", "linkVisualShapeIndices", "linkPositions",
        "linkOrientations", "linkInertialFramePositions", "linkInertialFrameOrientations",
        "linkParentIndices", "linkJointTypes", "linkJointAxis")}
    for name, parent, joint_type, _, axis, parts, mass in LINKS:
        collision, visual = mesh_shapes(parts, pivots[name])
        parent_pos = pivots[parent] if parent else np.zeros(3)
        kwargs["linkMasses"].append(mass)
        kwargs["linkCollisionShapeIndices"].append(collision)
        kwargs["linkVisualShapeIndices"].append(visual)
        kwargs["linkPositions"].append(list(pivots[name] - parent_pos))  # relative to parent frame
        kwargs["linkOrientations"].append([0, 0, 0, 1])
        kwargs["linkInertialFramePositions"].append([0, 0, 0])
        kwargs["linkInertialFrameOrientations"].append([0, 0, 0, 1])
        kwargs["linkParentIndices"].append(names.index(parent) + 1 if parent else 0)  # 0 = base
        kwargs["linkJointTypes"].append(joint_type)
        kwargs["linkJointAxis"].append(axis)

    robot_id = p.createMultiBody(
        baseMass=0,  # base box stands on the ground
        baseCollisionShapeIndex=base_collision,
        baseVisualShapeIndex=base_visual,
        basePosition=[0, 0, 0],
        **kwargs,
    )
    joints = {name: index for index, name in enumerate(names)}
    for j in joints.values():
        p.resetJointState(robot_id, j, 0)
        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, force=0)
    return robot_id, joints


def command(robot_id, joints, base_yaw, shoulder, elbow, wrist, gripper):
    """Joint angles in rad, gripper in 0 (closed) .. 1 (open)."""
    for name, q in zip(("base_yaw", "shoulder", "elbow", "wrist"), (base_yaw, shoulder, elbow, wrist)):
        q = float(np.clip(q, *JOINT_LIMITS[name]))
        p.setJointMotorControl2(robot_id, joints[name], p.POSITION_CONTROL,
                                targetPosition=q, force=JOINT_FORCE)
    opening = float(np.clip(gripper, 0.0, 1.0)) * JAW_TRAVEL
    for name in ("jaw_left", "jaw_right"):  # both jaws mirror each other
        p.setJointMotorControl2(robot_id, joints[name], p.POSITION_CONTROL,
                                targetPosition=opening, force=JAW_FORCE)


def joint_state(robot_id, joints):
    """Current positions in the same units as the commands."""
    state = {name: p.getJointState(robot_id, joints[name])[0] for name in COMMAND_NAMES[:4]}
    state["gripper"] = p.getJointState(robot_id, joints["jaw_left"])[0] / JAW_TRAVEL
    return state


def open_command_socket(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.setblocking(False)
    print(f"Listening for JSON commands on udp://{host}:{port}")
    return sock


def receive_commands(sock, targets, robot_id, joints):
    """Apply every pending UDP packet to targets; answer each one with the joint state.

    A packet is a JSON object with any subset of COMMAND_NAMES, e.g.
    {"base_yaw": 0.5, "gripper": 1}. {} only asks for the state.
    """
    received = False
    while True:
        try:
            data, sender = sock.recvfrom(4096)
        except (BlockingIOError, ConnectionResetError):
            return received
        try:
            message = json.loads(data)
            updates = {name: float(message[name]) for name in COMMAND_NAMES if name in message}
        except (ValueError, TypeError, AttributeError) as error:
            sock.sendto(json.dumps({"error": str(error)}).encode(), sender)
            continue
        targets.update(updates)
        received = received or bool(updates)
        sock.sendto(json.dumps(joint_state(robot_id, joints)).encode(), sender)


def main():
    parser = argparse.ArgumentParser(description="Compact robot arm in PyBullet.")
    parser.add_argument("--demo", action="store_true", help="scripted motion instead of sliders")
    parser.add_argument("--host", default="127.0.0.1",
                        help="address for UDP commands (0.0.0.0 = accept from the network)")
    parser.add_argument("--port", type=int, default=5005, help="UDP command port, 0 = off")
    args = parser.parse_args()

    # -----------------------------
    # PyBullet setup
    # -----------------------------
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(cameraDistance=0.45, cameraYaw=30, cameraPitch=-25,
                                 cameraTargetPosition=[0.05, 0, 0.08])

    robot_id, joints = build_robot()
    sock = open_command_socket(args.host, args.port) if args.port else None

    sliders = {}
    if not args.demo:
        for name, (low, high) in JOINT_LIMITS.items():
            sliders[name] = p.addUserDebugParameter(name, low, high, 0.0)
        sliders["gripper"] = p.addUserDebugParameter("gripper (0 closed, 1 open)", 0.0, 1.0, 0.0)

    # -----------------------------
    # Control loop
    # -----------------------------
    # Sliders and UDP share the targets: whichever changed last wins.
    targets = dict.fromkeys(COMMAND_NAMES, 0.0)
    last_sliders = {}
    external = False  # True while UDP commands are driving the arm (stops the demo)
    t = 0.0
    dt = 1.0 / 240.0
    while p.isConnected():
        if sock and receive_commands(sock, targets, robot_id, joints):
            external = True
        if args.demo and not external:
            targets.update(zip(COMMAND_NAMES, (
                0.8 * np.sin(0.5 * t),
                0.4 * np.sin(t),
                -0.6 - 0.5 * np.sin(t),
                0.6 * np.sin(1.5 * t),
                0.5 + 0.5 * np.sin(2 * t),
            )))
        if sliders:
            values = {name: p.readUserDebugParameter(sliders[name]) for name in COMMAND_NAMES}
            targets.update({name: v for name, v in values.items() if v != last_sliders.get(name)})
            last_sliders = values
        command(robot_id, joints, *(targets[name] for name in COMMAND_NAMES))

        p.stepSimulation()
        time.sleep(dt)
        t += dt


if __name__ == "__main__":
    main()

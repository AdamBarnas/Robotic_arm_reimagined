import pybullet as p
import pybullet_data
import time
import numpy as np
import os

abs_path = os.getcwd()
print(abs_path)

# -----------------------------
# PyBullet setup
# -----------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")

# -----------------------------
# SCARA parameters
# -----------------------------
base_scale = 0.001  # meshes are in mm
L1 = 0.04   # link 1 length
L2 = 0.03   # link 2 length
link_half_width = 0.002
link_mass = 1.0

# -----------------------------
# Create 2-DOF SCARA using createMultiBody
# -----------------------------
base_mesh_pos = [0.160, -0.183, 0]                                  # metres, after scaling
base_mesh_orn = p.getQuaternionFromEuler([np.pi/2, 0, 0])  # roll, pitch, yaw in radians

# B1.obj is B1.stl recentred by this offset (mesh units); B2.stl is still in the
# Fusion assembly frame, so apply the same shift to keep it aligned under B1.
b2_offset = [0, 0, 0]
base_mesh_pos_2, _ = p.multiplyTransforms(
    base_mesh_pos, base_mesh_orn,
    [c * base_scale for c in b2_offset], [0, 0, 0, 1],
)

# The base takes a single shape index, so both meshes go into one compound shape
base_files = [abs_path+"/stl/B1.stl", abs_path+"/stl/B2.stl"]
base_positions = [base_mesh_pos, base_mesh_pos_2]
base_orientations = [base_mesh_orn, base_mesh_orn]

base_collision_shape = p.createCollisionShapeArray(
    shapeTypes=[p.GEOM_MESH]*2,
    fileNames=base_files,
    meshScales=[[base_scale]*3]*2,
    collisionFramePositions=base_positions,
    collisionFrameOrientations=base_orientations,
)
base_visual_shape = p.createVisualShapeArray(
    shapeTypes=[p.GEOM_MESH]*2,
    fileNames=base_files,
    meshScales=[[base_scale]*3]*2,
    visualFramePositions=base_positions,
    visualFrameOrientations=base_orientations,
    rgbaColors=[[0.5, 0.5, 0.5, 1], [0.4, 0.4, 0.4, 1]],
)

collision_shape_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[L1/2, link_half_width, link_half_width], collisionFramePosition=[L1/2, 0, 0])
visual_shape_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[L1/2, link_half_width, link_half_width], visualFramePosition=[L1/2, 0, 0], rgbaColor=[0.2, 0.6, 0.8, 1])
collision_shape_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[L2/2, link_half_width, link_half_width], collisionFramePosition=[L2/2, 0, 0])
visual_shape_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[L2/2, link_half_width, link_half_width], visualFramePosition=[L2/2, 0, 0], rgbaColor=[0.8, 0.6, 0.2, 1])

robot_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=base_collision_shape,
    baseVisualShapeIndex=base_visual_shape,
    basePosition=[0, 0, 0.05],

    linkMasses=[link_mass, link_mass],
    linkCollisionShapeIndices=[collision_shape_1, collision_shape_2],
    linkVisualShapeIndices=[visual_shape_1, visual_shape_2],
    linkPositions=[
        [0, 0, 0],  # joint 1 → joint 2 offset
        [L1, 0, 0]   # joint 2 → end effector
    ],
    linkOrientations=[
        [0, 0, 0, 1],
        [0, 0, 0, 1]
    ],
    linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],
    linkInertialFrameOrientations=[
        [0, 0, 0, 1], 
        [0, 0, 0, 1]
    ],
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
    linkJointAxis=[
        [0, 0, 1],  # SCARA rotates around Z
        [0, 0, 1]
    ]
)

# -----------------------------
# Joint setup
# -----------------------------
for j in range(2):
    p.resetJointState(robot_id, j, 0)
    p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, force=0)

# -----------------------------
# Simple trajectory
# -----------------------------
t = 0.0
dt = 1.0 / 240.0

while True:
    # Joint angles (simple sine motion)
    q1 = 0.8 * np.sin(t)
    q2 = -0.8 * np.sin(t)

    p.setJointMotorControl2(
        robot_id, 0,
        p.POSITION_CONTROL,
        targetPosition=q1,
        force=5
    )

    p.setJointMotorControl2(
        robot_id, 1,
        p.POSITION_CONTROL,
        targetPosition=q2,
        force=5
    )

    p.stepSimulation()
    time.sleep(dt)
    t += dt

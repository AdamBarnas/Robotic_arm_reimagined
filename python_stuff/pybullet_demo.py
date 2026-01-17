import pybullet as p
import pybullet_data
import time
import numpy as np

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
L1 = 0.4   # link 1 length
L2 = 0.3   # link 2 length
link_mass = 1.0

# -----------------------------
# Create 2-DOF SCARA using createMultiBody
# -----------------------------
base_collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=0.2)
base_visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.2, rgbaColor=[0.5, 0.5, 0.5, 1])
collision_shape_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[L1/2, 0.02, 0.02])
visual_shape_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[L1/2, 0.02, 0.02], visualFramePosition=[L1/2, 0, 0], rgbaColor=[0.2, 0.6, 0.8, 1])
collision_shape_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[L2/2, 0.02, 0.02])
visual_shape_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[L2/2, 0.02, 0.02], visualFramePosition=[L2/2, 0, 0], rgbaColor=[0.8, 0.6, 0.2, 1])

robot_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=base_collision_shape,
    baseVisualShapeIndex=base_visual_shape,
    basePosition=[0, 0, 0],

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
    q2 = 0.8 * np.cos(t)

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

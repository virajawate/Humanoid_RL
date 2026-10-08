#!/usr/bin/env python3

import os
import time
import pybullet as pb
import pybullet_data
from utils import prepare_urdf

G1_DESCRIPTION = os.path.join(os.getcwd(), 'urdf', 'g1_description', 'urdf')
G1_URDF = os.path.join(G1_DESCRIPTION, 'g1_29dof.urdf')
URDF = prepare_urdf(G1_URDF, G1_DESCRIPTION)

init_pose = [0.0, 0.0, 0.750]
init_orien = pb.getQuaternionFromEuler([0.0, 0.0, 0.0])

if not os.path.isdir(G1_DESCRIPTION):
    raise FileNotFoundError(f"g1_description dir not found :\n {G1_DESCRIPTION} \n")
else:
    print("Got Dir")
if not os.path.isfile(G1_URDF):
    raise FileNotFoundError(f"g1_description dir not found :\n {G1_URDF} \n")
else:
    print("Got description file.")

phy_client = pb.connect(pb.GUI)
if phy_client < 0:
    raise RuntimeError("Failed to connect Pybullet.")

pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0.0, 0.0, -9.81)

plane_id = pb.loadURDF("plane.urdf", [0,0,0])

pb.setAdditionalSearchPath(G1_DESCRIPTION)
robot_id = pb.loadURDF(G1_URDF, init_pose, init_orien, useFixedBase=False, flags=(pb.URDF_USE_INERTIA_FROM_FILE | pb.URDF_USE_SELF_COLLISION))
print(f"Got Robot :: {robot_id}")
num_joints = pb.getNumJoints(robot_id)

for i in range(num_joints):
    joint_infos = pb.getJointInfo(robot_id, i)
    joint_name = joint_infos[1].decode("utf-8")
    joint_type = joint_infos[2]

try:
    while True:
        pb.stepSimulation()
        time.sleep(1.0/240.0)
except KeyboardInterrupt:
    print("-------\nStopping Pybullet\n-------\n")
finally:
    pb.disconnect()
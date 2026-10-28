#!/usr/bin/env python3

import os
import time
import pybullet as pb
import pybullet_data
from utils import prepare_urdf

G1_DESCRIPTION = os.path.join(os.getcwd(), 'g1_description', 'urdf')
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


def get_robot_center_of_mass():
    total_mass = 0.0
    weighted_position = [0.0, 0.0, 0.0]

    for link_index in range(-1, num_joints):
        dynamics_info = pb.getDynamicsInfo(
            robot_id, link_index, physicsClientId=phy_client
        )
        mass = dynamics_info[0]
        if mass == 0.0:
            continue

        if link_index == -1:
            base_position, base_orientation = pb.getBasePositionAndOrientation(
                robot_id, physicsClientId=phy_client
            )
            center_of_mass, _ = pb.multiplyTransforms(
                base_position,
                base_orientation,
                dynamics_info[3],
                dynamics_info[4],
            )
        else:
            link_state = pb.getLinkState(
                robot_id,
                link_index,
                computeForwardKinematics=True,
                physicsClientId=phy_client,
            )
            center_of_mass = link_state[0]

        total_mass += mass
        for axis in range(3):
            weighted_position[axis] += mass * center_of_mass[axis]

    if total_mass == 0.0:
        raise RuntimeError(
            "Cannot calculate the robot center of mass: total mass is zero."
        )

    return [coordinate / total_mass for coordinate in weighted_position]


cog_arrow_ids = [-1, -1, -1]
cog_label_id = -1

try:
    while True:
        pb.stepSimulation()
        center_of_mass = get_robot_center_of_mass()
        arrow_start = [center_of_mass[0], center_of_mass[1], center_of_mass[2] + 0.7]
        arrow_head_back = [center_of_mass[0], center_of_mass[1], center_of_mass[2] + 0.12]
        arrow_lines = [
            (arrow_start, center_of_mass),
            (
                center_of_mass,
                [
                    arrow_head_back[0] - 0.08,
                    arrow_head_back[1],
                    arrow_head_back[2],
                ],
            ),
            (
                center_of_mass,
                [
                    arrow_head_back[0] + 0.08,
                    arrow_head_back[1],
                    arrow_head_back[2],
                ],
            ),
        ]
        for index, (line_start, line_end) in enumerate(arrow_lines):
            cog_arrow_ids[index] = pb.addUserDebugLine(
                line_start,
                line_end,
                lineColorRGB=[1.0, 0.0, 0.0],
                lineWidth=3,
                lifeTime=0,
                replaceItemUniqueId=cog_arrow_ids[index],
                physicsClientId=phy_client,
            )
        cog_label_id = pb.addUserDebugText(
            "CoG",
            [arrow_start[0], arrow_start[1], arrow_start[2] + 0.05],
            textColorRGB=[1.0, 0.0, 0.0],
            textSize=1.2,
            lifeTime=0,
            replaceItemUniqueId=cog_label_id,
            physicsClientId=phy_client,
        )
        time.sleep(1.0/960.0)
except KeyboardInterrupt:
    print("-------\nStopping Pybullet\n-------\n")
finally:
    pb.disconnect()
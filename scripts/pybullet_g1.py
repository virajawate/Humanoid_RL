#!/usr/bin/env python3

import os
import time

import pybullet as pb  # pyright: ignore[reportMissingImports]
import pybullet_data  # pyright: ignore[reportMissingImports]
from robot_utils import (
    get_robot_center_of_mass,
    update_center_of_mass_indicator,
)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
G1_PACKAGE = os.path.join(PROJECT_ROOT, "g1_description")
G1_DESCRIPTION = os.path.join(G1_PACKAGE, "urdf")
G1_URDF = os.path.join(G1_DESCRIPTION, "g1_29dof.urdf")
INIT_POSE = [0.0, 0.0, 0.750]
INIT_ORIENTATION = [0.0, 0.0, 0.0]
SIMULATION_RATE = 960.0


def main() -> None:
    if not os.path.isdir(G1_DESCRIPTION):
        raise FileNotFoundError(f"g1_description dir not found:\n{G1_DESCRIPTION}")
    if not os.path.isfile(G1_URDF):
        raise FileNotFoundError(f"g1_description URDF not found:\n{G1_URDF}")

    physics_client = pb.connect(pb.GUI)
    if physics_client < 0:
        raise RuntimeError("Failed to connect to PyBullet.")

    try:
        pb.setAdditionalSearchPath(
            pybullet_data.getDataPath(), physicsClientId=physics_client
        )
        pb.setGravity(0.0, 0.0, -9.81, physicsClientId=physics_client)
        pb.loadURDF("plane.urdf", [0.0, 0.0, 0.0], physicsClientId=physics_client)

        pb.setAdditionalSearchPath(G1_PACKAGE, physicsClientId=physics_client)
        robot_id = pb.loadURDF(
            G1_URDF,
            INIT_POSE,
            pb.getQuaternionFromEuler(INIT_ORIENTATION),
            useFixedBase=False,
            flags=pb.URDF_USE_INERTIA_FROM_FILE | pb.URDF_USE_SELF_COLLISION,
            physicsClientId=physics_client,
        )
        print(f"Loaded robot: {robot_id}")
        num_joints = pb.getNumJoints(robot_id, physicsClientId=physics_client)
        cog_arrow_ids = [-1, -1, -1]
        cog_label_id = -1

        while True:
            pb.stepSimulation(physicsClientId=physics_client)
            center_of_mass = get_robot_center_of_mass(
                pb, robot_id, num_joints, physics_client
            )
            cog_arrow_ids, cog_label_id = update_center_of_mass_indicator(
                pb,
                center_of_mass,
                cog_arrow_ids,
                cog_label_id,
                physics_client,
            )
            time.sleep(1.0 / SIMULATION_RATE)
    except KeyboardInterrupt:
        print("-------\nStopping PyBullet\n-------")
    finally:
        pb.disconnect(physicsClientId=physics_client)


if __name__ == "__main__":
    main()

from typing import Sequence


def get_robot_center_of_mass(
    pybullet,
    robot_id: int,
    num_joints: int,
    physics_client_id: int,
) -> tuple[float, float, float]:
    total_mass = 0.0
    weighted_position = [0.0, 0.0, 0.0]

    for link_index in range(-1, num_joints):
        dynamics_info = pybullet.getDynamicsInfo(
            robot_id, link_index, physicsClientId=physics_client_id
        )
        mass = dynamics_info[0]
        if mass == 0.0:
            continue

        if link_index == -1:
            base_position, base_orientation = pybullet.getBasePositionAndOrientation(
                robot_id, physicsClientId=physics_client_id
            )
            center_of_mass, _ = pybullet.multiplyTransforms(
                base_position,
                base_orientation,
                dynamics_info[3],
                dynamics_info[4],
            )
        else:
            link_state = pybullet.getLinkState(
                robot_id,
                link_index,
                computeForwardKinematics=True,
                physicsClientId=physics_client_id,
            )
            center_of_mass = link_state[0]

        total_mass += mass
        for axis in range(3):
            weighted_position[axis] += mass * center_of_mass[axis]

    if total_mass == 0.0:
        raise RuntimeError(
            "Cannot calculate the robot center of mass: total mass is zero."
        )

    return tuple(coordinate / total_mass for coordinate in weighted_position)


def update_center_of_mass_indicator(
    pybullet,
    center_of_mass: Sequence[float],
    arrow_ids: Sequence[int],
    label_id: int,
    physics_client_id: int,
) -> tuple[list[int], int]:
    if len(arrow_ids) != 3:
        raise ValueError("The center-of-mass arrow requires exactly three line IDs.")

    arrow_start = [
        center_of_mass[0],
        center_of_mass[1],
        center_of_mass[2] + 0.7,
    ]
    arrow_head_back = [
        center_of_mass[0],
        center_of_mass[1],
        center_of_mass[2] + 0.12,
    ]
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

    updated_arrow_ids = [
        pybullet.addUserDebugLine(
            line_start,
            line_end,
            lineColorRGB=[1.0, 0.0, 0.0],
            lineWidth=3,
            lifeTime=0,
            replaceItemUniqueId=arrow_ids[index],
            physicsClientId=physics_client_id,
        )
        for index, (line_start, line_end) in enumerate(arrow_lines)
    ]
    updated_label_id = pybullet.addUserDebugText(
        "CoG",
        [arrow_start[0], arrow_start[1], arrow_start[2] + 0.05],
        textColorRGB=[1.0, 0.0, 0.0],
        textSize=1.2,
        lifeTime=0,
        replaceItemUniqueId=label_id,
        physicsClientId=physics_client_id,
    )

    return updated_arrow_ids, updated_label_id

from typing import Sequence

COG_ARROW_LENGTH = 0.4
MOTION_ARROW_LENGTH = 0.45
MOTION_MIN_SPEED = 0.02


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
        center_of_mass[2] + COG_ARROW_LENGTH,
    ]
    arrow_head_back = [
        center_of_mass[0],
        center_of_mass[1],
        center_of_mass[2] + 0.08,
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


def update_motion_indicator(
    pybullet,
    position: Sequence[float],
    velocity: Sequence[float],
    arrow_ids: Sequence[int],
    label_id: int,
    physics_client_id: int,
) -> tuple[list[int], int]:
    if len(arrow_ids) != 3:
        raise ValueError("The motion arrow requires exactly three line IDs.")

    speed = sum(component**2 for component in velocity) ** 0.5
    if speed < MOTION_MIN_SPEED:
        for item_id in (*arrow_ids, label_id):
            if item_id >= 0:
                pybullet.removeUserDebugItem(
                    item_id, physicsClientId=physics_client_id
                )
        return [-1, -1, -1], -1

    direction = [component / speed for component in velocity]
    start = [position[0], position[1], position[2] + 0.25]
    tip = [
        start[axis] + direction[axis] * MOTION_ARROW_LENGTH
        for axis in range(3)
    ]
    lateral = [-direction[1], direction[0], 0.0]
    lateral_length = sum(component**2 for component in lateral) ** 0.5
    if lateral_length < 1e-6:
        lateral = [1.0, 0.0, 0.0]
    else:
        lateral = [component / lateral_length for component in lateral]

    head_back = [
        tip[axis] - direction[axis] * 0.1
        for axis in range(3)
    ]
    arrow_lines = [
        (start, tip),
        (
            tip,
            [head_back[axis] + lateral[axis] * 0.05 for axis in range(3)],
        ),
        (
            tip,
            [head_back[axis] - lateral[axis] * 0.05 for axis in range(3)],
        ),
    ]
    updated_arrow_ids = [
        pybullet.addUserDebugLine(
            line_start,
            line_end,
            lineColorRGB=[0.0, 1.0, 0.0],
            lineWidth=3,
            lifeTime=0,
            replaceItemUniqueId=arrow_ids[index],
            physicsClientId=physics_client_id,
        )
        for index, (line_start, line_end) in enumerate(arrow_lines)
    ]
    updated_label_id = pybullet.addUserDebugText(
        "Motion",
        [tip[0], tip[1], tip[2] + 0.05],
        textColorRGB=[0.0, 1.0, 0.0],
        textSize=1.0,
        lifeTime=0,
        replaceItemUniqueId=label_id,
        physicsClientId=physics_client_id,
    )
    return updated_arrow_ids, updated_label_id

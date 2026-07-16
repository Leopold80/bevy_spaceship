"""Python 完整例程：在调用方手写姿态/位置闭环并记录可回放轨迹。

建议先阅读 main() 中标出的 1～4 步。plant 只接收 BodyWrench；要接入自己的
控制律、制导律或策略，替换 controller() 即可，不需要修改 apollo_sim。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from apollo_sim import (
    ApolloModelSpec,
    ApolloPlantFactory,
    ApolloState,
    BodyWrench,
    JsonlTrajectoryWriter,
)


OUTPUT_PATH = Path("runs/python_closed_loop_attitude.jsonl")
DESIRED_ATTITUDE_WXYZ = np.array([1.0, 0.0, 0.0, 0.0])
POSITION_NATURAL_FREQUENCY_RADPS = 0.8
POSITION_DAMPING_RATIO = 1.0
MAXIMUM_POSITION_ACCELERATION_MPS2 = 1.0


def quaternion_conjugate(q_wxyz: np.ndarray) -> np.ndarray:
    return np.array([q_wxyz[0], -q_wxyz[1], -q_wxyz[2], -q_wxyz[3]])


def quaternion_multiply(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )


def rotate_vector(q_wxyz: np.ndarray, vector: np.ndarray) -> np.ndarray:
    q_vector = q_wxyz[1:]
    return vector + 2.0 * np.cross(
        q_vector, np.cross(q_vector, vector) + q_wxyz[0] * vector
    )


def center_of_mass_state(
    state: ApolloState, model_spec: ApolloModelSpec
) -> tuple[np.ndarray, np.ndarray]:
    offset_world = rotate_vector(
        state.quaternion_body_to_world_wxyz,
        model_spec.center_of_mass_body_m,
    )
    angular_velocity_world = rotate_vector(
        state.quaternion_body_to_world_wxyz,
        state.angular_velocity_body_radps,
    )
    position_world = state.position_body_origin_world_m + offset_world
    velocity_world = state.linear_velocity_body_origin_world_mps + np.cross(
        angular_velocity_world, offset_world
    )
    return position_world, velocity_world


def attitude_pd_torque(state: ApolloState) -> np.ndarray:
    error = quaternion_multiply(
        DESIRED_ATTITUDE_WXYZ,
        quaternion_conjugate(state.quaternion_body_to_world_wxyz),
    )
    # q 与 -q 表示同一姿态；固定到最短旋转支路。
    if error[0] < 0.0:
        error = -error

    return (
        25_000.0 * (2.0 * error[1:])
        - 18_000.0 * state.angular_velocity_body_radps
    )


def position_hold_force_body(
    state: ApolloState,
    model_spec: ApolloModelSpec,
    target_com_position_world_m: np.ndarray,
) -> np.ndarray:
    position_world, velocity_world = center_of_mass_state(state, model_spec)
    omega = POSITION_NATURAL_FREQUENCY_RADPS
    acceleration_world = (
        omega**2 * (target_com_position_world_m - position_world)
        - 2.0 * POSITION_DAMPING_RATIO * omega * velocity_world
    )
    acceleration_norm = float(np.linalg.norm(acceleration_world))
    if acceleration_norm > MAXIMUM_POSITION_ACCELERATION_MPS2:
        acceleration_world *= MAXIMUM_POSITION_ACCELERATION_MPS2 / acceleration_norm

    force_world = model_spec.mass_kg * acceleration_world
    return rotate_vector(
        quaternion_conjugate(state.quaternion_body_to_world_wxyz), force_world
    )


def controller(
    state: ApolloState,
    model_spec: ApolloModelSpec,
    target_com_position_world_m: np.ndarray,
) -> BodyWrench:
    return BodyWrench(
        force_body_n=position_hold_force_body(
            state, model_spec, target_com_position_world_m
        ),
        torque_about_com_body_nm=attitude_pd_torque(state),
    )


def main() -> None:
    # 1. 工厂共享只读模型；spawn 接收显式初态并创建独立 plant。
    factory = ApolloPlantFactory()
    model_spec = factory.model_spec
    half_angle = np.deg2rad(25.0) / 2.0
    initial_quaternion = np.array(
        [np.cos(half_angle), 0.0, np.sin(half_angle), 0.0]
    )
    initial_angular_velocity_body = np.array([0.15, -0.10, 0.05])
    com_offset_world = rotate_vector(
        initial_quaternion, model_spec.center_of_mass_body_m
    )
    initial_angular_velocity_world = rotate_vector(
        initial_quaternion, initial_angular_velocity_body
    )
    initial_state = ApolloState(
        position_body_origin_world_m=np.zeros(3),
        quaternion_body_to_world_wxyz=initial_quaternion,
        # 使初始质心速度为零；原点速度为零会在非零角速度下产生平动漂移。
        linear_velocity_body_origin_world_mps=-np.cross(
            initial_angular_velocity_world, com_offset_world
        ),
        angular_velocity_body_radps=initial_angular_velocity_body,
    )

    target_com_position_world_m, _ = center_of_mass_state(initial_state, model_spec)
    plant = factory.spawn(initial_state)
    snapshot = plant.snapshot()

    # 2. 记录器由调用方创建，不是 plant.step() 的隐藏副作用。
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT_PATH.open("w", encoding="utf-8") as stream:
        writer = JsonlTrajectoryWriter(
            stream,
            snapshot,
            plant.timing,
            initial_desired_attitude_wxyz=DESIRED_ATTITUDE_WXYZ,
        )
        for _ in range(300):
            # 3. 完整闭环：读状态 -> 算动作 -> step 一次 -> 显式记录。
            action = controller(
                snapshot.state, model_spec, target_com_position_world_m
            )
            result = plant.step(action)
            writer.write_step(
                result, desired_attitude_wxyz=DESIRED_ATTITUDE_WXYZ
            )
            snapshot = result.snapshot

    # 4. 例程自带轻量验收，方便命令行运行和自动检查。
    q = snapshot.state.quaternion_body_to_world_wxyz
    angle_error_deg = np.rad2deg(2.0 * np.arccos(np.clip(abs(q[0]), 0.0, 1.0)))
    final_com_position, final_com_velocity = center_of_mass_state(
        snapshot.state, model_spec
    )
    position_error_m = float(
        np.linalg.norm(final_com_position - target_com_position_world_m)
    )
    com_speed_mps = float(np.linalg.norm(final_com_velocity))
    print(f"control_tick={snapshot.control_tick}")
    print(f"attitude_error_deg={angle_error_deg:.4f}")
    print(f"com_position_error_m={position_error_m:.9f}")
    print(f"com_speed_mps={com_speed_mps:.9f}")
    print(f"trajectory={OUTPUT_PATH}")

    if position_error_m >= 0.05 or com_speed_mps >= 0.02:
        raise RuntimeError(
            "position hold acceptance failed: "
            f"position_error={position_error_m}, com_speed={com_speed_mps}"
        )


if __name__ == "__main__":
    main()

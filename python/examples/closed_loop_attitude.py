"""在调用方手写闭环，演示 plant 不需要知道控制器。"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from apollo_sim import (
    ApolloPlantFactory,
    ApolloState,
    BodyWrench,
    JsonlTrajectoryWriter,
)


OUTPUT_PATH = Path("runs/python_closed_loop_attitude.jsonl")


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


def attitude_pd(state: ApolloState) -> BodyWrench:
    desired_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
    error = quaternion_multiply(
        desired_wxyz,
        quaternion_conjugate(state.quaternion_body_to_world_wxyz),
    )
    # q 与 -q 表示同一姿态；固定到最短旋转支路。
    if error[0] < 0.0:
        error = -error

    torque = 25_000.0 * (2.0 * error[1:]) - 18_000.0 * state.angular_velocity_body_radps
    return BodyWrench(
        force_body_n=np.zeros(3), torque_about_com_body_nm=torque
    )


def main() -> None:
    half_angle = np.deg2rad(25.0) / 2.0
    initial_state = ApolloState(
        position_body_origin_world_m=np.zeros(3),
        quaternion_body_to_world_wxyz=np.array(
            [np.cos(half_angle), 0.0, np.sin(half_angle), 0.0]
        ),
        linear_velocity_body_origin_world_mps=np.zeros(3),
        angular_velocity_body_radps=np.array([0.15, -0.10, 0.05]),
    )

    plant = ApolloPlantFactory().spawn(initial_state)
    snapshot = plant.snapshot()

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT_PATH.open("w", encoding="utf-8") as stream:
        writer = JsonlTrajectoryWriter(stream, snapshot, plant.timing)
        for _ in range(300):
            # 闭环就在普通 Python 循环里；可以直接替换为任意控制律或策略。
            action = attitude_pd(snapshot.state)
            result = plant.step(action)
            writer.write_step(result)
            snapshot = result.snapshot

    q = snapshot.state.quaternion_body_to_world_wxyz
    angle_error_deg = np.rad2deg(2.0 * np.arccos(np.clip(abs(q[0]), 0.0, 1.0)))
    print(f"control_tick={snapshot.control_tick}")
    print(f"attitude_error_deg={angle_error_deg:.4f}")
    print(f"trajectory={OUTPUT_PATH}")


if __name__ == "__main__":
    main()

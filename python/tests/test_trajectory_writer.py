import io
import json

import numpy as np
import pytest

from apollo_sim import (
    ApolloState,
    BodyWrench,
    JsonlTrajectoryWriter,
    PlantSnapshot,
    PlantStep,
    SimulationTiming,
)


def sample_step(control_tick: int = 3) -> PlantStep:
    state = ApolloState(
        position_body_origin_world_m=[1.0, 2.0, 3.0],
        quaternion_body_to_world_wxyz=[0.5, 0.5, 0.5, 0.5],
        linear_velocity_body_origin_world_mps=[4.0, 5.0, 6.0],
        angular_velocity_body_radps=[7.0, 8.0, 9.0],
    )
    requested = BodyWrench(
        force_body_n=[10.0, 11.0, 12.0],
        torque_about_com_body_nm=[13.0, 14.0, 15.0],
    )
    applied = BodyWrench(
        force_body_n=[16.0, 17.0, 18.0],
        torque_about_com_body_nm=[19.0, 20.0, 21.0],
    )
    return PlantStep(
        snapshot=PlantSnapshot(
            state=state,
            control_tick=control_tick,
            physics_tick=control_tick * 10,
        ),
        requested_action=requested,
        applied_action=applied,
    )


def initial_snapshot() -> PlantSnapshot:
    return PlantSnapshot(ApolloState.identity(), control_tick=0, physics_tick=0)


def test_writer_emits_rust_v1_header_and_explicit_wxyz_frame() -> None:
    stream = io.StringIO()
    writer = JsonlTrajectoryWriter(stream, initial_snapshot(), SimulationTiming())
    writer.write_step(sample_step())

    header_line, frame_line = stream.getvalue().splitlines()
    header = json.loads(header_line)
    assert header == {
        "format": "apollo-telemetry-jsonl",
        "version": 1,
        "model": "apollo_lander",
        "timing": {"physics_step_ns": 2_000_000, "substeps_per_control": 10},
        "initial_snapshot": {
            "state": {
                "position_body_origin_world_m": [0.0, 0.0, 0.0],
                "quaternion_body_to_world_wxyz": [1.0, 0.0, 0.0, 0.0],
                "linear_velocity_body_origin_world_mps": [0.0, 0.0, 0.0],
                "angular_velocity_body_radps": [0.0, 0.0, 0.0],
            },
            "control_tick": 0,
            "physics_tick": 0,
        },
    }

    frame = json.loads(frame_line)
    state = frame["snapshot"]["state"]
    assert state == {
        "position_body_origin_world_m": [1.0, 2.0, 3.0],
        "quaternion_body_to_world_wxyz": [0.5, 0.5, 0.5, 0.5],
        "linear_velocity_body_origin_world_mps": [4.0, 5.0, 6.0],
        "angular_velocity_body_radps": [7.0, 8.0, 9.0],
    }
    assert frame["requested_action"]["torque_about_com_body_nm"] == [
        13.0,
        14.0,
        15.0,
    ]


def test_writer_rejects_non_aligned_or_non_monotonic_ticks() -> None:
    stream = io.StringIO()
    writer = JsonlTrajectoryWriter(stream, initial_snapshot(), SimulationTiming())
    writer.write_step(sample_step(control_tick=1))

    with pytest.raises(ValueError, match="strictly increasing"):
        writer.write_step(sample_step(control_tick=1))

    invalid = sample_step(control_tick=2)
    invalid = PlantStep(
        snapshot=PlantSnapshot(invalid.snapshot.state, 2, 21),
        requested_action=invalid.requested_action,
        applied_action=invalid.applied_action,
    )
    with pytest.raises(ValueError, match="does not match"):
        writer.write_step(invalid)

    overflowing = PlantStep(
        snapshot=PlantSnapshot(
            invalid.snapshot.state,
            np.iinfo(np.uint64).max,
            np.iinfo(np.uint64).max,
        ),
        requested_action=invalid.requested_action,
        applied_action=invalid.applied_action,
    )
    with pytest.raises(ValueError, match="overflows"):
        writer.write_step(overflowing)


def test_writer_uses_strict_json_allow_nan_false() -> None:
    stream = io.StringIO()
    writer = JsonlTrajectoryWriter(stream, initial_snapshot(), SimulationTiming())
    step = sample_step(control_tick=1)

    # 数据对象正常情况下不可变且会拒绝 NaN；这里刻意破坏数组以验证写边界。
    position = step.snapshot.state.position_body_origin_world_m
    position.setflags(write=True)
    position[0] = np.nan

    with pytest.raises(ValueError, match="finite"):
        writer.write_step(step)


def test_writer_revalidates_mutated_quaternion_and_wrench() -> None:
    stream = io.StringIO()
    writer = JsonlTrajectoryWriter(stream, initial_snapshot(), SimulationTiming())

    quaternion_step = sample_step(control_tick=1)
    quaternion = quaternion_step.snapshot.state.quaternion_body_to_world_wxyz
    quaternion.setflags(write=True)
    quaternion[:] = [2.0, 0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="unit quaternion"):
        writer.write_step(quaternion_step)

    wrench_step = sample_step(control_tick=1)
    force = wrench_step.requested_action.force_body_n
    force.setflags(write=True)
    force[0] = np.inf
    with pytest.raises(ValueError, match="finite"):
        writer.write_step(wrench_step)


def test_writer_requires_a_valid_zero_tick_initial_snapshot() -> None:
    nonzero = PlantSnapshot(ApolloState.identity(), control_tick=1, physics_tick=10)
    with pytest.raises(ValueError, match="control tick 0"):
        JsonlTrajectoryWriter(io.StringIO(), nonzero, SimulationTiming())

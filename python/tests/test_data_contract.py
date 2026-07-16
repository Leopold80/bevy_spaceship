from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from apollo_sim import ApolloState, BodyWrench, PlantSnapshot, SimulationTiming


def test_state_vector_order_and_dtype_are_explicit() -> None:
    state = ApolloState(
        position_body_origin_world_m=[1, 2, 3],
        quaternion_body_to_world_wxyz=[1, 0, 0, 0],
        linear_velocity_body_origin_world_mps=[4, 5, 6],
        angular_velocity_body_radps=[7, 8, 9],
    )

    np.testing.assert_array_equal(
        state.as_vector(), np.array([1, 2, 3, 1, 0, 0, 0, 4, 5, 6, 7, 8, 9])
    )
    assert state.as_vector().dtype == np.float64
    assert not state.position_body_origin_world_m.flags.writeable


def test_data_objects_are_frozen_and_own_their_arrays() -> None:
    source = np.array([1.0, 2.0, 3.0])
    state = ApolloState(
        position_body_origin_world_m=source,
        quaternion_body_to_world_wxyz=[1, 0, 0, 0],
        linear_velocity_body_origin_world_mps=[0, 0, 0],
        angular_velocity_body_radps=[0, 0, 0],
    )
    source[0] = 99.0

    assert state.position_body_origin_world_m[0] == 1.0
    with pytest.raises(ValueError):
        state.position_body_origin_world_m[0] = 5.0
    with pytest.raises(FrozenInstanceError):
        state.position_body_origin_world_m = np.zeros(3)  # type: ignore[misc]


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("position_body_origin_world_m", [0, 0], "shape"),
        ("position_body_origin_world_m", [0, np.nan, 0], "finite"),
        ("quaternion_body_to_world_wxyz", [2, 0, 0, 0], "unit quaternion"),
        ("linear_velocity_body_origin_world_mps", [0, np.inf, 0], "finite"),
        ("angular_velocity_body_radps", [0, 0, 0, 0], "shape"),
    ],
)
def test_state_rejects_invalid_vectors(field: str, value: list[float], message: str) -> None:
    values = {
        "position_body_origin_world_m": [0, 0, 0],
        "quaternion_body_to_world_wxyz": [1, 0, 0, 0],
        "linear_velocity_body_origin_world_mps": [0, 0, 0],
        "angular_velocity_body_radps": [0, 0, 0],
    }
    values[field] = value

    with pytest.raises(ValueError, match=message):
        ApolloState(**values)


def test_wrench_vector_order_and_validation() -> None:
    wrench = BodyWrench(
        force_body_n=[1, 2, 3], torque_about_com_body_nm=[4, 5, 6]
    )
    np.testing.assert_array_equal(wrench.as_vector(), [1, 2, 3, 4, 5, 6])

    with pytest.raises(ValueError, match="finite"):
        BodyWrench(
            force_body_n=[0, 0, 0], torque_about_com_body_nm=[0, np.nan, 0]
        )


def test_timing_and_snapshot_validate_the_integer_clock() -> None:
    timing = SimulationTiming(physics_step_seconds=0.002, substeps_per_control=10)
    assert timing.control_step_ns == 20_000_000
    assert timing.control_step_seconds == pytest.approx(0.02)
    snapshot = PlantSnapshot(ApolloState.identity(), control_tick=3, physics_tick=30)
    assert snapshot.sim_time_ns(timing) == 60_000_000
    assert snapshot.sim_time_seconds(timing) == pytest.approx(0.06)

    with pytest.raises(ValueError, match="positive"):
        SimulationTiming(substeps_per_control=0)
    with pytest.raises(ValueError, match="non-negative"):
        PlantSnapshot(ApolloState.identity(), control_tick=-1, physics_tick=0)
    with pytest.raises(ValueError, match="64-bit"):
        PlantSnapshot(ApolloState.identity(), control_tick=2**64, physics_tick=0)


def test_timing_normalizes_to_the_same_integer_nanoseconds_as_rust() -> None:
    timing = SimulationTiming(
        physics_step_seconds=0.0020000000000005,
        substeps_per_control=np.int64(10),
    )

    assert timing.physics_step_seconds == 0.002
    assert timing.control_step_seconds == 0.02

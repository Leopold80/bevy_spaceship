import numpy as np
import pytest

import apollo_sim._api as api
from apollo_sim import (
    ApolloModelSpec,
    ApolloPlantFactory,
    ApolloState,
    BodyWrench,
    SimulationTiming,
)


@pytest.fixture(scope="module", autouse=True)
def require_native_extension() -> None:
    assert api._native is not None, (
        "apollo_sim native extension is required for contract tests; "
        "run `maturin develop` first"
    )


def nontrivial_state() -> ApolloState:
    half_angle = 0.25
    return ApolloState(
        position_body_origin_world_m=[1.0, -2.0, 0.5],
        quaternion_body_to_world_wxyz=[np.cos(half_angle), 0, np.sin(half_angle), 0],
        linear_velocity_body_origin_world_mps=[-0.2, 0.3, 0.4],
        angular_velocity_body_radps=[0.6, -0.35, 0.15],
    )


def test_missing_native_extension_preserves_original_import_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    failure = ImportError("sentinel native load failure")
    monkeypatch.setattr(api, "_native", None)
    monkeypatch.setattr(api, "_native_import_failure", failure)

    with pytest.raises(ImportError, match="sentinel native load failure") as caught:
        api.ApolloPlantFactory()

    assert caught.value.__cause__ is failure


def test_reset_snapshot_and_step_follow_rust_ticks() -> None:
    timing = SimulationTiming(physics_step_seconds=0.002, substeps_per_control=10)
    plant = ApolloPlantFactory(timing).spawn(ApolloState.identity())

    reset = plant.reset(nontrivial_state())
    assert reset.control_tick == 0
    assert reset.physics_tick == 0
    np.testing.assert_allclose(reset.state.as_vector(), nontrivial_state().as_vector())

    result = plant.step(BodyWrench.zero())
    assert result.snapshot.control_tick == 1
    assert result.snapshot.physics_tick == 10
    assert result.requested_action == BodyWrench.zero()
    assert result.applied_action == BodyWrench.zero()
    assert result.snapshot == plant.snapshot()


def test_python_and_native_timing_use_the_same_integer_nanoseconds() -> None:
    timing = SimulationTiming(
        physics_step_seconds=0.0020000000000005,
        substeps_per_control=10,
    )
    plant = ApolloPlantFactory(timing).spawn(ApolloState.identity())

    assert timing.physics_step_ns == 2_000_000
    assert plant._native_plant.timing() == (0.002, 10)
    assert plant.step(BodyWrench.zero()).snapshot.sim_time_seconds(timing) == 0.02


def test_factory_exposes_readonly_touchdown_model_spec() -> None:
    spec = ApolloPlantFactory().model_spec

    assert isinstance(spec, ApolloModelSpec)
    assert spec.name == "apollo_lander"
    assert spec.mass_kg == pytest.approx(4_932.0)
    np.testing.assert_allclose(
        spec.center_of_mass_body_m,
        [0.0, 2.012912912912913, 0.0],
        atol=1.0e-15,
    )
    np.testing.assert_allclose(
        spec.diagonal_inertia_body_kg_m2, [6_332.0, 7_953.0, 5_879.0]
    )
    assert not spec.center_of_mass_body_m.flags.writeable
    assert not spec.diagonal_inertia_body_kg_m2.flags.writeable


def test_reset_replay_is_deterministic() -> None:
    plant = ApolloPlantFactory().spawn(nontrivial_state())
    actions = [
        BodyWrench(
            force_body_n=[1000, -500, 250],
            torque_about_com_body_nm=[100, 200, -300],
        ),
        BodyWrench(
            force_body_n=[-200, 100, 50],
            torque_about_com_body_nm=[-25, 75, 50],
        ),
        BodyWrench.zero(),
    ]

    def rollout() -> np.ndarray:
        plant.reset(nontrivial_state())
        snapshots = [plant.step(action).snapshot.state.as_vector() for action in actions]
        return np.stack(snapshots)

    np.testing.assert_array_equal(rollout(), rollout())


def test_factory_spawns_independent_plants() -> None:
    factory = ApolloPlantFactory()
    left = factory.spawn(ApolloState.identity())
    right = factory.spawn(ApolloState.identity())

    left.step(
        BodyWrench(
            force_body_n=[1000, 0, 0], torque_about_com_body_nm=[0, 0, 0]
        )
    )

    assert left.snapshot().control_tick == 1
    assert right.snapshot().control_tick == 0
    np.testing.assert_array_equal(
        right.snapshot().state.as_vector(), ApolloState.identity().as_vector()
    )


def test_wxyz_rotation_maps_body_force_to_the_expected_world_axis() -> None:
    half_sqrt = np.sqrt(0.5)
    rotated = ApolloState(
        position_body_origin_world_m=[0, 0, 0],
        # +90 degrees around world Z, explicitly in wxyz order.
        quaternion_body_to_world_wxyz=[half_sqrt, 0, 0, half_sqrt],
        linear_velocity_body_origin_world_mps=[0, 0, 0],
        angular_velocity_body_radps=[0, 0, 0],
    )
    plant = ApolloPlantFactory().spawn(rotated)
    result = plant.step(
        BodyWrench(
            force_body_n=[10_000, 0, 0], torque_about_com_body_nm=[0, 0, 0]
        )
    )

    velocity = result.snapshot.state.linear_velocity_body_origin_world_mps
    assert velocity[1] > 0.0
    assert abs(velocity[0]) < velocity[1] * 1.0e-10
    assert abs(velocity[2]) < velocity[1] * 1.0e-10

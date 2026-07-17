import numpy as np
import pytest

import apollo_sim._api as api
from apollo_sim import (
    ApolloPropulsionPlantFactory,
    ApolloState,
    DpsCommand,
    DpsMode,
    PropulsionCommand,
    RCS_THRUSTER_ORDER,
    RcsCommand,
    RcsThrusterId,
    SimulationTiming,
)


@pytest.fixture(scope="module", autouse=True)
def require_native_extension() -> None:
    assert api._native is not None, (
        "apollo_sim native extension is required for propulsion tests; "
        "run `maturin develop` first"
    )
    assert hasattr(api._native, "ApolloPropulsionPlantFactory"), (
        "native extension is stale; run `maturin develop` after Rust changes"
    )


def test_factory_exposes_readonly_apollo11_propulsion_spec() -> None:
    factory = ApolloPropulsionPlantFactory.apollo11_touchdown()
    spec = factory.propulsion_spec

    assert tuple(thruster.id for thruster in spec.rcs_thrusters) == RCS_THRUSTER_ORDER
    assert [thruster.label for thruster in spec.rcs_thrusters] == [
        thruster.name for thruster in RCS_THRUSTER_ORDER
    ]
    np.testing.assert_array_equal(
        spec.rcs_thrusters[RcsThrusterId.A1U].force_direction_body,
        [0.0, -1.0, 0.0],
    )
    assert spec.rcs_thrusters[0].steady_thrust_n == pytest.approx(444.82216152605)
    assert spec.rcs_thrusters[0].minimum_pulse_ns == 14_000_000
    assert np.rad2deg(spec.dps.gimbal_rate_rad_s) == pytest.approx(0.2)
    assert not spec.rcs_thrusters[0].position_body_m.flags.writeable
    assert not spec.dps.gimbal_pivot_body_m.flags.writeable
    assert factory.spawn(ApolloState.identity()).propulsion_spec is spec


def test_short_rcs_request_is_raised_to_minimum_pulse_and_advances_one_tick() -> None:
    plant = ApolloPropulsionPlantFactory.apollo11_touchdown().spawn(
        ApolloState.identity()
    )
    command = PropulsionCommand(
        rcs=RcsCommand.single_pulse(RcsThrusterId.A1F, 1_000_000),
        dps=DpsCommand.off(),
    )

    step = plant.step(command)

    assert step.snapshot.control_tick == 1
    assert step.snapshot.physics_tick == 10
    assert step.requested_command == command
    assert (
        step.applied.rcs.applied_gate_on_time_ns[RcsThrusterId.A1F]
        == 14_000_000
    )
    assert step.applied.rcs.mean_thrust_n[RcsThrusterId.A1F] > 0.0
    assert np.count_nonzero(step.applied.rcs.mean_thrust_n) == 1
    assert step.applied.mean_wrench_body.force_body_n[2] < 0.0


def test_continuous_rcs_gate_across_ticks_does_not_reapply_minimum_pulse() -> None:
    plant = ApolloPropulsionPlantFactory.apollo11_touchdown().spawn(
        ApolloState.identity()
    )
    first = plant.step(
        PropulsionCommand(
            rcs=RcsCommand.single_pulse(RcsThrusterId.A1U, 20_000_000),
            dps=DpsCommand.off(),
        )
    )
    second = plant.step(
        PropulsionCommand(
            rcs=RcsCommand.single_pulse(RcsThrusterId.A1U, 1_000_000),
            dps=DpsCommand.off(),
        )
    )

    assert (
        first.applied.rcs.applied_gate_on_time_ns[RcsThrusterId.A1U]
        == 20_000_000
    )
    assert (
        second.applied.rcs.applied_gate_on_time_ns[RcsThrusterId.A1U]
        == 1_000_000
    )


def test_overlong_rcs_request_is_value_error_without_advancing() -> None:
    plant = ApolloPropulsionPlantFactory.apollo11_touchdown().spawn(
        ApolloState.identity()
    )
    before = plant.snapshot()
    command = PropulsionCommand(
        rcs=RcsCommand.single_pulse(RcsThrusterId.A1U, 20_000_001),
        dps=DpsCommand.off(),
    )

    with pytest.raises(ValueError, match="longer than control step"):
        plant.step(command)

    assert plant.snapshot() == before


def test_dps_modes_apply_the_point_two_degree_per_second_gimbal_rate() -> None:
    factory = ApolloPropulsionPlantFactory.apollo11_touchdown()
    plant = factory.spawn(ApolloState.identity())
    dps = factory.propulsion_spec.dps
    expected_delta = dps.gimbal_rate_rad_s * factory.timing.control_step_seconds
    assert np.rad2deg(dps.gimbal_rate_rad_s) == pytest.approx(0.2)
    assert np.rad2deg(expected_delta) == pytest.approx(0.004)

    variable = plant.step(
        PropulsionCommand(
            rcs=RcsCommand.all_off(),
            dps=DpsCommand.variable(
                1.0,
                gimbal_x_rad=np.deg2rad(10.0),
                gimbal_z_rad=np.deg2rad(10.0),
            ),
        )
    )

    assert variable.requested_command.dps.thrust_n == 1.0
    assert variable.applied.dps.mode is DpsMode.VARIABLE
    assert variable.applied.dps.thrust_n == factory.propulsion_spec.dps.variable_min_thrust_n
    assert np.hypot(
        variable.applied.dps.gimbal_x_rad,
        variable.applied.dps.gimbal_z_rad,
    ) == pytest.approx(expected_delta)
    assert variable.applied.dps.gimbal_x_rad == pytest.approx(
        variable.applied.dps.gimbal_z_rad
    )
    assert variable.applied.dps.force_direction_body[0] > 0.0
    assert variable.applied.dps.force_direction_body[1] > 0.0
    assert variable.applied.dps.force_direction_body[2] > 0.0

    full = plant.step(
        PropulsionCommand(
            rcs=RcsCommand.all_off(),
            dps=DpsCommand.full_thrust(
                gimbal_x_rad=np.deg2rad(10.0),
                gimbal_z_rad=np.deg2rad(10.0),
            ),
        )
    )
    assert full.applied.dps.mode is DpsMode.FULL_THRUST
    assert full.applied.dps.thrust_n == factory.propulsion_spec.dps.full_thrust_n
    assert np.hypot(
        full.applied.dps.gimbal_x_rad, full.applied.dps.gimbal_z_rad
    ) == pytest.approx(2.0 * expected_delta)


def test_dps_off_holds_gimbal_and_reset_recenters_it() -> None:
    factory = ApolloPropulsionPlantFactory.apollo11_touchdown()
    plant = factory.spawn(ApolloState.identity())
    driven = plant.step(
        PropulsionCommand(
            rcs=RcsCommand.all_off(),
            dps=DpsCommand.full_thrust(
                gimbal_x_rad=np.deg2rad(1.0),
                gimbal_z_rad=-np.deg2rad(2.0),
            ),
        )
    )

    held = plant.step(PropulsionCommand.all_off())
    assert held.applied.dps.mode is DpsMode.OFF
    assert held.applied.dps.thrust_n == 0.0
    assert held.applied.dps.gimbal_x_rad == driven.applied.dps.gimbal_x_rad
    assert held.applied.dps.gimbal_z_rad == driven.applied.dps.gimbal_z_rad

    plant.reset(ApolloState.identity())
    centered = plant.step(PropulsionCommand.all_off())
    assert centered.applied.dps.gimbal_x_rad == 0.0
    assert centered.applied.dps.gimbal_z_rad == 0.0
    np.testing.assert_array_equal(
        centered.applied.dps.force_direction_body,
        factory.propulsion_spec.dps.nominal_force_direction_body,
    )


def test_reset_clears_rcs_shutdown_tail() -> None:
    plant = ApolloPropulsionPlantFactory.apollo11_touchdown().spawn(
        ApolloState.identity()
    )
    plant.step(
        PropulsionCommand(
            rcs=RcsCommand.single_pulse(RcsThrusterId.A1U, 20_000_000),
            dps=DpsCommand.off(),
        )
    )
    plant.reset(ApolloState.identity())

    step = plant.step(PropulsionCommand.all_off())
    np.testing.assert_array_equal(step.applied.rcs.mean_thrust_n, np.zeros(16))


def test_factory_rejects_control_period_shorter_than_minimum_rcs_pulse() -> None:
    too_short = SimulationTiming(
        physics_step_seconds=0.001,
        substeps_per_control=10,
    )
    with pytest.raises(ValueError, match="shorter than RCS minimum pulse"):
        ApolloPropulsionPlantFactory(too_short)

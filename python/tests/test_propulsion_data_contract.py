from dataclasses import FrozenInstanceError
from types import SimpleNamespace

import numpy as np
import pytest

import apollo_sim._api as api
from apollo_sim import (
    AppliedDps,
    AppliedPropulsion,
    AppliedRcs,
    ApolloPropulsionSpec,
    ApolloPropulsionPlantFactory,
    ApolloState,
    BodyWrench,
    DpsCommand,
    DpsMode,
    DpsSpec,
    PlantSnapshot,
    PropulsionCommand,
    PropulsionStep,
    RCS_THRUSTER_ORDER,
    RcsCommand,
    RcsFeedSystem,
    RcsQuad,
    RcsThrusterSpec,
    RcsThrusterId,
)


def test_rcs_command_owns_a_readonly_uint64_vector() -> None:
    source = np.arange(16, dtype=np.uint64)
    command = RcsCommand(source)
    source[0] = 99

    assert command.on_time_ns.dtype == np.uint64
    assert command.on_time_ns.shape == (16,)
    assert command.on_time_ns[0] == 0
    assert not command.on_time_ns.flags.writeable
    assert not command.as_vector().flags.writeable

    with pytest.raises(ValueError):
        command.on_time_ns[0] = 1
    with pytest.raises(FrozenInstanceError):
        command.on_time_ns = np.zeros(16, dtype=np.uint64)  # type: ignore[misc]


@pytest.mark.parametrize(
    ("value", "message"),
    [
        ([0] * 15, "shape"),
        ([0.0] * 16, "integers"),
        ([False] * 16, "integers"),
        ([-1] + [0] * 15, "unsigned 64-bit"),
        ([2**64] + [0] * 15, "unsigned 64-bit"),
    ],
)
def test_rcs_command_rejects_ambiguous_or_out_of_range_values(
    value: list[object], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        RcsCommand(value)


def test_rcs_convenience_constructors_use_stable_thruster_indices() -> None:
    off = RcsCommand.all_off()
    assert np.count_nonzero(off.on_time_ns) == 0

    pulse = RcsCommand.single_thruster(7, 14_000_000)
    assert pulse.on_time_ns[7] == 14_000_000
    assert np.count_nonzero(pulse.on_time_ns) == 1

    with pytest.raises(ValueError, match=r"\[0, 15\]"):
        RcsCommand.single_thruster(16, 1)


def test_rcs_labels_define_the_public_vector_order() -> None:
    assert [thruster.name for thruster in RCS_THRUSTER_ORDER] == [
        "A1U",
        "B1D",
        "A1F",
        "B1L",
        "B2U",
        "A2D",
        "A2A",
        "B2L",
        "A3U",
        "B3D",
        "B3A",
        "A3R",
        "B4U",
        "A4D",
        "B4F",
        "A4R",
    ]
    pulse = RcsCommand.single_pulse(RcsThrusterId.B3A, 14_000_000)
    assert pulse.on_time_ns[RcsThrusterId.B3A] == 14_000_000
    assert np.count_nonzero(pulse.on_time_ns) == 1

    with pytest.raises(ValueError, match="RcsThrusterId"):
        RcsCommand.single_pulse(10, 14_000_000)  # type: ignore[arg-type]


def test_dps_modes_preserve_variant_specific_fields_and_units() -> None:
    assert DpsCommand.off() == DpsCommand(mode=DpsMode.OFF)
    assert DpsCommand.variable(
        20_000.0, gimbal_x_rad=0.01, gimbal_z_rad=-0.02
    ) == DpsCommand(
        mode=DpsMode.VARIABLE,
        thrust_n=20_000.0,
        gimbal_x_rad=0.01,
        gimbal_z_rad=-0.02,
    )
    assert DpsCommand.full_thrust(gimbal_x_rad=0.03).mode is DpsMode.FULL_THRUST

    with pytest.raises(ValueError, match="requires thrust_n"):
        DpsCommand(mode=DpsMode.VARIABLE)
    with pytest.raises(ValueError, match="only valid"):
        DpsCommand(mode=DpsMode.FULL_THRUST, thrust_n=10.0)
    with pytest.raises(ValueError, match="zero gimbal"):
        DpsCommand(mode=DpsMode.OFF, gimbal_x_rad=0.01)
    with pytest.raises(ValueError, match="finite"):
        DpsCommand.variable(20_000.0, gimbal_z_rad=np.inf)
    with pytest.raises(ValueError, match="DpsMode"):
        DpsCommand(mode="off")  # type: ignore[arg-type]


def test_propulsion_spec_preserves_stable_order_and_readonly_geometry() -> None:
    directions = {
        # 历史字母描述喷管/羽流方向；公共 spec 返回飞船受力方向，必须取反。
        "U": [0.0, -1.0, 0.0],
        "D": [0.0, 1.0, 0.0],
        "F": [0.0, 0.0, -1.0],
        "A": [0.0, 0.0, 1.0],
        "R": [-1.0, 0.0, 0.0],
        "L": [1.0, 0.0, 0.0],
    }
    thrusters = tuple(
        RcsThrusterSpec(
            id=thruster_id,
            label=thruster_id.name,
            quad=RcsQuad(int(thruster_id.name[1])),
            feed_system=RcsFeedSystem(thruster_id.name[0]),
            position_body_m=[float(thruster_id), 3.0, 0.0],
            force_direction_body=directions[thruster_id.name[-1]],
            steady_thrust_n=444.822,
            minimum_pulse_ns=14_000_000,
        )
        for thruster_id in RCS_THRUSTER_ORDER
    )
    dps = DpsSpec(
        gimbal_pivot_body_m=[0.0, 1.24, 0.0],
        nominal_force_direction_body=[0.0, 1.0, 0.0],
        variable_min_thrust_n=4_670.633,
        variable_max_thrust_n=28_023.796,
        full_thrust_n=43_903.947,
        maximum_gimbal_rad=np.deg2rad(6.0),
        gimbal_rate_rad_s=np.deg2rad(0.2),
    )
    spec = ApolloPropulsionSpec(rcs_thrusters=thrusters, dps=dps)

    assert tuple(item.id for item in spec.rcs_thrusters) == RCS_THRUSTER_ORDER
    assert not spec.rcs_thrusters[0].position_body_m.flags.writeable
    assert not spec.rcs_thrusters[0].force_direction_body.flags.writeable
    assert not spec.dps.gimbal_pivot_body_m.flags.writeable
    assert not spec.dps.nominal_force_direction_body.flags.writeable
    assert spec.dps.gimbal_rate_rad_s == pytest.approx(np.deg2rad(0.2))

    with pytest.raises(ValueError, match="RCS_THRUSTER_ORDER"):
        ApolloPropulsionSpec(rcs_thrusters=thrusters[::-1], dps=dps)


@pytest.mark.parametrize(
    "gimbal_rate_rad_s",
    [0.0, -1.0, np.nan, np.inf],
)
def test_dps_spec_rejects_invalid_gimbal_rate(gimbal_rate_rad_s: float) -> None:
    with pytest.raises(ValueError, match="gimbal_rate_rad_s"):
        DpsSpec(
            gimbal_pivot_body_m=[0.0, 1.24, 0.0],
            nominal_force_direction_body=[0.0, 1.0, 0.0],
            variable_min_thrust_n=4_670.633,
            variable_max_thrust_n=28_023.796,
            full_thrust_n=43_903.947,
            maximum_gimbal_rad=np.deg2rad(6.0),
            gimbal_rate_rad_s=gimbal_rate_rad_s,
        )


def test_propulsion_command_is_typed_and_all_off_is_explicit() -> None:
    command = PropulsionCommand.all_off()
    assert command.rcs == RcsCommand.all_off()
    assert command.dps == DpsCommand.off()

    with pytest.raises(ValueError, match="RcsCommand"):
        PropulsionCommand(rcs=np.zeros(16), dps=DpsCommand.off())  # type: ignore[arg-type]


def test_applied_propulsion_and_step_keep_arrays_readonly() -> None:
    applied = AppliedPropulsion(
        rcs=AppliedRcs(
            applied_gate_on_time_ns=[14_000_000] + [0] * 15,
            mean_thrust_n=[311.25] + [0.0] * 15,
        ),
        dps=AppliedDps(
            mode=DpsMode.OFF,
            thrust_n=0.0,
            gimbal_x_rad=0.0,
            gimbal_z_rad=0.0,
            force_direction_body=[0.0, 1.0, 0.0],
        ),
        mean_wrench_body=BodyWrench(
            force_body_n=[1.0, 2.0, 3.0],
            torque_about_com_body_nm=[4.0, 5.0, 6.0],
        ),
    )
    step = PropulsionStep(
        snapshot=PlantSnapshot(
            state=ApolloState.identity(), control_tick=1, physics_tick=10
        ),
        requested_command=PropulsionCommand.all_off(),
        applied=applied,
    )

    assert step.applied.rcs.applied_gate_on_time_ns.dtype == np.uint64
    assert not step.applied.rcs.applied_gate_on_time_ns.flags.writeable
    assert not step.applied.rcs.mean_thrust_n.flags.writeable
    assert not step.applied.dps.force_direction_body.flags.writeable

    with pytest.raises(ValueError, match="non-negative"):
        AppliedRcs(
            applied_gate_on_time_ns=[0] * 16,
            mean_thrust_n=[-1.0] + [0.0] * 15,
        )


def test_python_native_adapter_rebuilds_typed_propulsion_results(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    directions = {
        "U": [0.0, -1.0, 0.0],
        "D": [0.0, 1.0, 0.0],
        "F": [0.0, 0.0, -1.0],
        "A": [0.0, 0.0, 1.0],
        "R": [-1.0, 0.0, 0.0],
        "L": [1.0, 0.0, 0.0],
    }

    class FakePlant:
        def reset(self, state: list[float]) -> tuple[list[float], int, int]:
            return state, 0, 0

        def snapshot(self) -> tuple[list[float], int, int]:
            return ApolloState.identity().as_vector().tolist(), 0, 0

        def step(
            self,
            on_time_ns: list[int],
            mode: str,
            thrust_n: float | None,
            gimbal_x_rad: float,
            gimbal_z_rad: float,
        ) -> object:
            assert mode == "variable"
            assert thrust_n == 4_670.633
            applied_gate = [14_000_000 if value else 0 for value in on_time_ns]
            mean_thrust = [12.5 if value else 0.0 for value in on_time_ns]
            return (
                (ApolloState.identity().as_vector().tolist(), 1, 10),
                (
                    on_time_ns,
                    (mode, thrust_n, gimbal_x_rad, gimbal_z_rad),
                ),
                (
                    applied_gate,
                    mean_thrust,
                    (mode, thrust_n, gimbal_x_rad, gimbal_z_rad, [0.0, 1.0, 0.0]),
                    [0.0, thrust_n, 0.0, 0.0, 0.0, 0.0],
                ),
            )

    class FakeFactory:
        def __init__(self, physics_step_seconds: float, substeps: int) -> None:
            assert physics_step_seconds == 0.002
            assert substeps == 10

        def model_spec(self) -> object:
            return (
                "apollo_lander",
                4_932.0,
                [0.0, 2.012912912912913, 0.0],
                [6_332.0, 7_953.0, 5_879.0],
            )

        def propulsion_spec(self) -> object:
            rcs = [
                (
                    int(thruster),
                    thruster.name,
                    int(thruster.name[1]),
                    thruster.name[0],
                    [float(thruster), 3.0, 0.0],
                    directions[thruster.name[-1]],
                    444.822,
                    14_000_000,
                )
                for thruster in RCS_THRUSTER_ORDER
            ]
            dps = (
                [0.0, 1.24, 0.0],
                [0.0, 1.0, 0.0],
                4_670.633,
                28_023.796,
                43_903.947,
                float(np.deg2rad(6.0)),
                float(np.deg2rad(0.2)),
            )
            return rcs, dps

        def spawn(self, _state: list[float]) -> FakePlant:
            return FakePlant()

    monkeypatch.setattr(
        api,
        "_native",
        SimpleNamespace(ApolloPropulsionPlantFactory=FakeFactory),
    )
    factory = ApolloPropulsionPlantFactory.apollo11_touchdown()
    assert factory.propulsion_spec.rcs_thrusters[10].id is RcsThrusterId.B3A
    assert factory.propulsion_spec.dps.gimbal_rate_rad_s == pytest.approx(
        np.deg2rad(0.2)
    )

    plant = factory.spawn(ApolloState.identity())
    command = PropulsionCommand(
        rcs=RcsCommand.single_pulse(RcsThrusterId.B3A, 1_000_000),
        dps=DpsCommand.variable(4_670.633),
    )
    result = plant.step(command)

    assert result.snapshot.control_tick == 1
    assert result.requested_command == command
    assert (
        result.applied.rcs.applied_gate_on_time_ns[RcsThrusterId.B3A]
        == 14_000_000
    )
    assert result.applied.rcs.mean_thrust_n[RcsThrusterId.B3A] == 12.5
    assert result.applied.dps.mode is DpsMode.VARIABLE
    assert result.applied.mean_wrench_body.force_body_n[1] == 4_670.633

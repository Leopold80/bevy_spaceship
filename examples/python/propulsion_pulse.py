"""显式驱动 Apollo 11 RCS 短脉冲和 DPS 可调推力档。"""

import numpy as np

from apollo_sim import (
    ApolloPropulsionPlantFactory,
    ApolloState,
    DpsCommand,
    PropulsionCommand,
    RcsCommand,
    RcsThrusterId,
)


def main() -> None:
    factory = ApolloPropulsionPlantFactory.apollo11_touchdown()
    plant = factory.spawn(ApolloState.identity())

    # 历史标签描述喷管/羽流方向；spec.force_direction_body 才是飞船受力方向。
    thruster = RcsThrusterId.A1F
    short_request = PropulsionCommand(
        rcs=RcsCommand.single_pulse(thruster, 1_000_000),
        dps=DpsCommand.off(),
    )
    rcs_step = plant.step(short_request)
    print(
        f"RCS {thruster.name}: requested="
        f"{rcs_step.requested_command.rcs.on_time_ns[thruster]} ns, "
        f"applied_gate="
        f"{rcs_step.applied.rcs.applied_gate_on_time_ns[thruster]} ns, "
        f"mean_thrust={rcs_step.applied.rcs.mean_thrust_n[thruster]:.3f} N"
    )

    # RCS 有关断尾迹；先显式空跑一 tick，避免下一行的合力混入残余 RCS 冲量。
    plant.step(PropulsionCommand.all_off())

    dps_spec = factory.propulsion_spec.dps
    target_gimbal_x_rad = 2.0 * np.pi / 180.0
    target_gimbal_z_rad = -1.0 * np.pi / 180.0
    dps_step = plant.step(
        PropulsionCommand(
            rcs=RcsCommand.all_off(),
            # 这里提交的是目标摆角；AppliedDps 返回受 GDA 摆速限制后的实际摆角。
            dps=DpsCommand.variable(
                dps_spec.variable_min_thrust_n,
                gimbal_x_rad=target_gimbal_x_rad,
                gimbal_z_rad=target_gimbal_z_rad,
            ),
        )
    )
    print(
        "DPS: "
        f"mode={dps_step.applied.dps.mode.value}, "
        f"thrust={dps_step.applied.dps.thrust_n:.3f} N, "
        f"target_gimbal=({np.rad2deg(target_gimbal_x_rad):.3f}°, "
        f"{np.rad2deg(target_gimbal_z_rad):.3f}°), "
        f"actual_gimbal=({np.rad2deg(dps_step.applied.dps.gimbal_x_rad):.6f}°, "
        f"{np.rad2deg(dps_step.applied.dps.gimbal_z_rad):.6f}°), "
        f"mean_force_body={dps_step.applied.mean_wrench_body.force_body_n}"
    )

    # DPS 命令同样不跨 tick 隐式保持；连续工作必须每周期重复提交。
    plant.step(PropulsionCommand.all_off())


if __name__ == "__main__":
    main()

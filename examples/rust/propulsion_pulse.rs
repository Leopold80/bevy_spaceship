//! Rust 推进接口最小例程：直接操作一枚 RCS 喷口，再操作 DPS。
//!
//! 这里没有控制器或喷口分配器；调用方提交的就是 16 路阀门时间与 DPS 档位。

use apollo_mujoco::{
    ApolloPropulsionPlantFactory, ApolloState, DpsCommand, PropulsionCommand, RcsCommand,
    RcsThrusterId,
};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let factory = ApolloPropulsionPlantFactory::apollo11_touchdown()?;
    let spec = factory.propulsion_spec();
    let mut plant = factory.spawn(ApolloState::ZERO)?;

    // 请求 7 ms 会按实机最小脉冲约束提升为 14 ms；数组第 0 路是 A1U。
    let thruster_id = RcsThrusterId::new(0).expect("0 is a valid RCS id");
    let pulse = plant.step(PropulsionCommand {
        rcs: RcsCommand::single_pulse(thruster_id, 7_000_000),
        dps: DpsCommand::Off,
    })?;
    println!(
        "RCS {} requested=7ms applied={}ns mean_thrust={:.3}N",
        thruster_id.label(),
        pulse.applied.rcs[thruster_id.index()].applied_gate_on_time_ns,
        pulse.applied.rcs[thruster_id.index()].mean_thrust_n,
    );
    println!("RCS mean wrench: {:?}", pulse.applied.mean_wrench_body);

    // DPS 可调档会把低于 1,050 lbf 的请求钳到该档下限；摆角以弧度输入，
    // 但它是 GDA 的目标位置，不会在一个控制周期内瞬间到达。
    let descent = plant.step(PropulsionCommand {
        rcs: RcsCommand::OFF,
        dps: DpsCommand::Variable {
            thrust_n: 1_000.0,
            gimbal_x_rad: 2.0_f64.to_radians(),
            gimbal_z_rad: -1.0_f64.to_radians(),
        },
    })?;
    println!(
        "DPS mode={:?} applied_thrust={:.3}N actual_gimbal=({:.6}°, {:.6}°) direction={:?}",
        descent.applied.dps.mode,
        descent.applied.dps.thrust_n,
        descent.applied.dps.gimbal_x_rad.to_degrees(),
        descent.applied.dps.gimbal_z_rad.to_degrees(),
        descent.applied.dps.force_direction_body,
    );

    assert_eq!(
        pulse.applied.rcs[thruster_id.index()].applied_gate_on_time_ns,
        spec.rcs_thrusters[thruster_id.index()].minimum_pulse_ns
    );
    assert_eq!(descent.applied.dps.thrust_n, spec.dps.variable_min_thrust_n);
    let actual_gimbal_magnitude = descent
        .applied
        .dps
        .gimbal_x_rad
        .hypot(descent.applied.dps.gimbal_z_rad);
    let maximum_tick_delta = spec.dps.gimbal_rate_rad_s * factory.timing().control_step_seconds();
    assert!((actual_gimbal_magnitude - maximum_tick_delta).abs() < 1.0e-15);
    Ok(())
}

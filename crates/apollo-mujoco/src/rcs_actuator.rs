//! RCS 阀门/推力瞬态的确定性近似。
//!
//! NASA LM RCS News Reference 给出约 7 ms 后阀门完全打开，并明确说明
//! 14 ms 最小脉冲不能达到稳态 100 lbf；资料没有给出“前 7 ms 推力严格为零”的
//! 实测曲线。本模块把 7 ms 零推力延迟、20 ms 达到稳态和 8 ms 关断尾迹均作为
//! 显式工程近似，再对子步时间区间做解析积分；它不是完整阀门/燃烧室热流体模型。

/// 从打开阀门到开始建立推力的工程建模延迟；不是 NASA 直接给出的零推力区间。
pub(crate) const RCS_IGNITION_DELAY_NS: u64 = 7_000_000;
/// 连续开启后达到稳态推力的建模时刻。
pub(crate) const RCS_MODELED_FULL_THRUST_TIME_NS: u64 = 20_000_000;
/// 从满推力线性衰减到零所需的建模时间。
pub(crate) const RCS_MODELED_FALLOFF_NS: u64 = 8_000_000;

/// 单个 RCS 喷口跨控制周期保存的执行器状态。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct RcsActuatorState {
    gate_open: bool,
    thrust_fraction: f64,
    opening_elapsed_ns: u64,
    fall_start_fraction: f64,
    fall_elapsed_ns: u64,
}

impl RcsActuatorState {
    /// 周期边界处阀门是否仍保持开启。
    ///
    /// 只有上一周期的门控恰好延续到边界时才为真；关断尾迹仍有推力但阀门已关闭时为假。
    pub(crate) fn is_gate_open(&self) -> bool {
        self.gate_open
    }

    /// 推进一个物理子步，并返回该子步内的平均稳态推力比例。
    ///
    /// `gate_close_ns` 相对于当前控制周期起点。等于控制周期长度时，阀门在
    /// 周期边界保持开启，下一次调用可无缝继续稳态点火。
    pub(crate) fn advance_scheduled_interval(
        &mut self,
        interval_start_ns: u64,
        interval_duration_ns: u64,
        gate_close_ns: u64,
    ) -> f64 {
        debug_assert!(interval_duration_ns > 0);
        let interval_end_ns = interval_start_ns.saturating_add(interval_duration_ns);
        let open_end_ns = interval_end_ns.min(gate_close_ns);
        let open_duration_ns = open_end_ns.saturating_sub(interval_start_ns);
        let closed_duration_ns = interval_duration_ns - open_duration_ns;

        let mut integrated_fraction_ns = 0.0;
        if open_duration_ns > 0 {
            integrated_fraction_ns += self.advance_open(open_duration_ns);
        }
        if closed_duration_ns > 0 {
            integrated_fraction_ns += self.advance_closed(closed_duration_ns);
        }

        integrated_fraction_ns / interval_duration_ns as f64
    }

    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }

    fn advance_open(&mut self, duration_ns: u64) -> f64 {
        if !self.gate_open {
            self.gate_open = true;
            // 关断尾迹尚未归零时重新开启，按当前连续推力反推上升曲线位置，
            // 避免在控制周期边界制造不连续冲量。
            self.opening_elapsed_ns = if self.thrust_fraction > 0.0 {
                let ramp_ns = RCS_MODELED_FULL_THRUST_TIME_NS - RCS_IGNITION_DELAY_NS;
                RCS_IGNITION_DELAY_NS
                    .saturating_add((self.thrust_fraction * ramp_ns as f64).round() as u64)
            } else {
                0
            };
        }

        let start_ns = self.opening_elapsed_ns;
        let end_ns = start_ns.saturating_add(duration_ns);
        let integrated = integrate_open_curve(start_ns, end_ns);
        self.opening_elapsed_ns = end_ns.min(RCS_MODELED_FULL_THRUST_TIME_NS);
        self.thrust_fraction = open_fraction_at(self.opening_elapsed_ns);
        integrated
    }

    fn advance_closed(&mut self, duration_ns: u64) -> f64 {
        if self.gate_open {
            self.gate_open = false;
            self.fall_start_fraction = self.thrust_fraction;
            self.fall_elapsed_ns = 0;
        }

        if self.thrust_fraction <= 0.0 {
            self.thrust_fraction = 0.0;
            return 0.0;
        }

        let start_ns = self.fall_elapsed_ns;
        let end_ns = start_ns.saturating_add(duration_ns);
        let integrated = integrate_fall_curve(self.fall_start_fraction, start_ns, end_ns);
        self.fall_elapsed_ns = end_ns.min(RCS_MODELED_FALLOFF_NS);
        self.thrust_fraction = fall_fraction_at(self.fall_start_fraction, self.fall_elapsed_ns);
        integrated
    }
}

fn open_fraction_at(elapsed_ns: u64) -> f64 {
    if elapsed_ns <= RCS_IGNITION_DELAY_NS {
        0.0
    } else if elapsed_ns >= RCS_MODELED_FULL_THRUST_TIME_NS {
        1.0
    } else {
        (elapsed_ns - RCS_IGNITION_DELAY_NS) as f64
            / (RCS_MODELED_FULL_THRUST_TIME_NS - RCS_IGNITION_DELAY_NS) as f64
    }
}

fn fall_fraction_at(start_fraction: f64, elapsed_ns: u64) -> f64 {
    if elapsed_ns >= RCS_MODELED_FALLOFF_NS {
        0.0
    } else {
        start_fraction * (1.0 - elapsed_ns as f64 / RCS_MODELED_FALLOFF_NS as f64)
    }
}

/// 返回 `fraction × ns`，对分段线性曲线逐段使用梯形积分。
fn integrate_open_curve(start_ns: u64, end_ns: u64) -> f64 {
    integrate_piecewise_linear(
        start_ns,
        end_ns,
        &[0, RCS_IGNITION_DELAY_NS, RCS_MODELED_FULL_THRUST_TIME_NS],
        open_fraction_at,
    )
}

fn integrate_fall_curve(start_fraction: f64, start_ns: u64, end_ns: u64) -> f64 {
    integrate_piecewise_linear(start_ns, end_ns, &[0, RCS_MODELED_FALLOFF_NS], |time_ns| {
        fall_fraction_at(start_fraction, time_ns)
    })
}

fn integrate_piecewise_linear(
    start_ns: u64,
    end_ns: u64,
    breakpoints_ns: &[u64],
    value_at: impl Fn(u64) -> f64,
) -> f64 {
    if end_ns <= start_ns {
        return 0.0;
    }

    let mut cursor_ns = start_ns;
    let mut integral = 0.0;
    while cursor_ns < end_ns {
        let next_breakpoint = breakpoints_ns
            .iter()
            .copied()
            .find(|breakpoint| *breakpoint > cursor_ns)
            .unwrap_or(end_ns);
        let segment_end_ns = end_ns.min(next_breakpoint);
        let start_value = value_at(cursor_ns);
        let end_value = value_at(segment_end_ns);
        integral += 0.5 * (start_value + end_value) * (segment_end_ns - cursor_ns) as f64;
        cursor_ns = segment_end_ns;
    }
    integral
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fourteen_millisecond_pulse_does_not_reach_steady_thrust() {
        let mut actuator = RcsActuatorState::default();
        let mut impulse_fraction_ns = 0.0;
        for substep in 0..10 {
            let average =
                actuator.advance_scheduled_interval(substep * 2_000_000, 2_000_000, 14_000_000);
            impulse_fraction_ns += average * 2_000_000.0;
        }

        assert!(impulse_fraction_ns > 0.0);
        assert!(impulse_fraction_ns < 14_000_000.0);
        assert!(actuator.thrust_fraction < 1.0);
    }

    #[test]
    fn consecutive_full_control_ticks_reach_and_hold_steady_thrust() {
        let mut actuator = RcsActuatorState::default();
        for tick in 0..3 {
            let mut final_average = 0.0;
            for substep in 0..10 {
                final_average =
                    actuator.advance_scheduled_interval(substep * 2_000_000, 2_000_000, 20_000_000);
            }
            if tick == 0 {
                assert!(final_average < 1.0);
            }
        }
        assert_eq!(actuator.thrust_fraction, 1.0);

        let average = actuator.advance_scheduled_interval(0, 2_000_000, 20_000_000);
        assert!((average - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn seven_millisecond_breakpoint_is_integrated_inside_two_millisecond_step() {
        let mut actuator = RcsActuatorState::default();
        for substep in 0..3 {
            assert_eq!(
                actuator.advance_scheduled_interval(substep * 2_000_000, 2_000_000, 20_000_000,),
                0.0
            );
        }

        let average_6_to_8_ms =
            actuator.advance_scheduled_interval(6_000_000, 2_000_000, 20_000_000);
        let expected = 0.5 * (1_000_000.0 / 13_000_000.0) * 1_000_000.0 / 2_000_000.0;
        assert!((average_6_to_8_ms - expected).abs() < 1.0e-15);
    }

    #[test]
    fn reset_removes_tail_state() {
        let mut actuator = RcsActuatorState::default();
        for substep in 0..10 {
            actuator.advance_scheduled_interval(substep * 2_000_000, 2_000_000, 20_000_000);
        }
        actuator.reset();
        assert_eq!(actuator, RcsActuatorState::default());
        assert_eq!(actuator.advance_scheduled_interval(0, 2_000_000, 0), 0.0);
    }
}

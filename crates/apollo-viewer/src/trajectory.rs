use apollo_core::{
    ApolloState, BodyWrench, TelemetryFrame, TelemetryFrameError, TrajectoryHeader,
    TrajectoryHeaderError,
};
use glam::DQuat;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum TrajectoryError {
    #[error("failed to read trajectory: {0}")]
    Io(#[from] std::io::Error),
    #[error("trajectory is empty")]
    Empty,
    #[error("invalid JSON on line {line}: {source}")]
    Json {
        line: usize,
        #[source]
        source: serde_json::Error,
    },
    #[error("invalid trajectory header: {0}")]
    Header(#[from] TrajectoryHeaderError),
    #[error("control tick is not strictly increasing at frame {index}")]
    NonMonotonicTick { index: usize },
    #[error("invalid trajectory frame {index}: {source}")]
    InvalidFrame {
        index: usize,
        #[source]
        source: TelemetryFrameError,
    },
}

#[derive(Clone, Debug)]
pub struct Trajectory {
    header: TrajectoryHeader,
    frames: Vec<TelemetryFrame>,
}

#[derive(Clone, Copy, Debug)]
pub struct SampledFrame {
    pub state: ApolloState,
    /// 调用方记录了该时刻的期望姿态时，返回从期望机体系到世界系的旋转。
    pub desired_body_to_world: Option<DQuat>,
    /// 能从记录帧确定时，采样区间使用的请求动作。
    ///
    /// 稀疏轨迹的帧间动作不可恢复，此时为 `None`，viewer 不会伪造 wrench。
    pub requested_action: Option<BodyWrench>,
    /// 能从记录帧确定时，采样区间使用的实际动作。
    pub applied_action: Option<BodyWrench>,
    pub sim_time_seconds: f64,
}

impl Trajectory {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, TrajectoryError> {
        Self::from_reader(BufReader::new(File::open(path)?))
    }

    pub fn from_reader(reader: impl BufRead) -> Result<Self, TrajectoryError> {
        let mut non_empty_lines =
            reader
                .lines()
                .enumerate()
                .filter_map(|(index, line)| match line {
                    Ok(line) if line.trim().is_empty() => None,
                    line => Some((index + 1, line)),
                });

        let Some((header_line_number, header_line)) = non_empty_lines.next() else {
            return Err(TrajectoryError::Empty);
        };
        let header_line = header_line?;
        let header: TrajectoryHeader =
            serde_json::from_str(&header_line).map_err(|source| TrajectoryError::Json {
                line: header_line_number,
                source,
            })?;
        header.validate()?;

        let mut frames = Vec::new();
        for (line_number, line) in non_empty_lines {
            let line = line?;
            let frame: TelemetryFrame =
                serde_json::from_str(&line).map_err(|source| TrajectoryError::Json {
                    line: line_number,
                    source,
                })?;
            frames.push(frame);
        }

        let mut previous_control_tick = header.initial_snapshot.control_tick;
        for (index, frame) in frames.iter().enumerate() {
            frame
                .validate(header.timing)
                .map_err(|source| TrajectoryError::InvalidFrame { index, source })?;
            if frame.snapshot.control_tick <= previous_control_tick {
                return Err(TrajectoryError::NonMonotonicTick { index });
            }
            previous_control_tick = frame.snapshot.control_tick;
        }

        Ok(Self { header, frames })
    }

    /// 已校验的版本、模型、时序和初始快照。
    pub fn header(&self) -> &TrajectoryHeader {
        &self.header
    }

    /// 已校验且按控制 tick 排序的 post-step 帧。
    pub fn frames(&self) -> &[TelemetryFrame] {
        &self.frames
    }

    /// 是否至少记录了一个期望姿态样本。
    pub fn has_attitude_reference(&self) -> bool {
        self.header.initial_attitude_reference.is_some()
            || self
                .frames
                .iter()
                .any(|frame| frame.attitude_reference.is_some())
    }

    pub fn start_time_seconds(&self) -> f64 {
        self.header
            .initial_snapshot
            .sim_time_seconds(self.header.timing)
    }

    pub fn end_time_seconds(&self) -> f64 {
        self.frames.last().map_or_else(
            || self.start_time_seconds(),
            |frame| self.frame_time_seconds(frame),
        )
    }

    pub fn duration_seconds(&self) -> f64 {
        self.end_time_seconds() - self.start_time_seconds()
    }

    pub fn sample(&self, sim_time_seconds: f64) -> SampledFrame {
        if self.frames.is_empty() {
            return SampledFrame {
                state: self.header.initial_snapshot.state,
                desired_body_to_world: self
                    .header
                    .initial_attitude_reference
                    .map(|reference| reference.desired_body_to_world),
                requested_action: None,
                applied_action: None,
                sim_time_seconds: self.start_time_seconds(),
            };
        }

        // NaN 没有可解释的采样时刻；稳定地退回初态，避免把非有限值传播到
        // Bevy Transform。正负无穷仍由 clamp 映射到轨迹两端。
        let requested_time = if sim_time_seconds.is_nan() {
            self.start_time_seconds()
        } else {
            sim_time_seconds
        };
        let clamped = requested_time.clamp(self.start_time_seconds(), self.end_time_seconds());
        let upper = self
            .frames
            .partition_point(|frame| self.frame_time_seconds(frame) <= clamped);

        if upper >= self.frames.len() {
            return self.sample_from_frame(*self.frames.last().expect("trajectory has frames"));
        }

        let before = if upper == 0 {
            self.header.initial_snapshot
        } else {
            self.frames[upper - 1].snapshot
        };
        let after = self.frames[upper];
        let before_reference = if upper == 0 {
            self.header.initial_attitude_reference
        } else {
            self.frames[upper - 1].attitude_reference
        };
        let before_time = before.sim_time_seconds(self.header.timing);
        let after_time = self.frame_time_seconds(&after);
        let alpha = if after_time > before_time {
            ((clamped - before_time) / (after_time - before_time)).clamp(0.0, 1.0)
        } else {
            0.0
        };

        // `after` 帧中的动作只产生紧邻 after 的一个控制区间。若调用方稀疏
        // 记录，区间更早部分的动作未知，不能把 after wrench 扩展到整段。
        let action_start_physics_tick = self
            .header
            .timing
            .physics_ticks_for_control_ticks(after.snapshot.control_tick - 1)
            .expect("validated frame tick must have a representable preceding interval");
        let action_start = self
            .header
            .timing
            .sim_time_seconds(action_start_physics_tick);
        let action_is_known = clamped >= action_start;

        SampledFrame {
            state: interpolate_state(before.state, after.snapshot.state, alpha),
            desired_body_to_world: interpolate_optional_attitude(
                before_reference.map(|reference| reference.desired_body_to_world),
                after
                    .attitude_reference
                    .map(|reference| reference.desired_body_to_world),
                alpha,
            ),
            requested_action: action_is_known.then_some(after.requested_action),
            applied_action: action_is_known.then_some(after.applied_action),
            sim_time_seconds: clamped,
        }
    }

    fn frame_time_seconds(&self, frame: &TelemetryFrame) -> f64 {
        frame.snapshot.sim_time_seconds(self.header.timing)
    }

    fn sample_from_frame(&self, frame: TelemetryFrame) -> SampledFrame {
        SampledFrame {
            state: frame.snapshot.state,
            desired_body_to_world: frame
                .attitude_reference
                .map(|reference| reference.desired_body_to_world),
            requested_action: Some(frame.requested_action),
            applied_action: Some(frame.applied_action),
            sim_time_seconds: frame.snapshot.sim_time_seconds(self.header.timing),
        }
    }
}

fn interpolate_optional_attitude(
    before: Option<DQuat>,
    after: Option<DQuat>,
    alpha: f64,
) -> Option<DQuat> {
    if alpha <= f64::EPSILON {
        return before;
    }
    if alpha >= 1.0 - f64::EPSILON {
        return after;
    }
    let (before, mut after) = (before?, after?);
    if before.dot(after) < 0.0 {
        after = -after;
    }
    Some(before.slerp(after, alpha).normalize())
}

fn interpolate_state(before: ApolloState, after: ApolloState, alpha: f64) -> ApolloState {
    let mut after_attitude = after.body_to_world;
    if before.body_to_world.dot(after_attitude) < 0.0 {
        after_attitude = -after_attitude;
    }

    let body_to_world = before
        .body_to_world
        .slerp(after_attitude, alpha)
        .normalize();
    let angular_velocity_world = (before.body_to_world * before.angular_velocity_body_radps).lerp(
        after.body_to_world * after.angular_velocity_body_radps,
        alpha,
    );

    ApolloState {
        position_body_origin_world_m: before
            .position_body_origin_world_m
            .lerp(after.position_body_origin_world_m, alpha),
        body_to_world,
        linear_velocity_body_origin_world_mps: before
            .linear_velocity_body_origin_world_mps
            .lerp(after.linear_velocity_body_origin_world_mps, alpha),
        angular_velocity_body_radps: body_to_world.inverse() * angular_velocity_world,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use apollo_core::{PlantSnapshot, SimulationTiming};
    use glam::{DQuat, DVec3};
    use std::io::Cursor;

    fn sample_trajectory() -> String {
        let timing = SimulationTiming::APOLLO;
        let header = TrajectoryHeader::apollo(
            timing,
            PlantSnapshot::initial(ApolloState {
                position_body_origin_world_m: DVec3::new(-2.0, 0.0, 0.0),
                ..ApolloState::ZERO
            }),
        )
        .with_initial_desired_attitude(DQuat::IDENTITY);
        let first = TelemetryFrame {
            snapshot: PlantSnapshot {
                state: ApolloState::ZERO,
                control_tick: 1,
                physics_tick: 10,
            },
            requested_action: BodyWrench::ZERO,
            applied_action: BodyWrench::ZERO,
            attitude_reference: None,
        }
        .with_desired_attitude(DQuat::from_rotation_z(0.5));
        let second = TelemetryFrame {
            snapshot: PlantSnapshot {
                state: ApolloState {
                    position_body_origin_world_m: DVec3::new(2.0, 0.0, 0.0),
                    body_to_world: DQuat::from_rotation_y(std::f64::consts::PI),
                    ..ApolloState::ZERO
                },
                control_tick: 2,
                physics_tick: 20,
            },
            requested_action: BodyWrench {
                force_body_n: DVec3::X,
                torque_about_com_body_nm: DVec3::Y,
            },
            applied_action: BodyWrench::ZERO,
            attitude_reference: None,
        }
        .with_desired_attitude(DQuat::from_rotation_z(1.0));
        format!(
            "{}\n{}\n{}\n",
            serde_json::to_string(&header).unwrap(),
            serde_json::to_string(&first).unwrap(),
            serde_json::to_string(&second).unwrap(),
        )
    }

    #[test]
    fn parses_and_interpolates_versioned_jsonl() {
        let trajectory = Trajectory::from_reader(Cursor::new(sample_trajectory())).unwrap();
        assert_eq!(trajectory.frames().len(), 2);
        assert_eq!(trajectory.start_time_seconds(), 0.0);
        assert!((trajectory.duration_seconds() - 0.04).abs() < f64::EPSILON);
        assert_eq!(
            trajectory.sample(0.0).state.position_body_origin_world_m.x,
            -2.0
        );
        assert_eq!(trajectory.sample(f64::NAN).sim_time_seconds, 0.0);
        let sample = trajectory.sample(0.03);
        assert!((sample.state.position_body_origin_world_m.x - 1.0).abs() < 1.0e-12);
        assert_eq!(sample.requested_action.unwrap().force_body_n.x, 1.0);
        assert!(sample.state.body_to_world.is_normalized());
        let desired = sample.desired_body_to_world.unwrap();
        let expected = DQuat::from_rotation_z(0.75);
        assert!(desired.abs_diff_eq(expected, 1.0e-12));
        assert!(trajectory.has_attitude_reference());
    }

    #[test]
    fn header_only_trajectory_represents_a_zero_step_rollout() {
        let initial_state = ApolloState {
            position_body_origin_world_m: DVec3::new(3.0, -1.0, 2.0),
            ..ApolloState::ZERO
        };
        let header = TrajectoryHeader::apollo(
            SimulationTiming::APOLLO,
            PlantSnapshot::initial(initial_state),
        );
        let jsonl = format!("{}\n", serde_json::to_string(&header).unwrap());
        let trajectory = Trajectory::from_reader(Cursor::new(jsonl)).unwrap();

        assert!(trajectory.frames().is_empty());
        assert_eq!(trajectory.duration_seconds(), 0.0);
        let sample = trajectory.sample(10.0);
        assert_eq!(sample.state, initial_state);
        assert!(sample.applied_action.is_none());
        assert!(sample.desired_body_to_world.is_none());
        assert_eq!(sample.sim_time_seconds, 0.0);
    }

    #[test]
    fn sparse_trajectory_marks_unrecorded_actions_unknown() {
        let timing = SimulationTiming::APOLLO;
        let header = TrajectoryHeader::apollo(timing, PlantSnapshot::initial(ApolloState::ZERO));
        let frame = TelemetryFrame {
            snapshot: PlantSnapshot {
                state: ApolloState {
                    position_body_origin_world_m: DVec3::X,
                    ..ApolloState::ZERO
                },
                control_tick: 5,
                physics_tick: 50,
            },
            requested_action: BodyWrench {
                force_body_n: DVec3::X,
                torque_about_com_body_nm: DVec3::ZERO,
            },
            applied_action: BodyWrench::ZERO,
            attitude_reference: None,
        };
        let jsonl = format!(
            "{}\n{}\n",
            serde_json::to_string(&header).unwrap(),
            serde_json::to_string(&frame).unwrap()
        );
        let trajectory = Trajectory::from_reader(Cursor::new(jsonl)).unwrap();

        assert!(trajectory.sample(0.04).applied_action.is_none());
        assert!(trajectory.sample(0.08).applied_action.is_some());
    }

    #[test]
    fn angular_velocity_interpolation_preserves_a_constant_world_vector() {
        let before = ApolloState {
            angular_velocity_body_radps: DVec3::X,
            ..ApolloState::ZERO
        };
        let after_attitude = DQuat::from_rotation_z(std::f64::consts::PI);
        let after = ApolloState {
            body_to_world: after_attitude,
            angular_velocity_body_radps: after_attitude.inverse() * DVec3::X,
            ..ApolloState::ZERO
        };

        let interpolated = interpolate_state(before, after, 0.5);
        let angular_velocity_world =
            interpolated.body_to_world * interpolated.angular_velocity_body_radps;
        assert!((angular_velocity_world - DVec3::X).length() < 1.0e-12);
    }

    #[test]
    fn rejects_misaligned_ticks() {
        let broken = sample_trajectory().replace("\"physics_tick\":20", "\"physics_tick\":19");
        assert!(matches!(
            Trajectory::from_reader(Cursor::new(broken)),
            Err(TrajectoryError::InvalidFrame {
                source: TelemetryFrameError::MisalignedPhysicsTick { .. },
                ..
            })
        ));
    }
}

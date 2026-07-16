use crate::{BodyWrench, PlantSnapshot, PlantStep, SimulationTiming, ValidationError};
use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fmt;
use std::io::{self, Write};

/// JSONL 轨迹的稳定格式标识。
pub const TELEMETRY_FORMAT: &str = "apollo-telemetry-jsonl";
/// 当前 JSONL 轨迹格式版本。
pub const TELEMETRY_FORMAT_VERSION: u32 = 1;
/// 当前轨迹格式中由 Apollo viewer 支持的模型标识。
pub const APOLLO_TELEMETRY_MODEL: &str = "apollo_lander";

/// JSONL 轨迹的首行。
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrajectoryHeader {
    /// 格式标识，应为 [`TELEMETRY_FORMAT`]。
    pub format: String,
    /// 格式版本，应为 [`TELEMETRY_FORMAT_VERSION`]。
    pub version: u32,
    /// 生成轨迹的模型名。
    pub model: String,
    /// 轨迹的固定仿真时序。
    pub timing: SimulationTiming,
    /// reset 后、任何动作执行前的零 tick 快照。
    pub initial_snapshot: PlantSnapshot,
}

impl TrajectoryHeader {
    /// 创建当前 Apollo 格式的 header。
    pub fn apollo(timing: SimulationTiming, initial_snapshot: PlantSnapshot) -> Self {
        Self {
            format: TELEMETRY_FORMAT.to_owned(),
            version: TELEMETRY_FORMAT_VERSION,
            model: APOLLO_TELEMETRY_MODEL.to_owned(),
            timing,
            initial_snapshot,
        }
    }

    /// 拒绝未知格式、版本、模型或无效的初始快照。
    pub fn validate(&self) -> Result<(), TrajectoryHeaderError> {
        if self.format != TELEMETRY_FORMAT {
            return Err(TrajectoryHeaderError::UnsupportedFormat(
                self.format.clone(),
            ));
        }
        if self.version != TELEMETRY_FORMAT_VERSION {
            return Err(TrajectoryHeaderError::UnsupportedVersion(self.version));
        }
        if self.model != APOLLO_TELEMETRY_MODEL {
            return Err(TrajectoryHeaderError::UnsupportedModel(self.model.clone()));
        }
        self.initial_snapshot
            .state
            .validate()
            .map_err(TrajectoryHeaderError::InvalidInitialState)?;
        if self.initial_snapshot.control_tick != 0 || self.initial_snapshot.physics_tick != 0 {
            return Err(TrajectoryHeaderError::InitialSnapshotNotAtZero {
                control_tick: self.initial_snapshot.control_tick,
                physics_tick: self.initial_snapshot.physics_tick,
            });
        }
        Ok(())
    }
}

/// 轨迹 header 与当前读写器不兼容。
#[derive(Clone, Debug, PartialEq)]
pub enum TrajectoryHeaderError {
    /// 未知格式标识。
    UnsupportedFormat(String),
    /// 未知格式版本。
    UnsupportedVersion(u32),
    /// 当前读取器不支持该模型。
    UnsupportedModel(String),
    /// 初始状态包含无效数值。
    InvalidInitialState(ValidationError),
    /// 初始快照不是 reset 后的零 tick 状态。
    InitialSnapshotNotAtZero {
        control_tick: u64,
        physics_tick: u64,
    },
}

impl fmt::Display for TrajectoryHeaderError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedFormat(format) => {
                write!(formatter, "unsupported trajectory format: {format}")
            }
            Self::UnsupportedVersion(version) => {
                write!(formatter, "unsupported trajectory version: {version}")
            }
            Self::UnsupportedModel(model) => {
                write!(formatter, "unsupported trajectory model: {model}")
            }
            Self::InvalidInitialState(error) => {
                write!(formatter, "invalid initial trajectory state: {error}")
            }
            Self::InitialSnapshotNotAtZero {
                control_tick,
                physics_tick,
            } => write!(
                formatter,
                "initial trajectory snapshot must be at control tick 0 and physics tick 0 (got {control_tick} and {physics_tick})"
            ),
        }
    }
}

impl Error for TrajectoryHeaderError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidInitialState(error) => Some(error),
            Self::UnsupportedFormat(_)
            | Self::UnsupportedVersion(_)
            | Self::UnsupportedModel(_)
            | Self::InitialSnapshotNotAtZero { .. } => None,
        }
    }
}

/// JSONL 轨迹的显式记录器。
///
/// 记录器只持有输出流，不持有 plant 或任何运行循环。调用方必须
/// 在每个需要记录的 tick 上显式调用 [`write_frame`](Self::write_frame)。
pub struct JsonlTrajectoryWriter<W: Write> {
    writer: W,
    timing: SimulationTiming,
    last_control_tick: Option<u64>,
}

impl<W: Write> JsonlTrajectoryWriter<W> {
    /// 创建记录器并立即把版本化 header 写入第一行。
    pub fn new(mut writer: W, header: TrajectoryHeader) -> Result<Self, TrajectoryWriteError> {
        header.validate().map_err(TrajectoryWriteError::Header)?;
        write_json_line(&mut writer, &header)?;
        Ok(Self {
            writer,
            timing: header.timing,
            last_control_tick: Some(header.initial_snapshot.control_tick),
        })
    }

    /// 把一个遥测帧写成一行 JSON。
    pub fn write_frame(&mut self, frame: &TelemetryFrame) -> Result<(), TrajectoryWriteError> {
        frame
            .validate(self.timing)
            .map_err(TrajectoryWriteError::InvalidFrame)?;
        if let Some(previous) = self.last_control_tick
            && frame.snapshot.control_tick <= previous
        {
            return Err(TrajectoryWriteError::NonMonotonicControlTick {
                previous,
                current: frame.snapshot.control_tick,
            });
        }

        write_json_line(&mut self.writer, frame)?;
        self.last_control_tick = Some(frame.snapshot.control_tick);
        Ok(())
    }

    /// 借用底层输出流。
    pub fn get_ref(&self) -> &W {
        &self.writer
    }

    /// 可变借用底层输出流。
    pub fn get_mut(&mut self) -> &mut W {
        &mut self.writer
    }

    /// 消耗记录器并取回底层输出流。
    pub fn into_inner(self) -> W {
        self.writer
    }
}

fn write_json_line<W: Write, T: Serialize>(
    writer: &mut W,
    value: &T,
) -> Result<(), TrajectoryWriteError> {
    serde_json::to_writer(&mut *writer, value).map_err(TrajectoryWriteError::Serialize)?;
    writer.write_all(b"\n").map_err(TrajectoryWriteError::Io)
}

/// JSONL 记录错误。
#[derive(Debug)]
pub enum TrajectoryWriteError {
    /// header 格式或版本不受支持。
    Header(TrajectoryHeaderError),
    /// 遥测帧包含无效数值或与 header 时序不一致。
    InvalidFrame(TelemetryFrameError),
    /// 控制 tick 没有严格递增。
    NonMonotonicControlTick { previous: u64, current: u64 },
    /// JSON 序列化失败。
    Serialize(serde_json::Error),
    /// 底层输出失败。
    Io(io::Error),
}

impl fmt::Display for TrajectoryWriteError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Header(error) => write!(formatter, "invalid trajectory header: {error}"),
            Self::InvalidFrame(error) => write!(formatter, "invalid telemetry frame: {error}"),
            Self::NonMonotonicControlTick { previous, current } => write!(
                formatter,
                "control tick must be strictly increasing (previous: {previous}, current: {current})"
            ),
            Self::Serialize(error) => write!(formatter, "failed to serialize trajectory: {error}"),
            Self::Io(error) => write!(formatter, "failed to write trajectory: {error}"),
        }
    }
}

impl Error for TrajectoryWriteError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Header(error) => Some(error),
            Self::InvalidFrame(error) => Some(error),
            Self::NonMonotonicControlTick { .. } => None,
            Self::Serialize(error) => Some(error),
            Self::Io(error) => Some(error),
        }
    }
}

/// 一个控制 tick 的后端中立遥测帧。
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TelemetryFrame {
    /// 本 tick 动作执行后的 plant 快照。
    pub snapshot: PlantSnapshot,
    /// 调用方请求的动作。
    pub requested_action: BodyWrench,
    /// 后端实际应用的动作。
    pub applied_action: BodyWrench,
}

impl TelemetryFrame {
    /// 校验数值、四元数和该帧相对轨迹时序的 tick 对齐关系。
    pub fn validate(&self, timing: SimulationTiming) -> Result<(), TelemetryFrameError> {
        self.snapshot
            .state
            .validate()
            .map_err(TelemetryFrameError::InvalidState)?;
        self.requested_action
            .validate()
            .map_err(TelemetryFrameError::InvalidRequestedAction)?;
        self.applied_action
            .validate()
            .map_err(TelemetryFrameError::InvalidAppliedAction)?;

        let expected_physics_tick = timing
            .physics_ticks_for_control_ticks(self.snapshot.control_tick)
            .ok_or(TelemetryFrameError::TickOverflow {
                control_tick: self.snapshot.control_tick,
            })?;
        if self.snapshot.physics_tick != expected_physics_tick {
            return Err(TelemetryFrameError::MisalignedPhysicsTick {
                control_tick: self.snapshot.control_tick,
                physics_tick: self.snapshot.physics_tick,
                expected_physics_tick,
            });
        }
        Ok(())
    }
}

/// 单个遥测帧违反持久格式契约。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TelemetryFrameError {
    InvalidState(ValidationError),
    InvalidRequestedAction(ValidationError),
    InvalidAppliedAction(ValidationError),
    TickOverflow {
        control_tick: u64,
    },
    MisalignedPhysicsTick {
        control_tick: u64,
        physics_tick: u64,
        expected_physics_tick: u64,
    },
}

impl fmt::Display for TelemetryFrameError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidState(error) => write!(formatter, "invalid state: {error}"),
            Self::InvalidRequestedAction(error) => {
                write!(formatter, "invalid requested action: {error}")
            }
            Self::InvalidAppliedAction(error) => {
                write!(formatter, "invalid applied action: {error}")
            }
            Self::TickOverflow { control_tick } => {
                write!(
                    formatter,
                    "physics tick overflows for control tick {control_tick}"
                )
            }
            Self::MisalignedPhysicsTick {
                control_tick,
                physics_tick,
                expected_physics_tick,
            } => write!(
                formatter,
                "physics tick {physics_tick} does not match control tick {control_tick} (expected {expected_physics_tick})"
            ),
        }
    }
}

impl Error for TelemetryFrameError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidState(error)
            | Self::InvalidRequestedAction(error)
            | Self::InvalidAppliedAction(error) => Some(error),
            Self::TickOverflow { .. } | Self::MisalignedPhysicsTick { .. } => None,
        }
    }
}

impl From<PlantStep> for TelemetryFrame {
    fn from(step: PlantStep) -> Self {
        Self {
            snapshot: step.snapshot,
            requested_action: step.requested_action,
            applied_action: step.applied_action,
        }
    }
}

impl From<TelemetryFrame> for PlantStep {
    fn from(frame: TelemetryFrame) -> Self {
        Self {
            snapshot: frame.snapshot,
            requested_action: frame.requested_action,
            applied_action: frame.applied_action,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ApolloState;

    fn test_header() -> TrajectoryHeader {
        TrajectoryHeader::apollo(
            SimulationTiming::APOLLO,
            PlantSnapshot::initial(ApolloState::ZERO),
        )
    }

    #[test]
    fn header_and_frame_round_trip_through_json() {
        let header = test_header();
        let header_json = serde_json::to_string(&header).unwrap();
        let decoded_header: TrajectoryHeader = serde_json::from_str(&header_json).unwrap();
        assert_eq!(decoded_header, header);
        decoded_header.validate().unwrap();

        let frame = TelemetryFrame {
            snapshot: PlantSnapshot {
                state: ApolloState::ZERO,
                control_tick: 1,
                physics_tick: 10,
            },
            requested_action: BodyWrench::ZERO,
            applied_action: BodyWrench::ZERO,
        };
        let frame_json = serde_json::to_string(&frame).unwrap();
        let decoded_frame: TelemetryFrame = serde_json::from_str(&frame_json).unwrap();
        assert_eq!(decoded_frame, frame);
    }

    #[test]
    fn header_rejects_unknown_format_and_version() {
        let mut header = test_header();
        header.format = "another-format".to_owned();
        assert!(matches!(
            header.validate(),
            Err(TrajectoryHeaderError::UnsupportedFormat(_))
        ));

        let mut header = test_header();
        header.version += 1;
        assert_eq!(
            header.validate(),
            Err(TrajectoryHeaderError::UnsupportedVersion(2))
        );

        let mut header = test_header();
        header.model = "mars_rover".to_owned();
        assert_eq!(
            header.validate(),
            Err(TrajectoryHeaderError::UnsupportedModel(
                "mars_rover".to_owned()
            ))
        );

        let mut header = test_header();
        header.initial_snapshot.control_tick = 1;
        assert!(matches!(
            header.validate(),
            Err(TrajectoryHeaderError::InitialSnapshotNotAtZero { .. })
        ));
    }

    #[test]
    fn jsonl_writer_emits_one_header_and_one_line_per_explicit_frame() {
        let header = test_header();
        let mut writer = JsonlTrajectoryWriter::new(Vec::new(), header.clone()).unwrap();
        let frame = TelemetryFrame {
            snapshot: PlantSnapshot {
                state: ApolloState::ZERO,
                control_tick: 1,
                physics_tick: 10,
            },
            requested_action: BodyWrench::ZERO,
            applied_action: BodyWrench::ZERO,
        };
        writer.write_frame(&frame).unwrap();

        let bytes = writer.into_inner();
        let text = std::str::from_utf8(&bytes).unwrap();
        let mut lines = text.lines();
        assert_eq!(
            serde_json::from_str::<TrajectoryHeader>(lines.next().unwrap()).unwrap(),
            header
        );
        assert_eq!(
            serde_json::from_str::<TelemetryFrame>(lines.next().unwrap()).unwrap(),
            frame
        );
        assert_eq!(lines.next(), None);
        assert!(text.ends_with('\n'));
    }

    #[test]
    fn jsonl_writer_rejects_an_unknown_header_before_writing() {
        let mut header = test_header();
        header.version = 99;
        let error = match JsonlTrajectoryWriter::new(Vec::new(), header) {
            Ok(_) => panic!("unknown versions must be rejected"),
            Err(error) => error,
        };
        assert!(matches!(
            error,
            TrajectoryWriteError::Header(TrajectoryHeaderError::UnsupportedVersion(99))
        ));
    }

    #[test]
    fn jsonl_writer_rejects_invalid_or_non_monotonic_frames() {
        let header = test_header();
        let mut writer = JsonlTrajectoryWriter::new(Vec::new(), header).unwrap();
        let first = TelemetryFrame {
            snapshot: PlantSnapshot {
                state: ApolloState::ZERO,
                control_tick: 1,
                physics_tick: 10,
            },
            requested_action: BodyWrench::ZERO,
            applied_action: BodyWrench::ZERO,
        };
        writer.write_frame(&first).unwrap();

        assert!(matches!(
            writer.write_frame(&first),
            Err(TrajectoryWriteError::NonMonotonicControlTick { .. })
        ));

        let misaligned = TelemetryFrame {
            snapshot: PlantSnapshot {
                control_tick: 2,
                physics_tick: 19,
                ..first.snapshot
            },
            ..first
        };
        assert!(matches!(
            writer.write_frame(&misaligned),
            Err(TrajectoryWriteError::InvalidFrame(
                TelemetryFrameError::MisalignedPhysicsTick { .. }
            ))
        ));
    }
}

use crate::{BodyWrench, PlantSnapshot, SimulationTiming};
use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fmt;

/// Apollo LM RCS 喷口数量。
pub const RCS_THRUSTER_COUNT: usize = 16;
/// 单个 RCS 喷口的额定稳态推力：100 lbf。
pub const RCS_STEADY_THRUST_N: f64 = 444.822_161_526_05;
/// NASA 资料给出的最小 RCS 脉冲时间。
pub const RCS_MINIMUM_PULSE_NS: u64 = 14_000_000;
/// Apollo 11 LM-5 DPS 可调段最小推力：1,050 lbf。
pub const DPS_VARIABLE_MIN_THRUST_N: f64 = 4_670.632_696_023_525;
/// Apollo 11 LM-5 DPS 可调段最大推力：6,300 lbf。
pub const DPS_VARIABLE_MAX_THRUST_N: f64 = 28_023.796_176_141_15;
/// Apollo 11 LM-5 DPS 独立全推力档：9,870 lbf。
pub const DPS_FULL_THRUST_N: f64 = 43_903.947_342_622_13;
/// DPS 推力矢量相对名义轴的最大圆锥半角。
pub const DPS_MAXIMUM_GIMBAL_RAD: f64 = 6.0_f64.to_radians();
/// Apollo 11 DPS 万向节驱动执行器的额定摆速：0.2°/s。
pub const DPS_GIMBAL_RATE_RAD_S: f64 = 0.2_f64.to_radians();

const INCH_TO_M: f64 = 0.0254;
const NASA_RCS_REFERENCE_X_IN: f64 = 254.0;
const MODEL_RCS_REFERENCE_Y_M: f64 = 3.0;

/// 稳定的 16 路 RCS 索引。
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(transparent)]
pub struct RcsThrusterId(u8);

impl RcsThrusterId {
    pub const ALL: [Self; RCS_THRUSTER_COUNT] = [
        Self(0),
        Self(1),
        Self(2),
        Self(3),
        Self(4),
        Self(5),
        Self(6),
        Self(7),
        Self(8),
        Self(9),
        Self(10),
        Self(11),
        Self(12),
        Self(13),
        Self(14),
        Self(15),
    ];

    pub const fn new(index: u8) -> Option<Self> {
        if (index as usize) < RCS_THRUSTER_COUNT {
            Some(Self(index))
        } else {
            None
        }
    }

    pub const fn index(self) -> usize {
        self.0 as usize
    }

    /// 返回 Apollo Operations Handbook 中的稳定喷口标签。
    pub const fn label(self) -> &'static str {
        RCS_THRUSTER_LABELS[self.index()]
    }
}

impl<'de> Deserialize<'de> for RcsThrusterId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let index = u8::deserialize(deserializer)?;
        Self::new(index).ok_or_else(|| {
            <D::Error as serde::de::Error>::custom(format!(
                "RCS thruster index {index} is outside 0..{RCS_THRUSTER_COUNT}"
            ))
        })
    }
}

impl fmt::Display for RcsThrusterId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}", self.0)
    }
}

/// 与 [`RcsThrusterId`] 0..15 一一对应的 Apollo 历史标签。
pub const RCS_THRUSTER_LABELS: [&str; RCS_THRUSTER_COUNT] = [
    "A1U", "B1D", "A1F", "B1L", "B2U", "A2D", "A2A", "B2L", "A3U", "B3D", "B3A", "A3R", "B4U",
    "A4D", "B4F", "A4R",
];

/// RCS 四联装编号。
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum RcsQuad {
    Quad1,
    Quad2,
    Quad3,
    Quad4,
}

/// RCS 两套独立推进剂供给系统。
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum RcsFeedSystem {
    A,
    B,
}

/// 单个 RCS 喷口的后端中立规格。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcsThrusterSpec {
    pub id: RcsThrusterId,
    /// Apollo Operations Handbook 历史标签。末尾字母表示喷管/羽流方向。
    pub label: &'static str,
    pub quad: RcsQuad,
    pub feed_system: RcsFeedSystem,
    /// 推力作用点，使用本项目机体系，单位 m。
    pub position_body_m: DVec3,
    /// 飞船受到的力方向；它与历史标签所表示的羽流方向相反。
    pub force_direction_body: DVec3,
    pub steady_thrust_n: f64,
    pub minimum_pulse_ns: u64,
}

/// Apollo 11 下降发动机规格。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DpsSpec {
    pub gimbal_pivot_body_m: DVec3,
    pub nominal_force_direction_body: DVec3,
    pub variable_min_thrust_n: f64,
    pub variable_max_thrust_n: f64,
    pub full_thrust_n: f64,
    pub maximum_gimbal_rad: f64,
    /// 万向节从当前位置追踪目标位置的最大角速率，单位 rad/s。
    pub gimbal_rate_rad_s: f64,
}

/// 当前完整 Apollo 11 LM 的推进系统规格。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApolloPropulsionSpec {
    pub rcs_thrusters: [RcsThrusterSpec; RCS_THRUSTER_COUNT],
    pub dps: DpsSpec,
}

impl ApolloPropulsionSpec {
    /// Apollo 11 LM-5 完整着陆构型：16 个 RCS 加下降级 DPS。
    pub fn apollo11_touchdown() -> Self {
        // Quad IV 的站位来自 NASA/Grumman LM Data Book Vol. II §4.8.7.1。
        // NASA X=254 in 被整体平移到当前简化模型 y=3.0 m；NASA Y/Z
        // 分别映射到 code X/Z。其余 quad 按实机象限关系镜像。
        let mut thrusters = Vec::with_capacity(RCS_THRUSTER_COUNT);
        push_quad(
            &mut thrusters,
            RcsQuad::Quad1,
            -1.0,
            1.0,
            [
                ("A1U", RcsFeedSystem::A, JetKind::Up),
                ("B1D", RcsFeedSystem::B, JetKind::Down),
                ("A1F", RcsFeedSystem::A, JetKind::Forward),
                ("B1L", RcsFeedSystem::B, JetKind::Left),
            ],
        );
        push_quad(
            &mut thrusters,
            RcsQuad::Quad2,
            -1.0,
            -1.0,
            [
                ("B2U", RcsFeedSystem::B, JetKind::Up),
                ("A2D", RcsFeedSystem::A, JetKind::Down),
                ("A2A", RcsFeedSystem::A, JetKind::Aft),
                ("B2L", RcsFeedSystem::B, JetKind::Left),
            ],
        );
        push_quad(
            &mut thrusters,
            RcsQuad::Quad3,
            1.0,
            -1.0,
            [
                ("A3U", RcsFeedSystem::A, JetKind::Up),
                ("B3D", RcsFeedSystem::B, JetKind::Down),
                ("B3A", RcsFeedSystem::B, JetKind::Aft),
                ("A3R", RcsFeedSystem::A, JetKind::Right),
            ],
        );
        push_quad(
            &mut thrusters,
            RcsQuad::Quad4,
            1.0,
            1.0,
            [
                ("B4U", RcsFeedSystem::B, JetKind::Up),
                ("A4D", RcsFeedSystem::A, JetKind::Down),
                ("B4F", RcsFeedSystem::B, JetKind::Forward),
                ("A4R", RcsFeedSystem::A, JetKind::Right),
            ],
        );

        let rcs_thrusters: [RcsThrusterSpec; RCS_THRUSTER_COUNT] = thrusters
            .try_into()
            .expect("Apollo RCS specification must contain 16 thrusters");
        let spec = Self {
            rcs_thrusters,
            dps: DpsSpec {
                // 当前简化下降级几何中心；不是 NASA 万向节测绘坐标。
                gimbal_pivot_body_m: DVec3::new(0.0, 1.24, 0.0),
                nominal_force_direction_body: DVec3::Y,
                variable_min_thrust_n: DPS_VARIABLE_MIN_THRUST_N,
                variable_max_thrust_n: DPS_VARIABLE_MAX_THRUST_N,
                full_thrust_n: DPS_FULL_THRUST_N,
                maximum_gimbal_rad: DPS_MAXIMUM_GIMBAL_RAD,
                gimbal_rate_rad_s: DPS_GIMBAL_RATE_RAD_S,
            },
        };
        debug_assert!(spec.validate().is_ok());
        spec
    }

    pub fn validate(&self) -> Result<(), PropulsionValidationError> {
        let mut quad_counts = [0_usize; 4];
        let mut feed_counts = [[0_usize; 2]; 4];
        for (index, thruster) in self.rcs_thrusters.iter().enumerate() {
            if thruster.id.index() != index {
                return Err(invalid(format!(
                    "RCS index {index} has mismatched id {}",
                    thruster.id
                )));
            }
            if thruster.label.is_empty() {
                return Err(invalid(format!("RCS index {index} has an empty label")));
            }
            if !thruster.position_body_m.is_finite()
                || !thruster.force_direction_body.is_finite()
                || (thruster.force_direction_body.length() - 1.0).abs() > 1.0e-12
            {
                return Err(invalid(format!(
                    "RCS '{}' has invalid position or direction",
                    thruster.label
                )));
            }
            if !thruster.steady_thrust_n.is_finite()
                || thruster.steady_thrust_n <= 0.0
                || thruster.minimum_pulse_ns == 0
            {
                return Err(invalid(format!(
                    "RCS '{}' has invalid thrust or minimum pulse",
                    thruster.label
                )));
            }
            let quad = quad_index(thruster.quad);
            let feed = match thruster.feed_system {
                RcsFeedSystem::A => 0,
                RcsFeedSystem::B => 1,
            };
            quad_counts[quad] += 1;
            feed_counts[quad][feed] += 1;
        }
        if quad_counts != [4; 4] || feed_counts != [[2, 2]; 4] {
            return Err(invalid(
                "RCS must be four quads with two A and two B jets each",
            ));
        }

        self.dps.validate()
    }

    pub fn validate_for_timing(
        &self,
        timing: SimulationTiming,
    ) -> Result<(), PropulsionValidationError> {
        self.validate()?;
        let control_step_ns = control_step_u64(timing)?;
        let minimum_pulse_ns = self
            .rcs_thrusters
            .iter()
            .map(|thruster| thruster.minimum_pulse_ns)
            .max()
            .unwrap_or(0);
        if control_step_ns < minimum_pulse_ns {
            return Err(invalid(format!(
                "control step {control_step_ns} ns is shorter than RCS minimum pulse {minimum_pulse_ns} ns"
            )));
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
enum JetKind {
    Up,
    Down,
    Forward,
    Aft,
    Right,
    Left,
}

fn push_quad(
    output: &mut Vec<RcsThrusterSpec>,
    quad: RcsQuad,
    nasa_y_sign: f64,
    nasa_z_sign: f64,
    jets: [(&'static str, RcsFeedSystem, JetKind); 4],
) {
    for (label, feed_system, kind) in jets {
        let id = RcsThrusterId::new(output.len() as u8).unwrap();
        output.push(RcsThrusterSpec {
            id,
            label,
            quad,
            feed_system,
            position_body_m: rcs_position_body_m(kind, nasa_y_sign, nasa_z_sign),
            force_direction_body: rcs_force_direction_body(kind),
            steady_thrust_n: RCS_STEADY_THRUST_N,
            minimum_pulse_ns: RCS_MINIMUM_PULSE_NS,
        });
    }
}

fn rcs_position_body_m(kind: JetKind, nasa_y_sign: f64, nasa_z_sign: f64) -> DVec3 {
    let (nasa_x_in, nasa_y_in, nasa_z_in) = match kind {
        JetKind::Up => (258.8, 66.1, 66.1),
        JetKind::Down => (248.7, 66.1, 66.1),
        JetKind::Forward | JetKind::Aft => (254.0, 61.5, 66.35),
        JetKind::Right | JetKind::Left => (254.0, 66.35, 61.5),
    };
    DVec3::new(
        nasa_y_sign * nasa_y_in * INCH_TO_M,
        MODEL_RCS_REFERENCE_Y_M + (nasa_x_in - NASA_RCS_REFERENCE_X_IN) * INCH_TO_M,
        nasa_z_sign * nasa_z_in * INCH_TO_M,
    )
}

fn rcs_force_direction_body(kind: JetKind) -> DVec3 {
    // 历史字母表示喷管/羽流方向，因此飞船受到的力必须取反。
    match kind {
        JetKind::Up => DVec3::NEG_Y,
        JetKind::Down => DVec3::Y,
        JetKind::Forward => DVec3::NEG_Z,
        JetKind::Aft => DVec3::Z,
        JetKind::Right => DVec3::NEG_X,
        JetKind::Left => DVec3::X,
    }
}

fn quad_index(quad: RcsQuad) -> usize {
    match quad {
        RcsQuad::Quad1 => 0,
        RcsQuad::Quad2 => 1,
        RcsQuad::Quad3 => 2,
        RcsQuad::Quad4 => 3,
    }
}

/// 16 路 RCS 阀门请求。时间从当前控制周期起点计算。
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
pub struct RcsCommand {
    pub on_time_ns: [u64; RCS_THRUSTER_COUNT],
}

impl RcsCommand {
    pub const OFF: Self = Self {
        on_time_ns: [0; RCS_THRUSTER_COUNT],
    };

    pub const fn off() -> Self {
        Self::OFF
    }

    pub const fn from_on_times(on_time_ns: [u64; RCS_THRUSTER_COUNT]) -> Self {
        Self { on_time_ns }
    }

    pub fn single_pulse(id: RcsThrusterId, duration_ns: u64) -> Self {
        let mut command = Self::OFF;
        command.on_time_ns[id.index()] = duration_ns;
        command
    }

    pub fn with_on_time(mut self, id: RcsThrusterId, duration_ns: u64) -> Self {
        self.on_time_ns[id.index()] = duration_ns;
        self
    }

    pub fn hold(
        ids: impl IntoIterator<Item = RcsThrusterId>,
        timing: SimulationTiming,
    ) -> Result<Self, PropulsionValidationError> {
        let duration_ns = control_step_u64(timing)?;
        let mut command = Self::OFF;
        for id in ids {
            command.on_time_ns[id.index()] = duration_ns;
        }
        Ok(command)
    }

    /// 在假定所有阀门于周期起点均为关闭状态时，校验并应用最小脉冲约束。
    ///
    /// 这是无状态辅助方法，适合独立的新脉冲。保存执行器历史的 plant 不应在阀门已经
    /// 跨周期保持开启时直接使用它，否则会把连续点火的末段误当成新的最小脉冲；这类
    /// 调用方应使用 [`Self::applied_gate_on_times_with_initial_gate_state`]。
    pub fn applied_gate_on_times(
        &self,
        spec: ApolloPropulsionSpec,
        timing: SimulationTiming,
    ) -> Result<[u64; RCS_THRUSTER_COUNT], PropulsionValidationError> {
        self.applied_gate_on_times_with_initial_gate_state(
            spec,
            timing,
            [false; RCS_THRUSTER_COUNT],
        )
    }

    /// 根据周期起点的真实阀门状态，校验并应用最小脉冲约束。
    ///
    /// `gate_open_at_start[index] == true` 表示该阀门从上一控制周期边界无缝保持开启；
    /// 此时当前非零时长只是同一次连续点火的延续，不再次套用最小脉冲。若阀门原本
    /// 关闭，任何非零请求仍提升到该喷口的最小脉冲时间。
    pub fn applied_gate_on_times_with_initial_gate_state(
        &self,
        spec: ApolloPropulsionSpec,
        timing: SimulationTiming,
        gate_open_at_start: [bool; RCS_THRUSTER_COUNT],
    ) -> Result<[u64; RCS_THRUSTER_COUNT], PropulsionValidationError> {
        spec.validate_for_timing(timing)?;
        let control_step_ns = control_step_u64(timing)?;
        let mut applied = [0; RCS_THRUSTER_COUNT];
        for (index, requested) in self.on_time_ns.iter().copied().enumerate() {
            if requested > control_step_ns {
                return Err(invalid(format!(
                    "RCS '{}' requests {requested} ns, longer than control step {control_step_ns} ns",
                    spec.rcs_thrusters[index].label
                )));
            }
            applied[index] = if requested == 0 {
                0
            } else if gate_open_at_start[index] {
                requested
            } else {
                requested.max(spec.rcs_thrusters[index].minimum_pulse_ns)
            };
        }
        Ok(applied)
    }
}

/// DPS 的离散工作区间。
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum DpsMode {
    Off,
    Variable,
    FullThrust,
}

/// DPS 请求。全推力档与可调段有意分开表达。
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum DpsCommand {
    #[default]
    Off,
    Variable {
        thrust_n: f64,
        gimbal_x_rad: f64,
        gimbal_z_rad: f64,
    },
    FullThrust {
        gimbal_x_rad: f64,
        gimbal_z_rad: f64,
    },
}

impl DpsCommand {
    pub const fn mode(self) -> DpsMode {
        match self {
            Self::Off => DpsMode::Off,
            Self::Variable { .. } => DpsMode::Variable,
            Self::FullThrust { .. } => DpsMode::FullThrust,
        }
    }

    pub fn validate(self) -> Result<(), PropulsionValidationError> {
        match self {
            Self::Off => Ok(()),
            Self::Variable {
                thrust_n,
                gimbal_x_rad,
                gimbal_z_rad,
            } => {
                if !thrust_n.is_finite() || thrust_n <= 0.0 {
                    return Err(invalid("DPS variable thrust must be finite and positive"));
                }
                validate_gimbal(gimbal_x_rad, gimbal_z_rad)
            }
            Self::FullThrust {
                gimbal_x_rad,
                gimbal_z_rad,
            } => validate_gimbal(gimbal_x_rad, gimbal_z_rad),
        }
    }
}

/// 单个 RCS 喷口在一个控制周期中的实际结果。
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AppliedRcsThruster {
    pub applied_gate_on_time_ns: u64,
    pub mean_thrust_n: f64,
}

/// DPS 在一个控制周期中的实际结果。
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AppliedDps {
    pub mode: DpsMode,
    pub thrust_n: f64,
    pub gimbal_x_rad: f64,
    pub gimbal_z_rad: f64,
    pub force_direction_body: DVec3,
}

impl DpsSpec {
    /// 检查 DPS 几何、推力区间和万向节限制是否能安全用于求解。
    pub fn validate(self) -> Result<(), PropulsionValidationError> {
        if !self.gimbal_pivot_body_m.is_finite()
            || !self.nominal_force_direction_body.is_finite()
            || (self.nominal_force_direction_body.length() - 1.0).abs() > 1.0e-12
            || !self.variable_min_thrust_n.is_finite()
            || !self.variable_max_thrust_n.is_finite()
            || !self.full_thrust_n.is_finite()
            || !self.maximum_gimbal_rad.is_finite()
            || !self.gimbal_rate_rad_s.is_finite()
            || self.variable_min_thrust_n <= 0.0
            || self.variable_max_thrust_n <= self.variable_min_thrust_n
            || self.full_thrust_n <= self.variable_max_thrust_n
            || self.maximum_gimbal_rad <= 0.0
            || self.gimbal_rate_rad_s <= 0.0
        {
            return Err(invalid("DPS specification is invalid"));
        }
        Ok(())
    }

    pub fn apply(self, command: DpsCommand) -> Result<AppliedDps, PropulsionValidationError> {
        self.validate()?;
        command.validate()?;
        let (mode, thrust_n, requested_x, requested_z) = match command {
            DpsCommand::Off => (DpsMode::Off, 0.0, 0.0, 0.0),
            DpsCommand::Variable {
                thrust_n,
                gimbal_x_rad,
                gimbal_z_rad,
            } => (
                DpsMode::Variable,
                thrust_n.clamp(self.variable_min_thrust_n, self.variable_max_thrust_n),
                gimbal_x_rad,
                gimbal_z_rad,
            ),
            DpsCommand::FullThrust {
                gimbal_x_rad,
                gimbal_z_rad,
            } => (
                DpsMode::FullThrust,
                self.full_thrust_n,
                gimbal_x_rad,
                gimbal_z_rad,
            ),
        };

        let (gimbal_x_rad, gimbal_z_rad) =
            clamp_gimbal_components(requested_x, requested_z, self.maximum_gimbal_rad);
        // gimbal_x/z 表示分别朝机体 +X/+Z 方向的倾斜量，而不是绕同名轴旋转。
        // 默认 Apollo 规格的名义方向为 +Y；这里仍围绕规格给定的名义方向
        // 构造正交切平面，使零摆角对自定义单位名义轴也保持一致。
        let nominal = self.nominal_force_direction_body;
        let x_reference = if nominal.dot(DVec3::X).abs() < 0.99 {
            DVec3::X
        } else {
            DVec3::Y
        };
        let x_tangent = (x_reference - nominal * nominal.dot(x_reference)).normalize();
        let z_tangent = x_tangent.cross(nominal).normalize();
        let force_direction_body =
            (nominal + x_tangent * gimbal_x_rad.tan() + z_tangent * gimbal_z_rad.tan()).normalize();

        Ok(AppliedDps {
            mode,
            thrust_n,
            gimbal_x_rad,
            gimbal_z_rad,
            force_direction_body,
        })
    }
}

/// 将二维摆角请求限制在圆形包络内，同时避免极大有限输入的模长溢出。
fn clamp_gimbal_components(x: f64, z: f64, maximum: f64) -> (f64, f64) {
    let largest_component = x.abs().max(z.abs());
    if largest_component == 0.0 {
        return (0.0, 0.0);
    }

    let normalized_x = x / largest_component;
    let normalized_z = z / largest_component;
    let normalized_magnitude = normalized_x.hypot(normalized_z);
    if largest_component <= maximum / normalized_magnitude {
        (x, z)
    } else {
        let clamped_scale = maximum / normalized_magnitude;
        (normalized_x * clamped_scale, normalized_z * clamped_scale)
    }
}

fn validate_gimbal(x: f64, z: f64) -> Result<(), PropulsionValidationError> {
    if x.is_finite() && z.is_finite() {
        Ok(())
    } else {
        Err(invalid("DPS gimbal components must be finite"))
    }
}

/// 一个控制周期的完整推进器请求。
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PropulsionCommand {
    pub rcs: RcsCommand,
    pub dps: DpsCommand,
}

impl PropulsionCommand {
    pub const OFF: Self = Self {
        rcs: RcsCommand::OFF,
        dps: DpsCommand::Off,
    };

    pub fn validate(self) -> Result<(), PropulsionValidationError> {
        self.dps.validate()
    }
}

/// 一个控制周期实际产生的推进器状态和平均 wrench。
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AppliedPropulsion {
    pub rcs: [AppliedRcsThruster; RCS_THRUSTER_COUNT],
    pub dps: AppliedDps,
    pub mean_wrench_body: BodyWrench,
}

/// `ApolloPropulsionPlant::step` 的完整结果。
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PropulsionStep {
    pub snapshot: PlantSnapshot,
    pub requested_command: PropulsionCommand,
    pub applied: AppliedPropulsion,
}

/// 推进规格或命令校验错误。
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PropulsionValidationError {
    message: String,
}

impl PropulsionValidationError {
    pub fn message(&self) -> &str {
        &self.message
    }
}

impl fmt::Display for PropulsionValidationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl Error for PropulsionValidationError {}

fn invalid(message: impl Into<String>) -> PropulsionValidationError {
    PropulsionValidationError {
        message: message.into(),
    }
}

fn control_step_u64(timing: SimulationTiming) -> Result<u64, PropulsionValidationError> {
    u64::try_from(timing.control_step_ns())
        .map_err(|_| invalid("control step does not fit an unsigned 64-bit nanosecond value"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apollo11_spec_has_historical_ids_quads_and_feed_split() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        spec.validate().unwrap();
        let labels: Vec<_> = spec
            .rcs_thrusters
            .iter()
            .map(|thruster| thruster.label)
            .collect();
        assert_eq!(
            labels,
            [
                "A1U", "B1D", "A1F", "B1L", "B2U", "A2D", "A2A", "B2L", "A3U", "B3D", "B3A", "A3R",
                "B4U", "A4D", "B4F", "A4R"
            ]
        );
    }

    #[test]
    fn historical_suffix_is_plume_direction_and_force_is_opposite() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let expected = [
            DVec3::NEG_Y,
            DVec3::Y,
            DVec3::NEG_Z,
            DVec3::X,
            DVec3::NEG_Y,
            DVec3::Y,
            DVec3::Z,
            DVec3::X,
            DVec3::NEG_Y,
            DVec3::Y,
            DVec3::Z,
            DVec3::NEG_X,
            DVec3::NEG_Y,
            DVec3::Y,
            DVec3::NEG_Z,
            DVec3::NEG_X,
        ];
        for (thruster, expected) in spec.rcs_thrusters.iter().zip(expected) {
            assert_eq!(
                thruster.force_direction_body, expected,
                "{}",
                thruster.label
            );
        }
    }

    #[test]
    fn rcs_id_deserialization_rejects_out_of_range_indices() {
        let valid: RcsThrusterId = serde_json::from_str("15").unwrap();
        assert_eq!(valid.index(), 15);
        assert_eq!(valid.label(), "A4R");
        assert!(serde_json::from_str::<RcsThrusterId>("16").is_err());
        assert!(serde_json::from_str::<RcsThrusterId>("255").is_err());
    }

    #[test]
    fn data_book_relative_station_offsets_are_preserved() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let q4 = &spec.rcs_thrusters[12..16];
        assert!((q4[0].position_body_m.y - 3.12192).abs() < 1.0e-12);
        assert!((q4[1].position_body_m.y - 2.86538).abs() < 1.0e-12);
        assert!((q4[2].position_body_m.x - 1.5621).abs() < 1.0e-12);
        assert!((q4[2].position_body_m.z - 1.68529).abs() < 1.0e-12);
        assert!((q4[3].position_body_m.x - 1.68529).abs() < 1.0e-12);
        assert!((q4[3].position_body_m.z - 1.5621).abs() < 1.0e-12);
    }

    #[test]
    fn pulse_floor_is_exact_nanoseconds_without_substep_quantization() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let command = RcsCommand::single_pulse(RcsThrusterId::new(0).unwrap(), 7_000_001)
            .with_on_time(RcsThrusterId::new(1).unwrap(), 15_000_001);
        let applied = command
            .applied_gate_on_times(spec, SimulationTiming::APOLLO)
            .unwrap();
        assert_eq!(applied[0], 14_000_000);
        assert_eq!(applied[1], 15_000_001);
    }

    #[test]
    fn minimum_pulse_is_not_reapplied_to_an_already_open_gate() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let command = RcsCommand::single_pulse(RcsThrusterId::new(0).unwrap(), 1_000_000)
            .with_on_time(RcsThrusterId::new(1).unwrap(), 1_000_000);
        let mut initially_open = [false; RCS_THRUSTER_COUNT];
        initially_open[0] = true;

        let applied = command
            .applied_gate_on_times_with_initial_gate_state(
                spec,
                SimulationTiming::APOLLO,
                initially_open,
            )
            .unwrap();

        assert_eq!(applied[0], 1_000_000);
        assert_eq!(applied[1], RCS_MINIMUM_PULSE_NS);
    }

    #[test]
    fn overlong_pulse_is_rejected() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let command = RcsCommand::single_pulse(RcsThrusterId::new(0).unwrap(), 20_000_001);
        assert!(
            command
                .applied_gate_on_times(spec, SimulationTiming::APOLLO)
                .is_err()
        );
    }

    #[test]
    fn dps_has_separate_variable_and_full_thrust_modes() {
        let dps = ApolloPropulsionSpec::apollo11_touchdown().dps;
        assert_eq!(dps.gimbal_rate_rad_s, DPS_GIMBAL_RATE_RAD_S);
        assert!((dps.gimbal_rate_rad_s.to_degrees() - 0.2).abs() < 1.0e-15);
        let variable = dps
            .apply(DpsCommand::Variable {
                thrust_n: 1.0e9,
                gimbal_x_rad: 0.0,
                gimbal_z_rad: 0.0,
            })
            .unwrap();
        assert_eq!(variable.mode, DpsMode::Variable);
        assert_eq!(variable.thrust_n, DPS_VARIABLE_MAX_THRUST_N);

        let full = dps
            .apply(DpsCommand::FullThrust {
                gimbal_x_rad: 0.0,
                gimbal_z_rad: 0.0,
            })
            .unwrap();
        assert_eq!(full.mode, DpsMode::FullThrust);
        assert_eq!(full.thrust_n, DPS_FULL_THRUST_N);
    }

    #[test]
    fn dps_gimbal_is_limited_to_circular_six_degree_cone() {
        let dps = ApolloPropulsionSpec::apollo11_touchdown().dps;
        let applied = dps
            .apply(DpsCommand::FullThrust {
                gimbal_x_rad: 10.0_f64.to_radians(),
                gimbal_z_rad: 10.0_f64.to_radians(),
            })
            .unwrap();
        assert!(
            (applied.gimbal_x_rad.hypot(applied.gimbal_z_rad) - DPS_MAXIMUM_GIMBAL_RAD).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn dps_gimbal_clamp_preserves_extreme_finite_request_direction() {
        let dps = ApolloPropulsionSpec::apollo11_touchdown().dps;
        let applied = dps
            .apply(DpsCommand::FullThrust {
                gimbal_x_rad: f64::MAX,
                gimbal_z_rad: -f64::MAX / 2.0,
            })
            .unwrap();

        assert!(applied.gimbal_x_rad.is_finite());
        assert!(applied.gimbal_z_rad.is_finite());
        assert!(applied.gimbal_x_rad > 0.0);
        assert!(applied.gimbal_z_rad < 0.0);
        assert!(
            (applied.gimbal_x_rad.hypot(applied.gimbal_z_rad) - DPS_MAXIMUM_GIMBAL_RAD).abs()
                < 1.0e-12
        );
        assert!((applied.gimbal_x_rad / -applied.gimbal_z_rad - 2.0).abs() < 1.0e-12);
        assert!(applied.force_direction_body.is_finite());
    }

    #[test]
    fn dps_apply_validates_spec_before_command() {
        let mut dps = ApolloPropulsionSpec::apollo11_touchdown().dps;
        dps.nominal_force_direction_body = DVec3::ZERO;

        let error = dps
            .apply(DpsCommand::FullThrust {
                gimbal_x_rad: f64::NAN,
                gimbal_z_rad: 0.0,
            })
            .unwrap_err();

        assert_eq!(error.message(), "DPS specification is invalid");
    }

    #[test]
    fn dps_apply_rejects_invalid_thrust_ranges() {
        let valid = ApolloPropulsionSpec::apollo11_touchdown().dps;
        let invalid_specs = [
            DpsSpec {
                variable_min_thrust_n: 0.0,
                ..valid
            },
            DpsSpec {
                variable_max_thrust_n: valid.variable_min_thrust_n,
                ..valid
            },
            DpsSpec {
                full_thrust_n: valid.variable_max_thrust_n,
                ..valid
            },
        ];

        for dps in invalid_specs {
            assert!(dps.validate().is_err());
            assert!(dps.apply(DpsCommand::Off).is_err());
        }
    }

    #[test]
    fn dps_rejects_invalid_gimbal_rate() {
        let valid = ApolloPropulsionSpec::apollo11_touchdown().dps;
        for gimbal_rate_rad_s in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let dps = DpsSpec {
                gimbal_rate_rad_s,
                ..valid
            };
            assert!(dps.validate().is_err());
            assert!(dps.apply(DpsCommand::Off).is_err());
        }
    }

    #[test]
    fn zero_dps_gimbal_preserves_the_specified_nominal_direction() {
        let mut dps = ApolloPropulsionSpec::apollo11_touchdown().dps;
        dps.nominal_force_direction_body = DVec3::Z;
        let applied = dps
            .apply(DpsCommand::FullThrust {
                gimbal_x_rad: 0.0,
                gimbal_z_rad: 0.0,
            })
            .unwrap();
        assert!(applied.force_direction_body.distance(DVec3::Z) < 1.0e-12);
    }
}

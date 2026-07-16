use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fmt;

/// 单位四元数的长度误差上限。
pub const UNIT_QUATERNION_NORM_TOLERANCE: f64 = 1.0e-9;

/// Apollo 单刚体的完整状态。
///
/// 字段名显式标注坐标系和单位，避免后端或语言绑定隐式猜测。
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ApolloState {
    /// 机体系原点在世界系中的位置，米。
    ///
    /// 这不是质心位置。Apollo 规格的质心相对机体系原点有非零偏移。
    pub position_body_origin_world_m: DVec3,
    /// 从机体系旋转到世界系的单位四元数。
    ///
    /// 持久 JSON 中字段名为 `quaternion_body_to_world_wxyz`，并固定使用
    /// `w, x, y, z` 顺序，不继承具体数学库的内部序列化顺序。
    #[serde(rename = "quaternion_body_to_world_wxyz", with = "dquat_wxyz")]
    pub body_to_world: DQuat,
    /// 机体系原点在世界系中的线速度，米每秒。
    pub linear_velocity_body_origin_world_mps: DVec3,
    /// 机体系角速度，弧度每秒。
    pub angular_velocity_body_radps: DVec3,
}

impl ApolloState {
    /// 静止于原点、姿态为单位旋转的状态。
    pub const ZERO: Self = Self {
        position_body_origin_world_m: DVec3::ZERO,
        body_to_world: DQuat::IDENTITY,
        linear_velocity_body_origin_world_mps: DVec3::ZERO,
        angular_velocity_body_radps: DVec3::ZERO,
    };

    /// 检查所有状态分量有限，且姿态是单位四元数。
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_finite_vec3(
            self.position_body_origin_world_m,
            "position_body_origin_world_m",
        )?;
        validate_unit_quaternion(self.body_to_world, "body_to_world")?;
        validate_finite_vec3(
            self.linear_velocity_body_origin_world_mps,
            "linear_velocity_body_origin_world_mps",
        )?;
        validate_finite_vec3(
            self.angular_velocity_body_radps,
            "angular_velocity_body_radps",
        )?;
        Ok(())
    }

    /// 对有限且非退化的姿态四元数归一化。
    ///
    /// 这是显式的输入修复操作；[`validate`](Self::validate) 本身不会静默改变状态。
    pub fn with_normalized_attitude(mut self) -> Result<Self, ValidationError> {
        self.body_to_world = normalized_quaternion(self.body_to_world, "body_to_world")?;
        Ok(self)
    }
}

impl Default for ApolloState {
    fn default() -> Self {
        Self::ZERO
    }
}

/// 在机体坐标系中表达的六维力/力矩动作。
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BodyWrench {
    /// 在机体系中表达、等效作用于质心的力，牛顿。
    pub force_body_n: DVec3,
    /// 关于质心、在机体系中表达的力矩，牛顿米。
    pub torque_about_com_body_nm: DVec3,
}

impl BodyWrench {
    /// 零力、零力矩动作。
    pub const ZERO: Self = Self {
        force_body_n: DVec3::ZERO,
        torque_about_com_body_nm: DVec3::ZERO,
    };

    /// 检查力和力矩的所有分量都是有限数。
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_finite_vec3(self.force_body_n, "force_body_n")?;
        validate_finite_vec3(self.torque_about_com_body_nm, "torque_about_com_body_nm")?;
        Ok(())
    }
}

/// 固定持久格式中的四元数顺序，避免 `glam` 的 xyzw serde 细节泄漏到 API。
pub(crate) mod dquat_wxyz {
    use glam::DQuat;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S>(value: &DQuat, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        [value.w, value.x, value.y, value.z].serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<DQuat, D::Error>
    where
        D: Deserializer<'de>,
    {
        let [w, x, y, z] = <[f64; 4]>::deserialize(deserializer)?;
        Ok(DQuat::from_xyzw(x, y, z, w))
    }
}

/// 状态或动作的数值校验错误。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ValidationError {
    /// 指定字段包含 NaN 或无穷大。
    NonFinite {
        /// 失败字段名。
        field: &'static str,
    },
    /// 四元数长度过小，无法表示旋转。
    DegenerateQuaternion {
        /// 失败字段名。
        field: &'static str,
        /// 实际长度平方。
        norm_squared: f64,
    },
    /// 四元数有限且非退化，但不在单位长度容差内。
    NonUnitQuaternion {
        /// 失败字段名。
        field: &'static str,
        /// 实际长度。
        norm: f64,
        /// 允许的长度误差。
        tolerance: f64,
    },
}

impl fmt::Display for ValidationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite { field } => write!(formatter, "{field} contains a non-finite value"),
            Self::DegenerateQuaternion {
                field,
                norm_squared,
            } => write!(
                formatter,
                "{field} is a degenerate quaternion (norm squared: {norm_squared})"
            ),
            Self::NonUnitQuaternion {
                field,
                norm,
                tolerance,
            } => write!(
                formatter,
                "{field} must be a unit quaternion (norm: {norm}, tolerance: {tolerance})"
            ),
        }
    }
}

impl Error for ValidationError {}

/// 校验三维向量的所有分量为有限数。
pub fn validate_finite_vec3(value: DVec3, field: &'static str) -> Result<(), ValidationError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(ValidationError::NonFinite { field })
    }
}

/// 校验四元数的所有分量为有限数。
pub fn validate_finite_quaternion(
    value: DQuat,
    field: &'static str,
) -> Result<(), ValidationError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(ValidationError::NonFinite { field })
    }
}

/// 校验四元数有限、非退化且为单位长度。
pub fn validate_unit_quaternion(value: DQuat, field: &'static str) -> Result<(), ValidationError> {
    validate_finite_quaternion(value, field)?;
    let norm_squared = value.length_squared();
    if norm_squared <= f64::EPSILON {
        return Err(ValidationError::DegenerateQuaternion {
            field,
            norm_squared,
        });
    }

    let norm = norm_squared.sqrt();
    if (norm - 1.0).abs() > UNIT_QUATERNION_NORM_TOLERANCE {
        return Err(ValidationError::NonUnitQuaternion {
            field,
            norm,
            tolerance: UNIT_QUATERNION_NORM_TOLERANCE,
        });
    }
    Ok(())
}

/// 归一化有限且非退化的四元数。
pub fn normalized_quaternion(value: DQuat, field: &'static str) -> Result<DQuat, ValidationError> {
    validate_finite_quaternion(value, field)?;
    let norm_squared = value.length_squared();
    if norm_squared <= f64::EPSILON {
        return Err(ValidationError::DegenerateQuaternion {
            field,
            norm_squared,
        });
    }
    Ok(value / norm_squared.sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_state_and_zero_wrench_are_valid() {
        ApolloState::default().validate().unwrap();
        BodyWrench::default().validate().unwrap();
    }

    #[test]
    fn state_rejects_non_finite_components() {
        let state = ApolloState {
            position_body_origin_world_m: DVec3::new(f64::NAN, 0.0, 0.0),
            ..ApolloState::ZERO
        };
        assert_eq!(
            state.validate(),
            Err(ValidationError::NonFinite {
                field: "position_body_origin_world_m"
            })
        );
    }

    #[test]
    fn state_rejects_degenerate_and_non_unit_attitudes() {
        let zero_attitude = ApolloState {
            body_to_world: DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            ..ApolloState::ZERO
        };
        assert!(matches!(
            zero_attitude.validate(),
            Err(ValidationError::DegenerateQuaternion { .. })
        ));

        let scaled_attitude = ApolloState {
            body_to_world: DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0),
            ..ApolloState::ZERO
        };
        assert!(matches!(
            scaled_attitude.validate(),
            Err(ValidationError::NonUnitQuaternion { .. })
        ));
    }

    #[test]
    fn attitude_normalization_is_explicit_and_checked() {
        let state = ApolloState {
            body_to_world: DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0),
            ..ApolloState::ZERO
        }
        .with_normalized_attitude()
        .unwrap();
        state.validate().unwrap();
        assert_eq!(state.body_to_world, DQuat::IDENTITY);
    }

    #[test]
    fn wrench_rejects_infinite_values() {
        let wrench = BodyWrench {
            torque_about_com_body_nm: DVec3::new(0.0, f64::INFINITY, 0.0),
            ..BodyWrench::ZERO
        };
        assert_eq!(
            wrench.validate(),
            Err(ValidationError::NonFinite {
                field: "torque_about_com_body_nm"
            })
        );
    }

    #[test]
    fn persistent_state_schema_uses_explicit_wxyz_quaternion_order() {
        let state = ApolloState {
            body_to_world: DQuat::from_xyzw(0.1, 0.2, 0.3, 0.9).normalize(),
            ..ApolloState::ZERO
        };

        let json = serde_json::to_value(state).unwrap();
        assert!(json.get("body_to_world").is_none());
        assert_eq!(
            json["quaternion_body_to_world_wxyz"],
            serde_json::json!([
                state.body_to_world.w,
                state.body_to_world.x,
                state.body_to_world.y,
                state.body_to_world.z,
            ])
        );
        assert_eq!(serde_json::from_value::<ApolloState>(json).unwrap(), state);
    }
}

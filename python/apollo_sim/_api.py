"""Python 领域对象与 Rust 原生 plant 的薄适配层。"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Sequence, TextIO

import numpy as np
from numpy.typing import ArrayLike, NDArray

try:
    from . import _apollo_native as _native
except ImportError as exc:  # 允许未编译时单独使用数据类型。
    _native = None
    _native_import_failure: ImportError | None = exc
else:
    _native_import_failure = None


FloatVector = NDArray[np.float64]
UInt64Vector = NDArray[np.uint64]
_QUATERNION_NORM_TOLERANCE = 1.0e-9
_RCS_THRUSTER_COUNT = 16


def _readonly_vector(name: str, value: ArrayLike, length: int) -> FloatVector:
    """转换并校验一个具名向量，不保留调用方数组的可变引用。"""

    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain real numbers") from exc

    if array.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")

    owned = np.array(array, dtype=np.float64, copy=True)
    owned.setflags(write=False)
    return owned


def _readonly_uint64_vector(
    name: str, value: ArrayLike, length: int
) -> UInt64Vector:
    """校验无符号整数向量，避免 NumPy 静默截断浮点数或回绕负数。"""

    try:
        array = np.asarray(value, dtype=object)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain integers") from exc

    if array.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {array.shape}")

    converted: list[int] = []
    for item in array:
        if isinstance(item, (bool, np.bool_)) or not isinstance(
            item, (int, np.integer)
        ):
            raise ValueError(f"{name} must contain integers")
        integer = int(item)
        if integer < 0 or integer > np.iinfo(np.uint64).max:
            raise ValueError(
                f"{name} values must fit an unsigned 64-bit integer"
            )
        converted.append(integer)

    owned = np.array(converted, dtype=np.uint64)
    owned.setflags(write=False)
    return owned


def _finite_float(name: str, value: float) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be a real number")
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a real number") from exc
    if not np.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _non_negative_integer(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"{name} must be an integer")
    converted = int(value)
    if converted < 0:
        raise ValueError(f"{name} must be non-negative")
    if converted > np.iinfo(np.uint64).max:
        raise ValueError(f"{name} must fit an unsigned 64-bit integer")
    return converted


@dataclass(frozen=True, slots=True, eq=False)
class ApolloState:
    """Apollo 刚体状态；字段名显式标明坐标系与单位。"""

    position_body_origin_world_m: FloatVector
    quaternion_body_to_world_wxyz: FloatVector
    linear_velocity_body_origin_world_mps: FloatVector
    angular_velocity_body_radps: FloatVector

    def __post_init__(self) -> None:
        position = _readonly_vector(
            "position_body_origin_world_m", self.position_body_origin_world_m, 3
        )
        quaternion = _readonly_vector(
            "quaternion_body_to_world_wxyz",
            self.quaternion_body_to_world_wxyz,
            4,
        )
        linear_velocity = _readonly_vector(
            "linear_velocity_body_origin_world_mps",
            self.linear_velocity_body_origin_world_mps,
            3,
        )
        angular_velocity = _readonly_vector(
            "angular_velocity_body_radps", self.angular_velocity_body_radps, 3
        )

        norm = float(np.linalg.norm(quaternion))
        if abs(norm - 1.0) > _QUATERNION_NORM_TOLERANCE:
            raise ValueError(
                "quaternion_body_to_world_wxyz must be a unit quaternion "
                f"(norm={norm:.16g})"
            )

        object.__setattr__(self, "position_body_origin_world_m", position)
        object.__setattr__(self, "quaternion_body_to_world_wxyz", quaternion)
        object.__setattr__(
            self, "linear_velocity_body_origin_world_mps", linear_velocity
        )
        object.__setattr__(self, "angular_velocity_body_radps", angular_velocity)

    @classmethod
    def identity(cls) -> ApolloState:
        """返回原点、单位姿态、零速度状态。"""

        return cls(
            position_body_origin_world_m=np.zeros(3, dtype=np.float64),
            quaternion_body_to_world_wxyz=np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float64
            ),
            linear_velocity_body_origin_world_mps=np.zeros(3, dtype=np.float64),
            angular_velocity_body_radps=np.zeros(3, dtype=np.float64),
        )

    @classmethod
    def from_vector(cls, value: ArrayLike) -> ApolloState:
        """按文档规定的 13 维顺序构造状态。"""

        vector = _readonly_vector("state", value, 13)
        return cls(
            position_body_origin_world_m=vector[0:3],
            quaternion_body_to_world_wxyz=vector[3:7],
            linear_velocity_body_origin_world_mps=vector[7:10],
            angular_velocity_body_radps=vector[10:13],
        )

    def as_vector(self) -> FloatVector:
        """返回 `[p_body_origin_world, q_body_to_world(wxyz), v_body_origin_world, omega_body]`。"""

        vector = np.concatenate(
            (
                self.position_body_origin_world_m,
                self.quaternion_body_to_world_wxyz,
                self.linear_velocity_body_origin_world_mps,
                self.angular_velocity_body_radps,
            )
        )
        vector.setflags(write=False)
        return vector

    def __eq__(self, other: object) -> bool:
        return isinstance(other, ApolloState) and bool(
            np.array_equal(self.as_vector(), other.as_vector())
        )


@dataclass(frozen=True, slots=True, eq=False)
class BodyWrench:
    """在机体系表达的合力与合力矩。"""

    force_body_n: FloatVector
    torque_about_com_body_nm: FloatVector

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "force_body_n", _readonly_vector("force_body_n", self.force_body_n, 3)
        )
        object.__setattr__(
            self,
            "torque_about_com_body_nm",
            _readonly_vector(
                "torque_about_com_body_nm", self.torque_about_com_body_nm, 3
            ),
        )

    @classmethod
    def zero(cls) -> BodyWrench:
        return cls(
            force_body_n=np.zeros(3, dtype=np.float64),
            torque_about_com_body_nm=np.zeros(3, dtype=np.float64),
        )

    @classmethod
    def from_vector(cls, value: ArrayLike) -> BodyWrench:
        vector = _readonly_vector("wrench", value, 6)
        return cls(force_body_n=vector[0:3], torque_about_com_body_nm=vector[3:6])

    def as_vector(self) -> FloatVector:
        """返回 `[force_body_n, torque_about_com_body_nm]`。"""

        vector = np.concatenate((self.force_body_n, self.torque_about_com_body_nm))
        vector.setflags(write=False)
        return vector

    def __eq__(self, other: object) -> bool:
        return isinstance(other, BodyWrench) and bool(
            np.array_equal(self.as_vector(), other.as_vector())
        )


class RcsThrusterId(IntEnum):
    """Apollo LM Operations Handbook 的稳定 RCS 喷口标签与数组索引。"""

    A1U = 0
    B1D = 1
    A1F = 2
    B1L = 3
    B2U = 4
    A2D = 5
    A2A = 6
    B2L = 7
    A3U = 8
    B3D = 9
    B3A = 10
    A3R = 11
    B4U = 12
    A4D = 13
    B4F = 14
    A4R = 15


RCS_THRUSTER_ORDER: tuple[RcsThrusterId, ...] = tuple(RcsThrusterId)


class RcsQuad(IntEnum):
    """四个 RCS quad 的稳定编号。"""

    QUAD_1 = 1
    QUAD_2 = 2
    QUAD_3 = 3
    QUAD_4 = 4


class RcsFeedSystem(str, Enum):
    """喷口所属的两套独立推进剂供给系统。"""

    A = "A"
    B = "B"


@dataclass(frozen=True, slots=True, eq=False)
class RcsThrusterSpec:
    """单个 RCS 喷口的后端中立几何、分组和执行器规格。"""

    id: RcsThrusterId
    label: str
    quad: RcsQuad
    feed_system: RcsFeedSystem
    position_body_m: FloatVector
    force_direction_body: FloatVector
    steady_thrust_n: float
    minimum_pulse_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.id, RcsThrusterId):
            raise ValueError("id must be an RcsThrusterId")
        if not isinstance(self.label, str) or not self.label:
            raise ValueError("label must be a non-empty string")
        if not isinstance(self.quad, RcsQuad):
            raise ValueError("quad must be an RcsQuad")
        if not isinstance(self.feed_system, RcsFeedSystem):
            raise ValueError("feed_system must be an RcsFeedSystem")

        position = _readonly_vector(
            "position_body_m", self.position_body_m, 3
        )
        direction = _readonly_vector(
            "force_direction_body", self.force_direction_body, 3
        )
        direction_norm = float(np.linalg.norm(direction))
        if abs(direction_norm - 1.0) > _QUATERNION_NORM_TOLERANCE:
            raise ValueError("force_direction_body must be a unit vector")
        thrust_n = _finite_float("steady_thrust_n", self.steady_thrust_n)
        if thrust_n <= 0.0:
            raise ValueError("steady_thrust_n must be positive")
        minimum_pulse_ns = _non_negative_integer(
            "minimum_pulse_ns", self.minimum_pulse_ns
        )
        if minimum_pulse_ns == 0:
            raise ValueError("minimum_pulse_ns must be positive")
        object.__setattr__(self, "position_body_m", position)
        object.__setattr__(self, "force_direction_body", direction)
        object.__setattr__(self, "steady_thrust_n", thrust_n)
        object.__setattr__(self, "minimum_pulse_ns", minimum_pulse_ns)

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, RcsThrusterSpec)
            and self.id is other.id
            and self.label == other.label
            and self.quad is other.quad
            and self.feed_system is other.feed_system
            and bool(np.array_equal(self.position_body_m, other.position_body_m))
            and bool(
                np.array_equal(
                    self.force_direction_body, other.force_direction_body
                )
            )
            and self.steady_thrust_n == other.steady_thrust_n
            and self.minimum_pulse_ns == other.minimum_pulse_ns
        )


@dataclass(frozen=True, slots=True, eq=False)
class RcsCommand:
    """16 路 RCS 阀门请求时长，均从当前控制周期起点计时。"""

    on_time_ns: UInt64Vector

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "on_time_ns",
            _readonly_uint64_vector(
                "on_time_ns", self.on_time_ns, _RCS_THRUSTER_COUNT
            ),
        )

    @classmethod
    def all_off(cls) -> RcsCommand:
        """返回所有 RCS 喷口关闭的命令。"""

        return cls(np.zeros(_RCS_THRUSTER_COUNT, dtype=np.uint64))

    @classmethod
    def single_thruster(cls, thruster_index: int, on_time_ns: int) -> RcsCommand:
        """只请求一个喷口；索引范围固定为 0..15。"""

        index = _non_negative_integer("thruster_index", thruster_index)
        if index >= _RCS_THRUSTER_COUNT:
            raise ValueError(
                f"thruster_index must be in [0, {_RCS_THRUSTER_COUNT - 1}]"
            )
        duration = _non_negative_integer("on_time_ns", on_time_ns)
        values = np.zeros(_RCS_THRUSTER_COUNT, dtype=np.uint64)
        values[index] = duration
        return cls(values)

    @classmethod
    def single_pulse(
        cls, thruster: RcsThrusterId, on_time_ns: int
    ) -> RcsCommand:
        """按稳定 Apollo 喷口标签请求单路脉冲。"""

        if not isinstance(thruster, RcsThrusterId):
            raise ValueError("thruster must be an RcsThrusterId")
        return cls.single_thruster(int(thruster), on_time_ns)

    def as_vector(self) -> UInt64Vector:
        """返回固定喷口顺序的只读 ``numpy.uint64`` 向量。"""

        copied = np.array(self.on_time_ns, dtype=np.uint64, copy=True)
        copied.setflags(write=False)
        return copied

    def __eq__(self, other: object) -> bool:
        return isinstance(other, RcsCommand) and bool(
            np.array_equal(self.on_time_ns, other.on_time_ns)
        )


class DpsMode(str, Enum):
    """Apollo 下降级主发动机的离散工作方式。"""

    OFF = "off"
    VARIABLE = "variable"
    FULL_THRUST = "full_thrust"


@dataclass(frozen=True, slots=True)
class DpsCommand:
    """下降推进系统命令；力和两个摆角均使用显式 SI 单位。"""

    mode: DpsMode
    thrust_n: float | None = None
    gimbal_x_rad: float = 0.0
    gimbal_z_rad: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.mode, DpsMode):
            raise ValueError("mode must be a DpsMode")

        gimbal_x_rad = _finite_float("gimbal_x_rad", self.gimbal_x_rad)
        gimbal_z_rad = _finite_float("gimbal_z_rad", self.gimbal_z_rad)
        object.__setattr__(self, "gimbal_x_rad", gimbal_x_rad)
        object.__setattr__(self, "gimbal_z_rad", gimbal_z_rad)

        if self.mode is DpsMode.VARIABLE:
            if self.thrust_n is None:
                raise ValueError("VARIABLE mode requires thrust_n")
            thrust_n = _finite_float("thrust_n", self.thrust_n)
            if thrust_n <= 0.0:
                raise ValueError("thrust_n must be positive in VARIABLE mode")
            object.__setattr__(self, "thrust_n", thrust_n)
        elif self.thrust_n is not None:
            raise ValueError("thrust_n is only valid in VARIABLE mode")

        if self.mode is DpsMode.OFF and (
            gimbal_x_rad != 0.0 or gimbal_z_rad != 0.0
        ):
            raise ValueError("OFF mode requires zero gimbal angles")

    @classmethod
    def off(cls) -> DpsCommand:
        return cls(mode=DpsMode.OFF)

    @classmethod
    def variable(
        cls,
        thrust_n: float,
        *,
        gimbal_x_rad: float = 0.0,
        gimbal_z_rad: float = 0.0,
    ) -> DpsCommand:
        return cls(
            mode=DpsMode.VARIABLE,
            thrust_n=thrust_n,
            gimbal_x_rad=gimbal_x_rad,
            gimbal_z_rad=gimbal_z_rad,
        )

    @classmethod
    def full_thrust(
        cls, *, gimbal_x_rad: float = 0.0, gimbal_z_rad: float = 0.0
    ) -> DpsCommand:
        return cls(
            mode=DpsMode.FULL_THRUST,
            gimbal_x_rad=gimbal_x_rad,
            gimbal_z_rad=gimbal_z_rad,
        )


@dataclass(frozen=True, slots=True, eq=False)
class DpsSpec:
    """Apollo 11 LM-5 下降推进系统的稳态推力与万向节规格。"""

    gimbal_pivot_body_m: FloatVector
    nominal_force_direction_body: FloatVector
    variable_min_thrust_n: float
    variable_max_thrust_n: float
    full_thrust_n: float
    maximum_gimbal_rad: float
    gimbal_rate_rad_s: float

    def __post_init__(self) -> None:
        pivot = _readonly_vector(
            "gimbal_pivot_body_m", self.gimbal_pivot_body_m, 3
        )
        direction = _readonly_vector(
            "nominal_force_direction_body",
            self.nominal_force_direction_body,
            3,
        )
        if (
            abs(float(np.linalg.norm(direction)) - 1.0)
            > _QUATERNION_NORM_TOLERANCE
        ):
            raise ValueError("nominal_force_direction_body must be a unit vector")

        variable_min = _finite_float(
            "variable_min_thrust_n", self.variable_min_thrust_n
        )
        variable_max = _finite_float(
            "variable_max_thrust_n", self.variable_max_thrust_n
        )
        full_thrust = _finite_float("full_thrust_n", self.full_thrust_n)
        maximum_gimbal = _finite_float(
            "maximum_gimbal_rad", self.maximum_gimbal_rad
        )
        gimbal_rate = _finite_float(
            "gimbal_rate_rad_s", self.gimbal_rate_rad_s
        )
        if not (0.0 < variable_min < variable_max < full_thrust):
            raise ValueError(
                "DPS thrusts must satisfy 0 < variable_min < variable_max "
                "< full_thrust"
            )
        if maximum_gimbal <= 0.0:
            raise ValueError("maximum_gimbal_rad must be positive")
        if gimbal_rate <= 0.0:
            raise ValueError("gimbal_rate_rad_s must be positive")
        object.__setattr__(self, "gimbal_pivot_body_m", pivot)
        object.__setattr__(self, "nominal_force_direction_body", direction)
        object.__setattr__(self, "variable_min_thrust_n", variable_min)
        object.__setattr__(self, "variable_max_thrust_n", variable_max)
        object.__setattr__(self, "full_thrust_n", full_thrust)
        object.__setattr__(self, "maximum_gimbal_rad", maximum_gimbal)
        object.__setattr__(self, "gimbal_rate_rad_s", gimbal_rate)

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, DpsSpec)
            and bool(
                np.array_equal(
                    self.gimbal_pivot_body_m, other.gimbal_pivot_body_m
                )
            )
            and bool(
                np.array_equal(
                    self.nominal_force_direction_body,
                    other.nominal_force_direction_body,
                )
            )
            and self.variable_min_thrust_n == other.variable_min_thrust_n
            and self.variable_max_thrust_n == other.variable_max_thrust_n
            and self.full_thrust_n == other.full_thrust_n
            and self.maximum_gimbal_rad == other.maximum_gimbal_rad
            and self.gimbal_rate_rad_s == other.gimbal_rate_rad_s
        )


@dataclass(frozen=True, slots=True)
class ApolloPropulsionSpec:
    """RCS 16 路稳定顺序与 DPS 规格的只读集合。"""

    rcs_thrusters: tuple[RcsThrusterSpec, ...]
    dps: DpsSpec

    def __post_init__(self) -> None:
        thrusters = tuple(self.rcs_thrusters)
        if len(thrusters) != _RCS_THRUSTER_COUNT:
            raise ValueError(
                f"rcs_thrusters must contain {_RCS_THRUSTER_COUNT} entries"
            )
        if not all(isinstance(spec, RcsThrusterSpec) for spec in thrusters):
            raise ValueError("rcs_thrusters must contain RcsThrusterSpec values")
        ids = tuple(spec.id for spec in thrusters)
        if ids != RCS_THRUSTER_ORDER:
            raise ValueError("rcs_thrusters must follow RCS_THRUSTER_ORDER")
        if not isinstance(self.dps, DpsSpec):
            raise ValueError("dps must be a DpsSpec")
        object.__setattr__(self, "rcs_thrusters", thrusters)


@dataclass(frozen=True, slots=True)
class PropulsionCommand:
    """一个控制周期内的 RCS 与下降推进系统联合命令。"""

    rcs: RcsCommand
    dps: DpsCommand

    def __post_init__(self) -> None:
        if not isinstance(self.rcs, RcsCommand):
            raise ValueError("rcs must be an RcsCommand")
        if not isinstance(self.dps, DpsCommand):
            raise ValueError("dps must be a DpsCommand")

    @classmethod
    def all_off(cls) -> PropulsionCommand:
        return cls(rcs=RcsCommand.all_off(), dps=DpsCommand.off())


@dataclass(frozen=True, slots=True, eq=False)
class AppliedRcs:
    """RCS 执行器实际门控时长和控制周期平均推力。"""

    applied_gate_on_time_ns: UInt64Vector
    mean_thrust_n: FloatVector

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "applied_gate_on_time_ns",
            _readonly_uint64_vector(
                "applied_gate_on_time_ns",
                self.applied_gate_on_time_ns,
                _RCS_THRUSTER_COUNT,
            ),
        )
        mean_thrust_n = _readonly_vector(
            "mean_thrust_n", self.mean_thrust_n, _RCS_THRUSTER_COUNT
        )
        if np.any(mean_thrust_n < 0.0):
            raise ValueError("mean_thrust_n must be non-negative")
        object.__setattr__(self, "mean_thrust_n", mean_thrust_n)

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, AppliedRcs)
            and bool(
                np.array_equal(
                    self.applied_gate_on_time_ns,
                    other.applied_gate_on_time_ns,
                )
            )
            and bool(np.array_equal(self.mean_thrust_n, other.mean_thrust_n))
        )


@dataclass(frozen=True, slots=True, eq=False)
class AppliedDps:
    """DPS 在一个控制周期内实际采用的模式、推力、摆角和受力方向。"""

    mode: DpsMode
    thrust_n: float
    gimbal_x_rad: float
    gimbal_z_rad: float
    force_direction_body: FloatVector

    def __post_init__(self) -> None:
        if not isinstance(self.mode, DpsMode):
            raise ValueError("mode must be a DpsMode")
        thrust_n = _finite_float("thrust_n", self.thrust_n)
        if thrust_n < 0.0:
            raise ValueError("thrust_n must be non-negative")
        object.__setattr__(self, "thrust_n", thrust_n)
        object.__setattr__(
            self, "gimbal_x_rad", _finite_float("gimbal_x_rad", self.gimbal_x_rad)
        )
        object.__setattr__(
            self, "gimbal_z_rad", _finite_float("gimbal_z_rad", self.gimbal_z_rad)
        )
        direction = _readonly_vector(
            "force_direction_body", self.force_direction_body, 3
        )
        if abs(float(np.linalg.norm(direction)) - 1.0) > _QUATERNION_NORM_TOLERANCE:
            raise ValueError("force_direction_body must be a unit vector")
        object.__setattr__(self, "force_direction_body", direction)

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, AppliedDps)
            and self.mode is other.mode
            and self.thrust_n == other.thrust_n
            and self.gimbal_x_rad == other.gimbal_x_rad
            and self.gimbal_z_rad == other.gimbal_z_rad
            and bool(
                np.array_equal(
                    self.force_direction_body, other.force_direction_body
                )
            )
        )


@dataclass(frozen=True, slots=True)
class AppliedPropulsion:
    """执行器实际输出及其控制周期平均机体系 wrench。"""

    rcs: AppliedRcs
    dps: AppliedDps
    mean_wrench_body: BodyWrench

    def __post_init__(self) -> None:
        if not isinstance(self.rcs, AppliedRcs):
            raise ValueError("rcs must be an AppliedRcs")
        if not isinstance(self.dps, AppliedDps):
            raise ValueError("dps must be an AppliedDps")
        if not isinstance(self.mean_wrench_body, BodyWrench):
            raise ValueError("mean_wrench_body must be a BodyWrench")


@dataclass(frozen=True, slots=True)
class SimulationTiming:
    """固定 MuJoCo 小步与每个外部控制步包含的小步数。"""

    physics_step_seconds: float = 0.002
    substeps_per_control: int = 10
    _physics_step_ns: int = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if isinstance(self.physics_step_seconds, (bool, np.bool_)):
            raise ValueError("physics_step_seconds must be a real number")
        physics_step_seconds = float(self.physics_step_seconds)
        if not np.isfinite(physics_step_seconds) or physics_step_seconds <= 0.0:
            raise ValueError("physics_step_seconds must be finite and positive")
        if (
            isinstance(self.substeps_per_control, bool)
            or not isinstance(self.substeps_per_control, (int, np.integer))
            or int(self.substeps_per_control) <= 0
            or int(self.substeps_per_control) > np.iinfo(np.uint32).max
        ):
            raise ValueError("substeps_per_control must be a positive 32-bit integer")

        nanoseconds = physics_step_seconds * 1.0e9
        rounded_nanoseconds = round(nanoseconds)
        if (
            rounded_nanoseconds < 1
            or rounded_nanoseconds > np.iinfo(np.uint64).max
            or abs(nanoseconds - rounded_nanoseconds) > 1.0e-6
        ):
            raise ValueError(
                "physics_step_seconds must be representable as a positive integer "
                "number of nanoseconds"
            )

        # Rust 端以整数纳秒为权威值；在 Python 边界同步规范化，避免两个语言
        # 对同一个近似输入暴露略有不同的控制周期和轨迹时间。
        object.__setattr__(
            self, "physics_step_seconds", rounded_nanoseconds * 1.0e-9
        )
        object.__setattr__(self, "substeps_per_control", int(self.substeps_per_control))
        object.__setattr__(self, "_physics_step_ns", rounded_nanoseconds)

    @property
    def physics_step_ns(self) -> int:
        """与 Rust API 一致的权威整数纳秒物理步长。"""

        return self._physics_step_ns

    @property
    def control_step_ns(self) -> int:
        return self._physics_step_ns * self.substeps_per_control

    @property
    def control_step_seconds(self) -> float:
        return self.control_step_ns * 1.0e-9


@dataclass(frozen=True, slots=True, eq=False)
class ApolloModelSpec:
    """调用方控制律可读取的 Apollo 刚体质量属性。"""

    name: str
    mass_kg: float
    center_of_mass_body_m: FloatVector
    diagonal_inertia_body_kg_m2: FloatVector

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("name must be a non-empty string")
        mass_kg = float(self.mass_kg)
        if not np.isfinite(mass_kg) or mass_kg <= 0.0:
            raise ValueError("mass_kg must be finite and positive")

        center_of_mass = _readonly_vector(
            "center_of_mass_body_m", self.center_of_mass_body_m, 3
        )
        inertia = _readonly_vector(
            "diagonal_inertia_body_kg_m2",
            self.diagonal_inertia_body_kg_m2,
            3,
        )
        if np.any(inertia <= 0.0):
            raise ValueError("diagonal_inertia_body_kg_m2 must be positive")

        object.__setattr__(self, "mass_kg", mass_kg)
        object.__setattr__(self, "center_of_mass_body_m", center_of_mass)
        object.__setattr__(self, "diagonal_inertia_body_kg_m2", inertia)

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, ApolloModelSpec)
            and self.name == other.name
            and self.mass_kg == other.mass_kg
            and bool(
                np.array_equal(
                    self.center_of_mass_body_m, other.center_of_mass_body_m
                )
            )
            and bool(
                np.array_equal(
                    self.diagonal_inertia_body_kg_m2,
                    other.diagonal_inertia_body_kg_m2,
                )
            )
        )


@dataclass(frozen=True, slots=True)
class PlantSnapshot:
    state: ApolloState
    control_tick: int
    physics_tick: int

    def __post_init__(self) -> None:
        if not isinstance(self.state, ApolloState):
            raise ValueError("state must be an ApolloState")
        object.__setattr__(
            self, "control_tick", _non_negative_integer("control_tick", self.control_tick)
        )
        object.__setattr__(
            self, "physics_tick", _non_negative_integer("physics_tick", self.physics_tick)
        )

    def sim_time_ns(self, timing: SimulationTiming) -> int:
        if not isinstance(timing, SimulationTiming):
            raise ValueError("timing must be a SimulationTiming")
        return self.physics_tick * timing.physics_step_ns

    def sim_time_seconds(self, timing: SimulationTiming) -> float:
        """像 Rust API 一样，由整数 physics tick 和显式 timing 派生时间。"""

        return self.sim_time_ns(timing) * 1.0e-9


@dataclass(frozen=True, slots=True)
class PlantStep:
    snapshot: PlantSnapshot
    requested_action: BodyWrench
    applied_action: BodyWrench

    def __post_init__(self) -> None:
        if not isinstance(self.snapshot, PlantSnapshot):
            raise ValueError("snapshot must be a PlantSnapshot")
        if not isinstance(self.requested_action, BodyWrench):
            raise ValueError("requested_action must be a BodyWrench")
        if not isinstance(self.applied_action, BodyWrench):
            raise ValueError("applied_action must be a BodyWrench")


@dataclass(frozen=True, slots=True)
class PropulsionStep:
    """推进器命令执行一个控制周期后的完整结果。"""

    snapshot: PlantSnapshot
    requested_command: PropulsionCommand
    applied: AppliedPropulsion

    def __post_init__(self) -> None:
        if not isinstance(self.snapshot, PlantSnapshot):
            raise ValueError("snapshot must be a PlantSnapshot")
        if not isinstance(self.requested_command, PropulsionCommand):
            raise ValueError("requested_command must be a PropulsionCommand")
        if not isinstance(self.applied, AppliedPropulsion):
            raise ValueError("applied must be an AppliedPropulsion")


def _validated_state_copy(state: ApolloState) -> ApolloState:
    if not isinstance(state, ApolloState):
        raise ValueError("state must be an ApolloState")
    return ApolloState(
        position_body_origin_world_m=state.position_body_origin_world_m,
        quaternion_body_to_world_wxyz=state.quaternion_body_to_world_wxyz,
        linear_velocity_body_origin_world_mps=(
            state.linear_velocity_body_origin_world_mps
        ),
        angular_velocity_body_radps=state.angular_velocity_body_radps,
    )


def _validated_wrench_copy(wrench: BodyWrench) -> BodyWrench:
    if not isinstance(wrench, BodyWrench):
        raise ValueError("wrench must be a BodyWrench")
    return BodyWrench(
        force_body_n=wrench.force_body_n,
        torque_about_com_body_nm=wrench.torque_about_com_body_nm,
    )


def _validated_snapshot_copy(snapshot: PlantSnapshot) -> PlantSnapshot:
    if not isinstance(snapshot, PlantSnapshot):
        raise ValueError("snapshot must be a PlantSnapshot")
    return PlantSnapshot(
        state=_validated_state_copy(snapshot.state),
        control_tick=snapshot.control_tick,
        physics_tick=snapshot.physics_tick,
    )


def _validated_step_copy(step: PlantStep) -> PlantStep:
    return PlantStep(
        snapshot=_validated_snapshot_copy(step.snapshot),
        requested_action=_validated_wrench_copy(step.requested_action),
        applied_action=_validated_wrench_copy(step.applied_action),
    )


def _validated_attitude_reference(value: ArrayLike | None) -> FloatVector | None:
    if value is None:
        return None
    quaternion = _readonly_vector(
        "quaternion_desired_body_to_world_wxyz", value, 4
    )
    norm = float(np.linalg.norm(quaternion))
    if abs(norm - 1.0) > _QUATERNION_NORM_TOLERANCE:
        raise ValueError(
            "quaternion_desired_body_to_world_wxyz must be a unit quaternion "
            f"(norm={norm:.16g})"
        )
    return quaternion


def _attitude_reference_to_json(quaternion_wxyz: FloatVector) -> dict[str, object]:
    return {
        "quaternion_desired_body_to_world_wxyz": quaternion_wxyz.tolist()
    }


class JsonlTrajectoryWriter:
    """与 Rust v1 schema 一致、由调用方显式驱动的 JSONL 记录器。"""

    __slots__ = ("_stream", "_timing", "_last_control_tick")

    def __init__(
        self,
        stream: TextIO,
        initial_snapshot: PlantSnapshot,
        timing: SimulationTiming,
        initial_desired_attitude_wxyz: ArrayLike | None = None,
    ) -> None:
        if not hasattr(stream, "write"):
            raise ValueError("stream must be a text stream with write()")
        validated_initial = _validated_snapshot_copy(initial_snapshot)
        if (
            validated_initial.control_tick != 0
            or validated_initial.physics_tick != 0
        ):
            raise ValueError(
                "initial_snapshot must be at control tick 0 and physics tick 0"
            )
        if not isinstance(timing, SimulationTiming):
            raise ValueError("timing must be a SimulationTiming")
        selected_timing = timing
        initial_reference = _validated_attitude_reference(
            initial_desired_attitude_wxyz
        )

        self._stream = stream
        self._timing = selected_timing
        self._last_control_tick: int | None = validated_initial.control_tick
        header: dict[str, object] = {
            "format": "apollo-telemetry-jsonl",
            "version": 1,
            "model": "apollo_lander",
            "timing": {
                "physics_step_ns": selected_timing.physics_step_ns,
                "substeps_per_control": selected_timing.substeps_per_control,
            },
            "initial_snapshot": _snapshot_to_json(validated_initial),
        }
        if initial_reference is not None:
            header["initial_attitude_reference"] = _attitude_reference_to_json(
                initial_reference
            )
        self._write_json_line(header)

    @property
    def timing(self) -> SimulationTiming:
        return self._timing

    def write_step(
        self,
        step: PlantStep,
        *,
        desired_attitude_wxyz: ArrayLike | None = None,
    ) -> None:
        """写一个调用方明确选择的 step；本对象不持有或驱动 plant。"""

        if not isinstance(step, PlantStep):
            raise ValueError("step must be a PlantStep")

        validated_step = _validated_step_copy(step)
        attitude_reference = _validated_attitude_reference(desired_attitude_wxyz)
        snapshot = validated_step.snapshot
        if snapshot.control_tick > (
            np.iinfo(np.uint64).max // self._timing.substeps_per_control
        ):
            raise ValueError(
                f"physics tick overflows for control tick {snapshot.control_tick}"
            )
        expected_physics_tick = (
            snapshot.control_tick * self._timing.substeps_per_control
        )
        if snapshot.physics_tick != expected_physics_tick:
            raise ValueError(
                f"physics tick {snapshot.physics_tick} does not match control tick "
                f"{snapshot.control_tick} (expected {expected_physics_tick})"
            )
        if (
            self._last_control_tick is not None
            and snapshot.control_tick <= self._last_control_tick
        ):
            raise ValueError(
                "control tick must be strictly increasing "
                f"(previous: {self._last_control_tick}, current: {snapshot.control_tick})"
            )

        frame = _step_to_json(validated_step)
        if attitude_reference is not None:
            frame["attitude_reference"] = _attitude_reference_to_json(
                attitude_reference
            )
        self._write_json_line(frame)
        self._last_control_tick = snapshot.control_tick

    def _write_json_line(self, value: object) -> None:
        # 禁止 NaN/Infinity，保持与 serde_json 的严格 JSON 行为一致。
        line = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self._stream.write(line)
        self._stream.write("\n")


def _state_to_json(state: ApolloState) -> dict[str, object]:
    return {
        "position_body_origin_world_m": state.position_body_origin_world_m.tolist(),
        # 不依赖 NumPy 或 glam 内部布局，持久格式固定为 wxyz。
        "quaternion_body_to_world_wxyz": state.quaternion_body_to_world_wxyz.tolist(),
        "linear_velocity_body_origin_world_mps": (
            state.linear_velocity_body_origin_world_mps.tolist()
        ),
        "angular_velocity_body_radps": state.angular_velocity_body_radps.tolist(),
    }


def _wrench_to_json(wrench: BodyWrench) -> dict[str, object]:
    return {
        "force_body_n": wrench.force_body_n.tolist(),
        "torque_about_com_body_nm": wrench.torque_about_com_body_nm.tolist(),
    }


def _snapshot_to_json(snapshot: PlantSnapshot) -> dict[str, object]:
    return {
        "state": _state_to_json(snapshot.state),
        "control_tick": snapshot.control_tick,
        "physics_tick": snapshot.physics_tick,
    }


def _step_to_json(step: PlantStep) -> dict[str, object]:
    return {
        "snapshot": _snapshot_to_json(step.snapshot),
        "requested_action": _wrench_to_json(step.requested_action),
        "applied_action": _wrench_to_json(step.applied_action),
    }


def _require_native() -> object:
    if _native is None:
        raise ImportError(
            "apollo_sim native extension is unavailable; run `maturin develop` or "
            "install a built wheel, and ensure the MuJoCo runtime library is "
            f"discoverable; original import error: {_native_import_failure}"
        ) from _native_import_failure
    return _native


def _snapshot_from_native(raw: tuple[Sequence[float], int, int]) -> PlantSnapshot:
    state, control_tick, physics_tick = raw
    return PlantSnapshot(
        state=ApolloState.from_vector(state),
        control_tick=control_tick,
        physics_tick=physics_tick,
    )


def _dps_command_to_native(
    command: DpsCommand,
) -> tuple[str, float | None, float, float]:
    return (
        command.mode.value,
        command.thrust_n,
        command.gimbal_x_rad,
        command.gimbal_z_rad,
    )


def _dps_command_from_native(
    raw: tuple[str, float | None, float, float],
) -> DpsCommand:
    mode, thrust_n, gimbal_x_rad, gimbal_z_rad = raw
    try:
        parsed_mode = DpsMode(mode)
    except ValueError as exc:
        raise RuntimeError(f"native plant returned unknown DPS mode: {mode}") from exc
    return DpsCommand(
        mode=parsed_mode,
        thrust_n=thrust_n,
        gimbal_x_rad=gimbal_x_rad,
        gimbal_z_rad=gimbal_z_rad,
    )


def _propulsion_command_from_native(
    raw: tuple[Sequence[int], tuple[str, float | None, float, float]],
) -> PropulsionCommand:
    rcs_on_time_ns, dps = raw
    return PropulsionCommand(
        rcs=RcsCommand(rcs_on_time_ns),
        dps=_dps_command_from_native(dps),
    )


def _applied_propulsion_from_native(
    raw: tuple[
        Sequence[int],
        Sequence[float],
        tuple[str, float, float, float, Sequence[float]],
        Sequence[float],
    ],
) -> AppliedPropulsion:
    gate_on_time_ns, mean_thrust_n, dps_raw, mean_wrench = raw
    mode, thrust_n, gimbal_x_rad, gimbal_z_rad, force_direction_body = dps_raw
    try:
        parsed_mode = DpsMode(mode)
    except ValueError as exc:
        raise RuntimeError(f"native plant returned unknown DPS mode: {mode}") from exc
    return AppliedPropulsion(
        rcs=AppliedRcs(
            applied_gate_on_time_ns=gate_on_time_ns,
            mean_thrust_n=mean_thrust_n,
        ),
        dps=AppliedDps(
            mode=parsed_mode,
            thrust_n=thrust_n,
            gimbal_x_rad=gimbal_x_rad,
            gimbal_z_rad=gimbal_z_rad,
            force_direction_body=force_direction_body,
        ),
        mean_wrench_body=BodyWrench.from_vector(mean_wrench),
    )


def _propulsion_spec_from_native(
    raw: tuple[
        Sequence[
            tuple[
                int,
                str,
                int,
                str,
                Sequence[float],
                Sequence[float],
                float,
                int,
            ]
        ],
        tuple[
            Sequence[float],
            Sequence[float],
            float,
            float,
            float,
            float,
            float,
        ],
    ],
) -> ApolloPropulsionSpec:
    rcs_raw, dps_raw = raw
    thrusters: list[RcsThrusterSpec] = []
    for (
        id_index,
        label,
        quad,
        feed_system,
        position_body_m,
        force_direction_body,
        steady_thrust_n,
        minimum_pulse_ns,
    ) in rcs_raw:
        try:
            thruster_id = RcsThrusterId(id_index)
            parsed_quad = RcsQuad(quad)
            parsed_feed_system = RcsFeedSystem(feed_system)
        except ValueError as exc:
            raise RuntimeError(
                "native plant returned an invalid RCS specification enum"
            ) from exc
        thrusters.append(
            RcsThrusterSpec(
                id=thruster_id,
                label=label,
                quad=parsed_quad,
                feed_system=parsed_feed_system,
                position_body_m=position_body_m,
                force_direction_body=force_direction_body,
                steady_thrust_n=steady_thrust_n,
                minimum_pulse_ns=minimum_pulse_ns,
            )
        )

    (
        gimbal_pivot_body_m,
        nominal_force_direction_body,
        variable_min_thrust_n,
        variable_max_thrust_n,
        full_thrust_n,
        maximum_gimbal_rad,
        gimbal_rate_rad_s,
    ) = dps_raw
    return ApolloPropulsionSpec(
        rcs_thrusters=tuple(thrusters),
        dps=DpsSpec(
            gimbal_pivot_body_m=gimbal_pivot_body_m,
            nominal_force_direction_body=nominal_force_direction_body,
            variable_min_thrust_n=variable_min_thrust_n,
            variable_max_thrust_n=variable_max_thrust_n,
            full_thrust_n=full_thrust_n,
            maximum_gimbal_rad=maximum_gimbal_rad,
            gimbal_rate_rad_s=gimbal_rate_rad_s,
        ),
    )


class ApolloPlantFactory:
    """编译并共享只读 MuJoCo 模型，用于创建相互独立的 plant。"""

    __slots__ = ("_model_spec", "_native_factory", "_timing")

    def __init__(self, timing: SimulationTiming | None = None) -> None:
        selected_timing = timing if timing is not None else SimulationTiming()
        if not isinstance(selected_timing, SimulationTiming):
            raise ValueError("timing must be a SimulationTiming")

        native = _require_native()
        self._native_factory = native.ApolloPlantFactory(
            selected_timing.physics_step_seconds,
            selected_timing.substeps_per_control,
        )
        name, mass_kg, center_of_mass, diagonal_inertia = (
            self._native_factory.model_spec()
        )
        self._model_spec = ApolloModelSpec(
            name=name,
            mass_kg=mass_kg,
            center_of_mass_body_m=center_of_mass,
            diagonal_inertia_body_kg_m2=diagonal_inertia,
        )
        self._timing = selected_timing

    @property
    def timing(self) -> SimulationTiming:
        return self._timing

    @property
    def model_spec(self) -> ApolloModelSpec:
        return self._model_spec

    def spawn(self, initial_state: ApolloState) -> ApolloPlant:
        if not isinstance(initial_state, ApolloState):
            raise ValueError("initial_state must be an ApolloState")
        native_plant = self._native_factory.spawn(initial_state.as_vector().tolist())
        return ApolloPlant(native_plant, self._timing)


class ApolloPlant:
    """同步、无线程、无 sleep 的外部动作驱动 plant。"""

    __slots__ = ("_native_plant", "_timing")

    def __init__(self, native_plant: object, timing: SimulationTiming) -> None:
        # 用户不直接构造；只由 ApolloPlantFactory.spawn 返回。
        self._native_plant = native_plant
        self._timing = timing

    @property
    def timing(self) -> SimulationTiming:
        return self._timing

    def reset(self, state: ApolloState) -> PlantSnapshot:
        if not isinstance(state, ApolloState):
            raise ValueError("state must be an ApolloState")
        return _snapshot_from_native(self._native_plant.reset(state.as_vector().tolist()))

    def snapshot(self) -> PlantSnapshot:
        return _snapshot_from_native(self._native_plant.snapshot())

    def step(self, action: BodyWrench) -> PlantStep:
        if not isinstance(action, BodyWrench):
            raise ValueError("action must be a BodyWrench")
        raw_snapshot, requested_action, applied_action = self._native_plant.step(
            action.as_vector().tolist()
        )
        return PlantStep(
            snapshot=_snapshot_from_native(raw_snapshot),
            requested_action=BodyWrench.from_vector(requested_action),
            applied_action=BodyWrench.from_vector(applied_action),
        )


class ApolloPropulsionPlantFactory:
    """编译带 Apollo RCS 与下降推进系统的 MuJoCo plant 工厂。"""

    __slots__ = (
        "_model_spec",
        "_native_factory",
        "_propulsion_spec",
        "_timing",
    )

    def __init__(self, timing: SimulationTiming | None = None) -> None:
        selected_timing = timing if timing is not None else SimulationTiming()
        if not isinstance(selected_timing, SimulationTiming):
            raise ValueError("timing must be a SimulationTiming")

        native = _require_native()
        self._native_factory = native.ApolloPropulsionPlantFactory(
            selected_timing.physics_step_seconds,
            selected_timing.substeps_per_control,
        )
        name, mass_kg, center_of_mass, diagonal_inertia = (
            self._native_factory.model_spec()
        )
        self._model_spec = ApolloModelSpec(
            name=name,
            mass_kg=mass_kg,
            center_of_mass_body_m=center_of_mass,
            diagonal_inertia_body_kg_m2=diagonal_inertia,
        )
        self._propulsion_spec = _propulsion_spec_from_native(
            self._native_factory.propulsion_spec()
        )
        self._timing = selected_timing

    @classmethod
    def apollo11_touchdown(
        cls, timing: SimulationTiming | None = None
    ) -> ApolloPropulsionPlantFactory:
        """构造完整、尚未分级的 Apollo 11 LM-5 RCS + DPS 工厂。"""

        return cls(timing)

    @property
    def timing(self) -> SimulationTiming:
        return self._timing

    @property
    def model_spec(self) -> ApolloModelSpec:
        return self._model_spec

    @property
    def propulsion_spec(self) -> ApolloPropulsionSpec:
        return self._propulsion_spec

    def spawn(self, initial_state: ApolloState) -> ApolloPropulsionPlant:
        if not isinstance(initial_state, ApolloState):
            raise ValueError("initial_state must be an ApolloState")
        native_plant = self._native_factory.spawn(initial_state.as_vector().tolist())
        return ApolloPropulsionPlant(
            native_plant, self._timing, self._propulsion_spec
        )


class ApolloPropulsionPlant:
    """同步推进器 plant；每次 step 严格推进一个固定控制周期。"""

    __slots__ = ("_native_plant", "_propulsion_spec", "_timing")

    def __init__(
        self,
        native_plant: object,
        timing: SimulationTiming,
        propulsion_spec: ApolloPropulsionSpec,
    ) -> None:
        self._native_plant = native_plant
        self._timing = timing
        self._propulsion_spec = propulsion_spec

    @property
    def timing(self) -> SimulationTiming:
        return self._timing

    @property
    def propulsion_spec(self) -> ApolloPropulsionSpec:
        return self._propulsion_spec

    def reset(self, state: ApolloState) -> PlantSnapshot:
        if not isinstance(state, ApolloState):
            raise ValueError("state must be an ApolloState")
        return _snapshot_from_native(self._native_plant.reset(state.as_vector().tolist()))

    def snapshot(self) -> PlantSnapshot:
        return _snapshot_from_native(self._native_plant.snapshot())

    def step(self, command: PropulsionCommand) -> PropulsionStep:
        if not isinstance(command, PropulsionCommand):
            raise ValueError("command must be a PropulsionCommand")
        dps_mode, dps_thrust_n, gimbal_x_rad, gimbal_z_rad = (
            _dps_command_to_native(command.dps)
        )
        raw_snapshot, requested_command, applied = self._native_plant.step(
            command.rcs.as_vector().tolist(),
            dps_mode,
            dps_thrust_n,
            gimbal_x_rad,
            gimbal_z_rad,
        )
        return PropulsionStep(
            snapshot=_snapshot_from_native(raw_snapshot),
            requested_command=_propulsion_command_from_native(requested_command),
            applied=_applied_propulsion_from_native(applied),
        )

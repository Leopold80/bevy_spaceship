# API 参考手册

本页是 Rust 与 Python 公共接口的逐项参考。若想先看如何组合完整程序，请阅读
[使用方式](usage.md)；可运行代码集中在 [例程目录](../examples/README.md)。

## 共同执行模型

Rust 与 Python 调用同一份 Rust/MuJoCo 实现。仓库有两种输入层级，但遵循相同的同步
生命周期：

```text
factory -> spawn(explicit initial state) -> plant
plant.snapshot()                       -> current snapshot, no advance
plant.step(explicit command)           -> exactly one control period
plant.reset(explicit state)            -> tick 0 snapshot
```

plant 不创建线程、不按 wall clock 等待，也不持有控制器、目标、奖励、episode、记录器
或 viewer。一次 `step` 默认包含 10 个 2 ms MuJoCo 子步，即推进 20 ms 仿真时间。

| plant | `step` 输入 | 用途 |
|---|---|---|
| `ApolloPlant` | 理想 `BodyWrench` | 控制律与刚体动力学基线 |
| `ApolloPropulsionPlant` | `PropulsionCommand` | 16 路 RCS、DPS、安装点和执行器约束 |

两者有意并存；推进器 plant 不先把喷口合成为理想 wrench 再注入，而是在 MuJoCo 中对
各自安装点施力。

## 坐标系、单位与数组顺序

本项目使用右手坐标系；单位姿态时机体系与世界系重合，模型 `+X` 向右、`+Y` 向上、
`+Z` 向前。公共浮点数均为双精度。

| 语义 | Rust | Python / JSON | 表达与单位 |
|---|---|---|---|
| 机体系原点位置 | `position_body_origin_world_m` | 同名 | 世界系，m |
| 机体到世界姿态 | `body_to_world` | `quaternion_body_to_world_wxyz` | 主动旋转，wxyz |
| 机体系原点线速度 | `linear_velocity_body_origin_world_mps` | 同名 | 世界系，m/s |
| 角速度 | `angular_velocity_body_radps` | 同名 | 机体系，rad/s |
| 合力 | `force_body_n` | 同名 | 机体系，等效作用于质心，N |
| 合力矩 | `torque_about_com_body_nm` | 同名 | 机体系，关于质心，N·m |

扁平状态固定为 13 维：

```text
[position(3), quaternion_wxyz(4), linear_velocity(3), angular_velocity(3)]
```

扁平动作固定为 6 维：

```text
[force_body_n(3), torque_about_com_body_nm(3)]
```

状态描述的是机体系原点而非质心。设 `r_com_body` 为质心偏置，则：

```text
p_com = p_origin + R_body_to_world r_com_body
v_com = v_origin + omega_world x (R_body_to_world r_com_body)
```

非零角速度下若要令初始质心静止，应显式设置
`v_origin = -omega_world x (R_body_to_world r_com_body)`。

## Rust API

### crate 入口

- `apollo-mujoco`：通常由控制程序直接依赖；重导出主要状态、动作、时序和 plant 类型。
- `apollo-core`：后端中立的数据契约、`Plant` trait、模型规格和 JSONL writer。

外部工程最小依赖：

```toml
[dependencies]
apollo-mujoco = { path = "../bevy_spaceship/crates/apollo-mujoco" }
```

需要记录轨迹时再加入 `apollo-core`。

### `ApolloPlantFactory`

| 接口 | 返回 | 说明 |
|---|---|---|
| `ApolloPlantFactory::apollo_touchdown()` | `Result<Self, PlantError>` | 默认 Apollo 模型与默认时序 |
| `ApolloPlantFactory::new(model_spec, timing)` | `Result<Self, PlantError>` | 用显式模型规格和时序编译 MuJoCo 模型 |
| `spawn(initial_state)` | `Result<ApolloPlant, PlantError>` | 创建独立 MuJoCo data，并重置到给定状态 |
| `model_spec()` | `ApolloModelSpec` | 返回质量、质心偏置和惯量 |
| `timing()` | `SimulationTiming` | 返回工厂固定时序 |

工厂可 clone；clone 共享只读 MuJoCo model。每次 `spawn` 的状态、外力和 tick 相互独立。

### `ApolloPlant`

| 接口 | 返回 | 副作用 |
|---|---|---|
| `snapshot()` | `PlantSnapshot` | 无推进 |
| `step(action)` | `Result<PlantStep, PlantError>` | 恰好推进一个控制周期 |
| `reset(initial_state)` | `Result<PlantSnapshot, PlantError>` | 清空动力学状态和 tick |
| `timing()` | `SimulationTiming` | 无推进 |

`ApolloPlant` 同时实现后端中立的 `apollo_core::Plant` trait。动作在一个控制周期内按
机体系零阶保持；刚体转动时，后端在每个物理子步重新换算世界系 wrench。

### `ApolloPropulsionPlantFactory`

| 接口 | 返回 | 说明 |
|---|---|---|
| `apollo11_touchdown()` | `Result<Self, PlantError>` | Apollo 11 LM-5 着陆构型和默认时序 |
| `new(model_spec, propulsion_spec, timing)` | `Result<Self, PlantError>` | 显式规格；控制周期不得短于 RCS 最小脉冲 |
| `spawn(initial_state)` | `Result<ApolloPropulsionPlant, PlantError>` | 创建状态与执行器历史独立的 plant |
| `model_spec()` | `ApolloModelSpec` | 固定质量、质心和惯量 |
| `propulsion_spec()` | `ApolloPropulsionSpec` | 16 路 RCS 与 DPS 只读规格副本 |
| `timing()` | `SimulationTiming` | 工厂固定时序 |

### `ApolloPropulsionPlant`

| 接口 | 返回 | 说明 |
|---|---|---|
| `snapshot()` | `PlantSnapshot` | 读取但不推进 |
| `step(command)` | `Result<PropulsionStep, PlantError>` | 执行 RCS/DPS 命令并推进一个控制周期 |
| `reset(initial_state)` | `Result<PlantSnapshot, PlantError>` | 清空 MuJoCo 状态、tick、RCS 尾迹，并将 DPS GDA 回中 |
| `timing()` | `SimulationTiming` | 固定时序 |
| `propulsion_spec()` | `ApolloPropulsionSpec` | 当前推进规格 |

它没有实现以 `BodyWrench` 为动作的 `Plant` trait，避免把两种动作语义混在一起。

### 推进规格与稳定 RCS 顺序

`ApolloPropulsionSpec::apollo11_touchdown()` 包含：

- `rcs_thrusters: [RcsThrusterSpec; 16]`：ID、历史标签、quad、A/B 供给、机体系位置、
  飞船受力单位方向、100 lbf 稳态推力和 14 ms 最小脉冲；
- `dps: DpsSpec`：枢轴、名义推力方向、可调推力上下限、独立全推力值、6° 圆锥摆角和
  `0.2°/s` GDA 摆速。

16 路数组顺序固定为：

```text
0 A1U   1 B1D   2 A1F   3 B1L
4 B2U   5 A2D   6 A2A   7 B2L
8 A3U   9 B3D  10 B3A  11 A3R
12 B4U 13 A4D  14 B4F  15 A4R
```

`RcsThrusterId::new(index)` 检查 `0..15`，`index()` 返回数组位置，`label()` 返回历史
标签。标签末字母描述喷管/羽流朝向，`force_direction_body` 是反向的飞船受力方向。
完整物理背景和 NASA/项目坐标转换见
[Apollo 11 月球舱推进系统说明](apollo_propulsion_system.md)。

### 推进命令

`PropulsionCommand { rcs, dps }` 联合提交两套系统；`PropulsionCommand::OFF` 全部关闭。

`RcsCommand` 的 `on_time_ns: [u64; 16]` 从当前控制周期起点计时：

- `OFF` / `off()`：全关闭；
- `single_pulse(id, duration_ns)`：单路请求；
- `with_on_time(id, duration_ns)`：在已有命令上设置一路；
- `hold(ids, timing)`：指定喷口请求完整控制周期，适合连续点火；
- `from_on_times(array)`：直接提供稳定顺序的 16 路数组。
- `applied_gate_on_times(spec, timing)`：低层无状态归一化，明确假定周期起点 16 路全关；
- `applied_gate_on_times_with_initial_gate_state(spec, timing, initial_gate_open)`：供有状态
  plant 在周期边界传入各阀门初态；通常应用代码无需直接调用。

周期起点阀门关闭时，非零且短于 14 ms 的新脉冲提升到 14 ms；若上一 tick 的门控恰好
延续到边界，则短请求是连续点火的精确末段，不再重复套用最小脉冲。例如 `20 ms + 1 ms`
得到连续 `21 ms`，而真正关断后再请求 `1 ms` 仍提升到 `14 ms`。超过当前控制周期的请求
使整个 step 返回错误且不推进。门控时间不会按 2 ms 子步取整。

`DpsCommand` 是显式枚举：

```rust
DpsCommand::Off
DpsCommand::Variable { thrust_n, gimbal_x_rad, gimbal_z_rad }
DpsCommand::FullThrust { gimbal_x_rad, gimbal_z_rad }
```

可调推力钳在 1,050–6,300 lbf，独立全推力档为 9,870 lbf。`gimbal_x_rad` / `z` 表示
推力分别朝机体 `+X` / `+Z` 倾斜的目标分量，不是绕同名轴旋转。目标二维长度先钳在
6° 圆锥内；有状态 GDA 随后在每个物理子步按二维模长 `gimbal_rate_rad_s` 追踪目标。
Apollo 11 默认值为 `0.2°/s`，所以 2 ms 子步最多移动 `0.0004°`，默认 20 ms 控制周期
最多移动 `0.004°`。该执行机构动态不要求缩短已有仿真步长或控制周期。

`Off` 令 DPS 推力为零，但保持上一个实际 GDA 角；下一次有推力命令从该角继续追踪。
`reset` 才把 GDA 回中。无效命令不推进 plant，也不改变该执行机构状态。

### `PropulsionStep` 与实际输出

`PropulsionStep`：

- `snapshot`：推进后的状态与 tick；
- `requested_command`：调用方原始请求；
- `applied: AppliedPropulsion`：执行器实际结果。

`AppliedPropulsion`：

- `rcs[16].applied_gate_on_time_ns`：最小脉冲处理后的阀门门控时间；
- `rcs[16].mean_thrust_n`：含启动和关断尾迹的本控制周期平均推力；
- `dps`：实际模式、推力、本周期末 GDA 实际摆角和机体系受力方向；
- `mean_wrench_body`：所有点力关于当前固定质心合成并按控制周期平均的 wrench。

门控时间不是“非零推力持续时间”。RCS 执行器跨 tick 保存状态，因而全关闭命令后的
一个周期仍可能包含上一脉冲的关断尾迹；`reset` 会清掉它。

同理，`requested_command.dps` 中的摆角是目标角，不能当作本周期实际摆角；应读取
`applied.dps.gimbal_x_rad/gimbal_z_rad`。点力与 `mean_wrench_body` 在每个物理子步都按
当时的实际 GDA 位置计算，`AppliedDps` 报告周期末位置。

### 状态与动作

`ApolloState` 的四个公共字段见上方坐标表。

- `ApolloState::ZERO` / `Default`：原点、单位姿态、零速度。
- `validate()`：校验所有数值有限且姿态为单位四元数，不修改输入。
- `with_normalized_attitude()`：显式归一化有限、非退化四元数。

`BodyWrench` 有 `force_body_n` 与 `torque_about_com_body_nm` 两个字段。

- `BodyWrench::ZERO` / `Default`：零合力和零合力矩。
- `validate()`：校验六个动作分量均为有限数。

### 快照与单步结果

`PlantSnapshot`：

- `state: ApolloState`
- `control_tick: u64`：自最近 reset 后完成的外部控制步数
- `physics_tick: u64`：自最近 reset 后完成的物理子步数
- `sim_time_ns(timing)` / `sim_time_seconds(timing)`：从整数 tick 派生时间

`PlantStep`：

- `snapshot`：动作执行后的快照
- `requested_action`：调用方提交的动作
- `applied_action`：后端实际应用的动作；当前理想 wrench plant 与 requested 相同

requested/applied 分开保留，可直接观察推进器 plant 的最小脉冲、节流和摆角钳位。

### `SimulationTiming`

- `SimulationTiming::APOLLO` / `Default`：2,000,000 ns，10 子步/控制步。
- `new(nonzero_ns, nonzero_substeps)`：用非零整数类型构造。
- `from_raw(ns, substeps)`：任一为零时返回 `None`。
- `physics_step_seconds()`、`control_step_ns()`、`control_step_seconds()`：派生周期。
- `sim_time_ns(physics_tick)`、`sim_time_seconds(physics_tick)`：派生时间。
- `physics_ticks_for_control_ticks(control_ticks)`：检查溢出的 tick 换算。

整数纳秒和整数 tick 是权威时间源。

### `ApolloModelSpec`

| 字段 | 含义 |
|---|---|
| `name` | 稳定模型名 |
| `mass_kg` | 总质量 |
| `center_of_mass_body_m` | 机体系中的质心位置 |
| `diagonal_inertia_body_kg_m2` | 绕质心、沿机体 XYZ 轴的对角惯量 |

默认规格由 `ApolloModelSpec::touchdown()` 返回。位置控制、质心状态换算或执行器分配应
从工厂读取规格，不应在调用方复制常数。

### Rust 轨迹记录

`TrajectoryHeader::apollo(timing, initial_snapshot)` 创建 v1 header；初始快照必须是 reset
后的 tick 0。`with_initial_desired_attitude(q)` 可附加 tick 0 的期望姿态。

`JsonlTrajectoryWriter<W>`：

- `new(writer, header)`：校验并立即写入首行 header。
- `write_frame(&TelemetryFrame)`：写入严格递增、与时序对齐的帧。
- `get_ref()` / `get_mut()` / `into_inner()`：访问底层 `Write`。

`TelemetryFrame::from(plant_step)` 从单步结果构造帧；
`with_desired_attitude(q)` 附加调用方期望姿态。记录器不持有或推进 plant。

### Rust 错误

`PlantError` 区分模型规格/加载、初态、理想动作、推进规格、推进命令、点力施加、仿真
状态和 tick 溢出错误。
`TrajectoryWriteError` 区分 header、帧、tick 顺序、序列化和 I/O 错误。调用方应传播或
显式处理这些错误，不要在控制循环中静默忽略。

### 其他 Rust 公开项

以下接口用于后端、工具或高级集成，普通控制循环通常不需要直接调用：

- `apollo_mujoco::generate_apollo_mjcf(model_spec, timing)`：生成当前单刚体模型的 MJCF。
- `apollo_core::apollo_visual_parts()`：后端中立的可视部件清单。
- `apollo_core::apollo_collision_parts()`：碰撞/物理几何清单。
- `apollo_core::apollo_mass_points()`、`total_physics_mass_kg()`、
  `center_of_mass_body_m()`：质量点与派生质量属性。
- `validate_finite_vec3()`、`validate_finite_quaternion()`、
  `validate_unit_quaternion()`、`normalized_quaternion()`：边界输入校验工具。

对应的几何枚举、部件结构、稳定名称与基线常量也从 `apollo-core` 导出。它们不改变
plant 的 `reset/snapshot/step` 主契约。

## Python API

公共名称均从 `apollo_sim` 导入：

```python
from apollo_sim import (
    ApolloModelSpec,
    ApolloPlant,
    ApolloPlantFactory,
    ApolloPropulsionPlant,
    ApolloPropulsionPlantFactory,
    ApolloPropulsionSpec,
    ApolloState,
    AppliedDps,
    AppliedPropulsion,
    AppliedRcs,
    BodyWrench,
    DpsCommand,
    DpsMode,
    DpsSpec,
    JsonlTrajectoryWriter,
    PlantSnapshot,
    PlantStep,
    PropulsionCommand,
    PropulsionStep,
    RcsCommand,
    RcsThrusterId,
    SimulationTiming,
)
```

Python 领域对象是 frozen dataclass，数组为 `numpy.float64` 并默认只读。这用于尽早发现
误改，不是安全隔离；调用方仍可主动改变 NumPy write flag。

### `ApolloPlantFactory`

```python
factory = ApolloPlantFactory(timing: SimulationTiming | None = None)
```

| 成员 | 类型或返回 | 说明 |
|---|---|---|
| `timing` | `SimulationTiming` | 只读属性 |
| `model_spec` | `ApolloModelSpec` | 只读质量属性 |
| `spawn(initial_state)` | `ApolloPlant` | 初态必须显式提供 |

### `ApolloPlant`

`ApolloPlant` 只由 factory 创建，不应直接构造。

| 成员 | 返回 | 说明 |
|---|---|---|
| `timing` | `SimulationTiming` | 只读属性 |
| `snapshot()` | `PlantSnapshot` | 读取但不推进 |
| `step(action)` | `PlantStep` | 推进一个控制周期，只接收 `BodyWrench` |
| `reset(state)` | `PlantSnapshot` | 重置状态和 tick |

当前 PyO3 factory/plant 是线程受限对象，`step()` 不释放 GIL。多实例并行采样应优先让
每个 worker 进程自行创建 factory/plant，不要在线程间共享同一实例。

### Python 数据类型

`ApolloState`：

- 构造函数接收四个具名字段；单位和坐标见本页开头。
- `identity()`：原点、单位姿态、零速度。
- `from_vector(value)`：从固定 13 维顺序构造。
- `as_vector()`：返回固定 13 维只读数组。

`BodyWrench`：

- 构造函数接收 `force_body_n` 和 `torque_about_com_body_nm`。
- `zero()`：零动作。
- `from_vector(value)` / `as_vector()`：6 维动作转换。

`SimulationTiming`：

- `SimulationTiming()` 默认 0.002 s、10 子步/控制步。
- 构造参数 `physics_step_seconds` 必须能精确规范化为正整数纳秒。
- 属性：`physics_step_seconds`、`physics_step_ns`、`substeps_per_control`、
  `control_step_seconds`、`control_step_ns`。

`ApolloModelSpec` 提供 `name`、`mass_kg`、`center_of_mass_body_m` 和
`diagonal_inertia_body_kg_m2`。

`PlantSnapshot` 提供 `state`、`control_tick`、`physics_tick`，以及
`sim_time_ns(timing)` / `sim_time_seconds(timing)`。

`PlantStep` 提供 `snapshot`、`requested_action` 和 `applied_action`。

### Python 推进接口

```python
factory = ApolloPropulsionPlantFactory.apollo11_touchdown()
plant = factory.spawn(ApolloState.identity())

command = PropulsionCommand(
    rcs=RcsCommand.single_pulse(RcsThrusterId.A1U, 14_000_000),
    dps=DpsCommand.off(),
)
step = plant.step(command)
```

`ApolloPropulsionPlantFactory` 的 `timing`、`model_spec`、`propulsion_spec` 均为只读属性；
构造函数和 `apollo11_touchdown(timing=None)` 都可接收自定义 `SimulationTiming`。
`ApolloPropulsionPlant` 提供同样的 `timing/reset/snapshot` 生命周期，`step` 只接收
`PropulsionCommand`。

Python 类型对应关系：

| 类型 | 关键成员 |
|---|---|
| `RcsThrusterId` | 16 个 `IntEnum` 名称，例如 `A1U`、`B1D` |
| `RcsCommand` | `on_time_ns`: 形状 `(16,)` 的只读 `numpy.uint64` |
| `DpsCommand` | `off()`、`variable(...)`、`full_thrust(...)` |
| `DpsSpec` | 推力范围、`maximum_gimbal_rad` 与 `gimbal_rate_rad_s` |
| `ApolloPropulsionSpec` | `rcs_thrusters` 只读 tuple、`dps` |
| `AppliedRcs` | `applied_gate_on_time_ns` 与 `mean_thrust_n` 两个只读数组 |
| `AppliedDps` | 实际模式、推力、摆角与 `force_direction_body` |
| `AppliedPropulsion` | `rcs`、`dps`、`mean_wrench_body` |
| `PropulsionStep` | `snapshot`、`requested_command`、`applied` |

这些 Python 领域对象同样是 frozen dataclass。Python 层先做形状、枚举和有限性检查，
真正的最小脉冲、DPS 目标限幅与 GDA 摆速、RCS 瞬态及 MuJoCo 积分只在 Rust 端执行。

### Python 轨迹记录

```python
writer = JsonlTrajectoryWriter(
    stream,
    initial_snapshot,
    timing,
    initial_desired_attitude_wxyz=None,
)
writer.write_step(step, desired_attitude_wxyz=None)
```

- `stream` 必须是文本流并提供 `write()`。
- `initial_snapshot` 必须是 reset 后的 tick 0 快照。
- `timing` 应直接使用 `plant.timing`。
- 期望姿态可省略；提供时必须是 wxyz 单位四元数。
- `write_step` 只记录调用方提交的 step，不驱动 plant。
- `timing` 属性返回 writer 固定时序。

输入类型、长度、有限性、单位四元数和 tick 对齐错误通常抛出 `ValueError`；原生模型
加载或仿真故障映射为 `RuntimeError`；原生扩展或 MuJoCo 动态库不可用时抛出
`ImportError` 并提示重新构建/配置运行环境。

## JSONL 与 viewer 命令

JSONL 第一行是版本化 header，后续每行是一个遥测帧。Rust 和 Python writer 生成相同
v1 schema；姿态持久化顺序始终为 wxyz。

当前 JSONL v1 只接收理想 `PlantStep<BodyWrench>`，尚未定义推进器命令/实际输出的 v2
schema；不要把 `PropulsionStep` 强行写入 v1 writer。

```bash
# 只校验，不打开窗口
cargo run -p apollo-viewer --bin apollo-replay -- --validate-only runs/my_run.jsonl

# 离线回放
cargo run -p apollo-viewer --bin apollo-replay -- runs/my_run.jsonl

# 只查看模型
cargo run -p apollo-viewer --bin apollo-model-viewer

# 生成 8 全景 + 4 安装座 + 4 喷流/导流板 + 1 DPS，共 17 张视觉 QA 图
cargo run -p apollo-viewer --bin apollo-model-viewer -- \
  --capture-all target/visual-qa/apollo-propulsion
```

viewer 是轨迹消费者，不会反向驱动 plant。回放中粗实线坐标系表示调用方记录的期望
姿态，细半透明坐标系表示当前姿态；未记录期望姿态时粗实线隐藏。

## 非 API 内容

以下内容有意不属于当前公共 API：

- 闭环 runner 或 wall-clock 调度器
- 控制器、制导律与目标生成器
- reward、terminated、truncated、episode 和 Gymnasium wrapper
- 自动喷口分配器、推进剂管理与故障重构
- viewer 生命周期和输入事件

它们应在调用方、独立 adapter 或例程层组合。若未来增加 Gym 接口，也应包装现有
现有 plant，而不是改变这里的同步 `reset/snapshot/step` 契约。

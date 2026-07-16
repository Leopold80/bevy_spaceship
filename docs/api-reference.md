# API 参考手册

本页是 Rust 与 Python 公共接口的逐项参考。若想先看如何组合完整程序，请阅读
[使用方式](usage.md)；可运行代码集中在 [例程目录](../examples/README.md)。

## 共同执行模型

两套 API 调用同一个 Rust/MuJoCo plant，并遵循相同的同步契约：

```text
factory -> spawn(explicit initial state) -> plant
plant.snapshot()                       -> current snapshot, no advance
plant.step(explicit body wrench)       -> exactly one control period
plant.reset(explicit state)            -> tick 0 snapshot
```

plant 不创建线程、不按 wall clock 等待，也不持有控制器、目标、奖励、episode、记录器
或 viewer。一次 `step` 默认包含 10 个 2 ms MuJoCo 子步，即推进 20 ms 仿真时间。

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

两种动作分开保留，是为了将来加入限幅或执行器分配后仍能观察差异。

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

`PlantError` 区分模型规格/加载、初态、动作、仿真状态和 tick 溢出错误。
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
    ApolloState,
    BodyWrench,
    JsonlTrajectoryWriter,
    PlantSnapshot,
    PlantStep,
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

```bash
# 只校验，不打开窗口
cargo run -p apollo-viewer --bin apollo-replay -- --validate-only runs/my_run.jsonl

# 离线回放
cargo run -p apollo-viewer --bin apollo-replay -- runs/my_run.jsonl

# 只查看模型
cargo run -p apollo-viewer --bin apollo-model-viewer
```

viewer 是轨迹消费者，不会反向驱动 plant。回放中粗实线坐标系表示调用方记录的期望
姿态，细半透明坐标系表示当前姿态；未记录期望姿态时粗实线隐藏。

## 非 API 内容

以下内容有意不属于当前公共 API：

- 闭环 runner 或 wall-clock 调度器
- 控制器、制导律与目标生成器
- reward、terminated、truncated、episode 和 Gymnasium wrapper
- 执行器/喷口分配与限幅
- viewer 生命周期和输入事件

它们应在调用方、独立 adapter 或例程层组合。若未来增加 Gym 接口，也应包装现有
`ApolloPlant`，而不是改变这里的同步 `reset/snapshot/step` 契约。

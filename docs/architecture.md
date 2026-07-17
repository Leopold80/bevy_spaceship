# 架构与边界

## 目标

本仓库提供一个可被外部算法直接驱动的 Apollo 风格 MuJoCo 被控对象，并用 Bevy 显示状态或记录轨迹。强化学习策略、经典控制律、制导律和可视化都属于 plant 的调用方，不是 plant 内部的运行模式。

核心契约是：

```text
显式初始状态 + 显式 BodyWrench 或 PropulsionCommand -> 确定性推进一个控制周期
```

## Workspace 依赖方向

实际 crate 依赖为：

```text
apollo-python  ------> apollo-mujoco ------> apollo-core
       |                                      ^
       +--------------------------------------+

apollo-viewer  -----------------------------> apollo-core
       |
       +-- optional `live` feature ---------> apollo-mujoco
```

- `apollo-core`：后端无关的状态、动作、时序、Apollo 规格和遥测数据。
- `apollo-mujoco`：唯一的动力学实现和 Rust plant API。
- `apollo-python`：同时使用 core 数据契约与同一 MuJoCo plant 的 PyO3 薄绑定，不复制动力学。
- `apollo-viewer`：消费模型规格、遥测或轨迹的 Bevy 显示程序。默认不链接 MuJoCo；只有 `live` feature 下的应用例程组合 `apollo-mujoco`。

禁止出现以下反向依赖：

- plant 不依赖控制器、制导律、RL task、记录器或 viewer。
- core 不依赖 MuJoCo、Bevy 或 Python。
- Python 层不重新积分状态或重算力/力矩。
- viewer 的渲染帧时间不参与物理推进。

第一版 Python 绑定保持对象线程绑定；并行采样优先由调用方以多进程组合多个独立 plant。这个限制属于语言绑定，不会让 plant 内部创建线程或调度闭环。

## Plant 契约

Rust 和 Python API 共享同一组语义：

```text
factory.spawn(explicit_state)    -> plant
plant.reset(explicit_state)      -> snapshot at tick 0
plant.step(explicit_command)     -> step result after one control tick
plant.snapshot()                 -> current snapshot
plant timing                     -> fixed timing
```

Rust 使用 `plant.timing()` 方法，Python 使用 `plant.timing` 属性。两种语言都要求 `spawn()` 显式接收初始状态。

工厂还以只读方式暴露模型质量属性：Rust 使用 `factory.model_spec()`，Python 使用
`factory.model_spec`。这让调用方的位置控制、观测换算和后续执行器分配使用与 plant
相同的质量、质心和惯量数据，但不会把控制器目标或状态放进 factory。

推进器工厂另外暴露同一份 `propulsion_spec`，并创建独立的
`ApolloPropulsionPlant`。理想 `ApolloPlant` 与推进器 plant 不共用一个全局动作枚举。

`step()` 每次恰好推进一个控制周期。默认时序为：

```text
MuJoCo physics step:  0.002 s
substeps per action:  10
control step:         0.020 s
```

动作在十个物理小步内使用零阶保持。整数 `control_tick` 和 `physics_tick` 是权威时间源，秒数只由 tick 与固定步长派生，不能在多个模块中独立累加浮点时间。

DPS 命令的摆角目标也在一个控制周期内零阶保持，但 GDA 实际位置不是跳变：目标先限制
到 6° 圆锥，实际二维摆角再在每个 2 ms 物理子步按 `0.2°/s` 追踪。默认每子步最多
`0.0004°`、每个 20 ms 控制周期最多 `0.004°`，因此无需改变既有仿真或控制周期。

Rust `SimulationTiming` 直接保存整数 `physics_step_ns`。Python 构造函数接收 `physics_step_seconds`，但只接受能由正整数纳秒表示的值，并将规范化后的权威整数暴露为 `physics_step_ns`。

## 坐标系、参考点和字段名

公共数据使用 `f64`，与 MuJoCo 内部精度一致。本项目使用右手坐标系；单位姿态时机体系与世界系重合。Apollo 模型的 `+X` 向右、`+Y` 向上、`+Z` 向前，因此 `+X × +Y = +Z`。`body_to_world` 是把机体系向量主动旋转到世界系的单位四元数。

| 语义 | Rust 字段 | Python / JSON 字段 | 单位或顺序 |
|---|---|---|---|
| 机体系原点的世界系位置 | `position_body_origin_world_m` | `position_body_origin_world_m` | m |
| 机体系到世界系姿态 | `body_to_world` | `quaternion_body_to_world_wxyz` | wxyz |
| 机体系原点的世界系线速度 | `linear_velocity_body_origin_world_mps` | `linear_velocity_body_origin_world_mps` | m/s |
| 机体系角速度 | `angular_velocity_body_radps` | `angular_velocity_body_radps` | rad/s |
| 机体系力，等效作用于质心 | `force_body_n` | `force_body_n` | N |
| 关于质心的机体系力矩 | `torque_about_com_body_nm` | `torque_about_com_body_nm` | N·m |

Apollo 规格的质心相对机体系原点约偏移 `(0, 2.013, 0) m`。状态中的平移量明确对应机体系原点；wrench 明确对应质心，不能在导航、喷口或日志代码中混用两个参考点。

因此非零角速度下的“零原点线速度”一般不等于“零质心速度”。调用方若要构造静止
质心初态或位置环，应使用刚体运动学
`v_com = v_origin + omega_world x (R_body_to_world r_com_body)` 完成参考点换算。

Bevy 只在显示边界把 `f64` 转为 `f32`。Python 公共层使用 `numpy.float64`。这些转换不能改变字段的坐标系含义。

持久 JSON 使用显式姿态字段 `quaternion_body_to_world_wxyz`，顺序固定为 wxyz。Rust 内部 `DQuat` 的存储或 serde 默认约定不属于轨迹格式契约。

## 为什么没有 ClosedLoopRunner

闭环属于调用方：

```text
snapshot -> user controller/guidance/policy -> BodyWrench -> plant.step()
```

仓库提供 Rust 与 Python 闭环例程，但不提供公共 `ControlLaw`、`Guidance` 或 `ClosedLoopRunner`。这样不会把某种控制器生命周期、目标表示、奖励或采样方式写进被控对象。

live 例程可以在应用层创建线程、按 wall clock 等待并向 Bevy 发布最新快照；这些能力只存在于该二进制例程，不会反向进入 `apollo-mujoco`。

## 动作层级

公共动作现在有两条并行路径：理想机体系六维 wrench 用于验证刚体动力学和算法接口；
独立 `PropulsionCommand` 直接表达 16 路 RCS 阀门时间与 DPS 档位。它们不扩展成一个
全局 `Action` 枚举：

```text
desired wrench -> future allocator -> PropulsionCommand -> propulsion plant/actuator
```

理想 wrench 基线保留，用于定位“控制算法问题”与“执行器/分配问题”。当前推进器 plant
只实现执行器与点力；其中 DPS 摆角字段是 GDA 目标，plant 保存实际摆角状态并在每个
物理子步限速。`Off` 保持实际摆角、`reset` 回中，`AppliedDps` 返回周期末实际角。
自动 allocator 仍属于调用方或后续独立模块。

## 轨迹与可视化

版本化 JSONL 的首行是 `TrajectoryHeader`。v1 header 包含格式、版本、模型、固定时序，以及 reset 后、任何动作执行前的 tick 0 `initial_snapshot`。header 还可包含调用方提供的 tick 0 期望姿态；后续每个 `TelemetryFrame` 也可附加当时的期望姿态。它们只是可选遥测，不属于 plant 状态或控制器抽象。后续每行是调用方明确选择记录的一个 `TelemetryFrame`：

```text
reset snapshot at t=0 -> TrajectoryHeader
PlantStep             -> caller-selected recorder -> TelemetryFrame
JSONL                 -> apollo-viewer
```

header 中的初始快照保证离线回放从真实 `t=0` 开始。每个 `TelemetryFrame` 保存本次动作执行后的 snapshot，以及产生该 snapshot 的 requested/applied action。

调用方可以连续记录，也可以稀疏记录。viewer 对位置、姿态和速度做视觉插值；只有紧邻已记录 step 的控制区间才能确定 action。稀疏帧之间无法恢复的 wrench 显示为 `unknown`，不会被线性插值，也不会把端点 action 错误扩展到整段。

离线回放不加载 MuJoCo。实时例程初始暂停，并手工组合 plant、控制循环和最新帧通道；这种组合只存在于应用层。

## 当前模型数据边界

当前 4932 kg 质量和 `(6332, 7953, 5879) kg·m²` 对角惯量来自 [NASA NTRS 20260000331](https://ntrs.nasa.gov/citations/20260000331) Table 1 的 “Apollo 11 actual light touchdown”列，并按项目的 X/Y/Z 轴顺序转换。

这些质量属性不等于完整飞行模型。当前后端仍是零重力、无接触、无推进剂消耗和变质量
的 freejoint 单刚体；新增推进器 plant 只把 Apollo 11 的 RCS/DPS 拓扑、脉冲/限幅和
点力耦合接到这副固定质量刚体上。APS、分级和月面环境仍未建模。

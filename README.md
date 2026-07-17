# Apollo Plant API

这是一个面向控制律、制导律和强化学习算法的 Apollo 风格被控对象。动力学由 MuJoCo 提供；Rust 与 Python 调用同一份 Rust plant 实现；Bevy 负责模型查看、离线轨迹回放以及一个可选的 live 组合例程。

公共边界是同步、显式的 plant：

```text
explicit initial state      -> spawn/reset -> snapshot
ideal body wrench OR
explicit RCS + DPS command  -> step        -> step result
```

plant 内部没有控制器、目标、奖励、episode、线程、sleep 或 viewer。仓库也不提供公共 `ClosedLoopRunner`；调用方直接写循环，自己决定控制律、记录频率、并行方式和实时节拍。

## Workspace

```text
apollo-core      状态、动作、时序、模型规格、版本化遥测
apollo-mujoco    MuJoCo plant 与 Rust API
apollo-python    同一 plant 的 PyO3 绑定
apollo-viewer    Apollo 模型查看、JSONL 离线回放与可选 live 例程
```

完整依赖方向和禁止边界见[架构说明](docs/architecture.md)。除非特别说明，下面的仓库命令都从仓库根目录执行。

## Rust API

在另一个 Rust 工程中使用本地 checkout 时，先按实际相对位置加入 path dependency：

```toml
[dependencies]
apollo-mujoco = { path = "../bevy_spaceship/crates/apollo-mujoco" }
```

若还要使用版本化轨迹 writer，再加入：

```toml
apollo-core = { path = "../bevy_spaceship/crates/apollo-core" }
```

下面是可以直接放入调用方 `src/main.rs` 的最小程序：

```rust
use apollo_mujoco::{ApolloPlantFactory, ApolloState, BodyWrench, PlantError};

fn user_algorithm(_state: ApolloState) -> BodyWrench {
    // 在这里替换为控制律、制导律或策略输出。
    BodyWrench::ZERO
}

fn main() -> Result<(), PlantError> {
    let factory = ApolloPlantFactory::apollo_touchdown()?;
    let mut plant = factory.spawn(ApolloState::ZERO)?;
    let mut snapshot = plant.snapshot();

    for _ in 0..500 {
        let action = user_algorithm(snapshot.state);
        snapshot = plant.step(action)?.snapshot;
    }

    Ok(())
}
```

每次 `step()` 恰好推进一个控制周期。默认时间基准为 2 ms 物理步长、每个动作保持 10 个物理小步，即 20 ms 控制周期。

要直接驱动 Apollo 11 LM-5 的 16 路 RCS 与 DPS，使用独立的推进器 plant：

```rust
use apollo_mujoco::{
    ApolloPropulsionPlantFactory, ApolloState, DpsCommand,
    PropulsionCommand, RcsCommand, RcsThrusterId,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let factory = ApolloPropulsionPlantFactory::apollo11_touchdown()?;
    let mut plant = factory.spawn(ApolloState::ZERO)?;
    let step = plant.step(PropulsionCommand {
        rcs: RcsCommand::single_pulse(RcsThrusterId::new(0).unwrap(), 14_000_000),
        dps: DpsCommand::Off,
    })?;
    println!("{:?}", step.applied.mean_wrench_body);
    Ok(())
}
```

理想 `ApolloPlant<BodyWrench>` 继续保留，便于把控制律问题与执行器/分配问题分开定位。
`DpsCommand` 中的摆角是 GDA 目标角：目标先限制在 6° 圆锥内，实际机构再以
`0.2°/s` 追踪。默认 20 ms 控制周期最多移动 `0.004°`；`step.applied.dps` 返回周期末
实际角。`Off` 将推力置零但保持最后摆角，`reset` 才回中，现有 2 ms × 10 时序无需改变。

仓库内的手写闭环例程源码位于
[`examples/rust/closed_loop_attitude.rs`](examples/rust/closed_loop_attitude.rs)，会生成
`runs/closed_loop_attitude.jsonl`：

```bash
source scripts/mujoco_env.zsh
cargo run -p apollo-mujoco --example closed_loop_attitude
cargo run -p apollo-mujoco --example propulsion_pulse
```

## Python API

本机直接复用现有 `cybernetic_env`，不需要新建环境。首次构建或原生代码变化后运行：

```bash
conda activate cybernetic_env
source scripts/mujoco_env.zsh
maturin develop
pytest python/tests
```

此后每个新 shell 在使用 Python plant 前仍须执行：

```bash
conda activate cybernetic_env
source scripts/mujoco_env.zsh
```

不要用 `conda run` 代替上述顺序；macOS 会过滤子进程的 `DYLD_LIBRARY_PATH`，导致扩展找不到仓库内的 MuJoCo framework。

Python 接口不是 Gym 环境。`spawn()` 要求显式初始状态；策略使用 ndarray 时，通过公开的向量转换方法接入：

```python
import numpy as np

from apollo_sim import ApolloPlantFactory, ApolloState, BodyWrench


def policy(observation: np.ndarray) -> np.ndarray:
    # 返回 [force_body_n(3), torque_about_com_body_nm(3)]。
    return np.zeros(6, dtype=np.float64)


initial_state = ApolloState.identity()
plant = ApolloPlantFactory().spawn(initial_state)
snapshot = plant.snapshot()

for _ in range(500):
    observation = snapshot.state.as_vector()
    action = BodyWrench.from_vector(policy(observation))
    snapshot = plant.step(action).snapshot
```

Python 领域对象是 frozen dataclass，向量数组以 `numpy.float64` 保存并默认设为只读；这是防止误改的 API 约定，不是不可绕过的安全边界。

完整 Python 闭环例程位于
[`examples/python/closed_loop_attitude.py`](examples/python/closed_loop_attitude.py)，会生成
`runs/python_closed_loop_attitude.jsonl`：

```bash
python examples/python/closed_loop_attitude.py
python examples/python/propulsion_pulse.py
```

## 可视化

模型查看器不依赖 MuJoCo：

```bash
cargo run -p apollo-viewer --bin apollo-model-viewer
```

它会显示四个 RCS quad、16 个独立喷管、Apollo 11 喷流挡板与 DPS 扩张喷管。水平喷流
严格沿本体 `±X/±Z`，不是 quad 局部切向；它与对角 quad 的局部径向和切向各成 45°。
12 路自由喷流避开共享简化外形件，4 路下向喷流则按实物构型有意打到导流板后向外
排走。带 MuJoCo 的交互推进 demo 使用实际 `step.applied` 驱动尾焰与 DPS 喷管摆角：

```bash
source scripts/mujoco_env.zsh
cargo run -p apollo-viewer --features live --bin apollo-propulsion-demo
```

固定视觉 QA 使用 `--capture-all` 生成 17 张图：8 张全景、4 张安装座近景、4 张喷流/
导流板近景和 1 张 DPS 近景：

```bash
cargo run -p apollo-viewer --bin apollo-model-viewer -- \
  --capture-all target/visual-qa/apollo-propulsion
```

Rust 和 Python 例程生成的轨迹都可以离线回放：

```bash
cargo run -p apollo-viewer --bin apollo-replay -- runs/closed_loop_attitude.jsonl
cargo run -p apollo-viewer --bin apollo-replay -- runs/python_closed_loop_attitude.jsonl
```

两个例程都在调用方组合姿态控制与一个限幅的质心位置/速度 PD 定点环。初态还会按
`v_origin = -omega_world x r_com_world` 设置机体系原点速度，使非零初始角速度对应
零质心速度；否则零重力下的质心会带着初始平动速度一直飞出画面。这些控制律仍然
只是例程代码，不属于 plant。Rust 可从 `factory.model_spec()`、Python 可从
`factory.model_spec` 读取质量、质心偏置和惯量，无需在外部算法中复制模型常数。

按键：`Space` 播放/暂停，`R` 回到开头，左右方向键按控制 tick 单步，上下方向键调整速度。轨迹 header 保存 reset 后的 tick 0 初始快照，因此回放从 `t=0` 开始。若调用方只稀疏记录帧，未记录控制区间的 action 无法恢复，viewer 会显示 `unknown`，不会伪造 wrench。

画面右侧的姿态坐标系使用粗实线表示调用方记录的期望姿态、细半透明线表示当前姿态。Rust/Python 记录器都允许把期望姿态作为可选遥测附加到 header 和逐 tick 帧中；旧轨迹或未提供目标的调用方不会显示粗实线，viewer 也不会拿世界系冒充目标。状态栏只使用 Bevy 默认字体可靠覆盖的 ASCII 字形。

只做无窗口格式与契约校验：

```bash
cargo run -p apollo-viewer --bin apollo-replay -- --validate-only runs/closed_loop_attitude.jsonl
cargo run -p apollo-viewer --bin apollo-replay -- --validate-only runs/python_closed_loop_attitude.jsonl
```

实时闭环只是一项应用层组合例程，不是库内 runner：

```bash
source scripts/mujoco_env.zsh
cargo run -p apollo-viewer --features live --bin apollo-live-example
```

live 例程初始处于暂停状态：`Space` 继续/暂停，`R` 重置并保持暂停，暂停时按右方向键推进一个控制 tick。

## 数据契约

公共数值统一为 `f64`。本项目采用右手坐标系；单位姿态时机体系与世界系重合，Apollo 模型的 `+X` 向右、`+Y` 向上、`+Z` 向前。`body_to_world` 是把机体系向量主动旋转到世界系的单位四元数。

| 语义 | Rust 字段 | Python / JSON 字段 | 单位或顺序 |
|---|---|---|---|
| 机体系原点的世界系位置 | `position_body_origin_world_m` | `position_body_origin_world_m` | m |
| 机体系到世界系姿态 | `body_to_world` | `quaternion_body_to_world_wxyz` | wxyz |
| 机体系原点的世界系线速度 | `linear_velocity_body_origin_world_mps` | `linear_velocity_body_origin_world_mps` | m/s |
| 机体系角速度 | `angular_velocity_body_radps` | `angular_velocity_body_radps` | rad/s |
| 在机体系表达、等效作用于质心的力 | `force_body_n` | `force_body_n` | N |
| 关于质心、在机体系表达的力矩 | `torque_about_com_body_nm` | `torque_about_com_body_nm` | N·m |

Rust `SimulationTiming` 以 `physics_step_ns` 为权威值；Python 构造时接收 `physics_step_seconds`，但同样规范化为可由整数纳秒精确表示的步长，并通过 `physics_step_ns` 暴露权威值。

Apollo 规格的质心相对机体系原点约偏移 `(0, 2.013, 0) m`，所以状态字段不会把二者简称为同一个“位置”。整数 `control_tick` 与 `physics_tick` 是权威时间源。

JSONL 第一行是 `TrajectoryHeader`，包含格式版本、模型、时序和 reset 后的 tick 0 `initial_snapshot`；后续每行是调用方明确选择记录的 `TelemetryFrame`。调用方还可在 header 和帧中附加可选期望姿态，plant 本身不知道控制目标。姿态持久字段固定为显式 `wxyz` 顺序，不继承 Rust 数学库的内部序列化顺序。

## 当前模型边界

当前动力学是 Apollo 外形和着陆工况质量属性的零重力 freejoint 单刚体。质量为 4932 kg，项目坐标轴下的对角惯量为 `(6332, 7953, 5879) kg·m²`。数据取自 [NASA NTRS 20260000331](https://ntrs.nasa.gov/citations/20260000331) 的 Table 1 “Apollo 11 actual light touchdown”列，并按本项目轴顺序转换。

它现在提供两条并行输入路径：理想六维 wrench 基线，以及具有 Apollo 11 的 16 路 RCS
拓扑、最小脉冲、点力站位、DPS 节流与 0.2°/s GDA 摆速约束的推进器 plant。后者仍使用
固定 4932 kg 质量，不包含月球重力、月面接触、推进剂消耗、变质量、APS/级间分离或
自动喷口分配器，因此不能当作完整高保真 Apollo 登月飞行模型。

## 文档与验证

- [架构与依赖边界](docs/architecture.md)
- [Rust 与 Python API 参考手册](docs/api-reference.md)
- [Apollo 11 RCS、DPS、GDA 与本项目实现详解](docs/apollo_propulsion_system.md)
- [Rust、Python、记录与回放用法](docs/usage.md)
- [可运行例程索引](examples/README.md)
- [构建、测试与开发约定](docs/development.md)
- [四元数姿态控制推导](docs/quaternion_attitude_control.md)
- [RCS 控制分配后续计划](docs/rcs_thruster_allocation_dynamics_todo.md)

Rust 验证：

```bash
cargo test -p apollo-core
cargo test -p apollo-viewer
source scripts/mujoco_env.zsh
cargo test -p apollo-mujoco
```

完整验证命令见[开发说明](docs/development.md)。

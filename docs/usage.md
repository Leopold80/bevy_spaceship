# 使用方式

本项目有两个一等调用入口：Rust API 和 Python API。两者驱动同一份 MuJoCo plant；控制律、制导律、策略、奖励、episode 和运行循环都由调用方持有。

仓库内命令默认从仓库根目录执行。macOS 下，任何需要 MuJoCo 的进程都要先在当前 shell 加载 `scripts/mujoco_env.zsh`。

## Rust：在调用方工程中直接驱动 plant

若调用方工程与本仓库相邻，在调用方 `Cargo.toml` 中加入：

```toml
[dependencies]
apollo-mujoco = { path = "../bevy_spaceship/crates/apollo-mujoco" }
```

调整 path 使其指向本 checkout。运行调用方程序前，可从任意目录用实际路径加载 MuJoCo 环境，例如：

```bash
source ../bevy_spaceship/scripts/mujoco_env.zsh
cargo run
```

`src/main.rs` 的最小完整程序：

```rust
use apollo_mujoco::{ApolloPlantFactory, ApolloState, BodyWrench, PlantError};

fn user_algorithm(_state: ApolloState) -> BodyWrench {
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

`user_algorithm` 可以替换为控制律、制导律、优化器、强化学习策略或测试输入。plant 不知道调用者属于哪一种算法。

## Python：直接驱动同一个 plant

首次构建和每个新 shell 的环境步骤见[开发说明](development.md)。`spawn()` 必须接收显式的 `ApolloState`；Python API 不会静默选择初始条件。

下面演示 ndarray 策略与具名 plant API 的边界：

```python
import numpy as np

from apollo_sim import ApolloPlantFactory, ApolloState, BodyWrench


def policy(observation: np.ndarray) -> np.ndarray:
    return np.zeros(6, dtype=np.float64)


initial_state = ApolloState.identity()
factory = ApolloPlantFactory()
plant = factory.spawn(initial_state)
snapshot = plant.snapshot()

for _ in range(500):
    observation = snapshot.state.as_vector()
    action = BodyWrench.from_vector(policy(observation))
    result = plant.step(action)
    snapshot = result.snapshot
```

`ApolloState.as_vector()` 的顺序是位置 3、wxyz 姿态 4、线速度 3、角速度 3；`BodyWrench.from_vector()` 接收力 3、力矩 3。`step()` 不接收裸 ndarray，这个显式转换正是后续 RL task adapter 应放置的位置。

Python 领域对象是 frozen dataclass，数组为 `numpy.float64` 且默认只读。该设计用于尽早发现意外修改，不应理解为安全隔离；拥有底层数组的调用方仍可主动改变 NumPy write flag。

## 自己实现闭环

闭环只是普通调用方循环：

```text
read snapshot
    -> compute reference if needed
    -> compute BodyWrench
    -> plant.step(wrench)
    -> repeat
```

仓库中的 Rust/Python 控制器都只存在于根目录的
[例程区](../examples/README.md)，不是公共库 API。运行两个例程：

```bash
source scripts/mujoco_env.zsh
cargo run -p apollo-mujoco --example closed_loop_attitude

conda activate cybernetic_env
source scripts/mujoco_env.zsh
python examples/python/closed_loop_attitude.py
```

例程在姿态环之外组合了一个低增益、限加速度的质心位置/速度 PD 环。它先用模型的
质心偏置把“机体系原点状态”换算为“质心状态”，在世界系计算定点合力，再旋转回
plant 要求的机体系表达。不要直接把非零角速度与零原点线速度组合为“静止”初态：

```text
p_com = p_origin + R_body_to_world r_com_body
v_com = v_origin + omega_world x (R_body_to_world r_com_body)
```

若希望初始质心静止，应设置
`v_origin = -omega_world x (R_body_to_world r_com_body)`。Rust 从
`factory.model_spec()`、Python 从 `factory.model_spec` 取得质量和质心偏置；位置环与
这个初态换算都只属于调用方例程。

它们分别生成：

```text
runs/closed_loop_attitude.jsonl
runs/python_closed_loop_attitude.jsonl
```

## 多实例

一个 `ApolloPlantFactory` 可以生成多个 plant。它们共享只读 MuJoCo model，每个实例拥有独立的 MuJoCo data、状态和 tick：

```rust
let plants = initial_states
    .into_iter()
    .map(|state| factory.spawn(state))
    .collect::<Result<Vec<_>, _>>()?;
```

第一版不提供 batch/vector API。顺序执行或进程级并行由调用方决定；不要让多个执行单元并发修改同一个 plant。

当前 PyO3 `ApolloPlant` 和 factory 是线程绑定对象，且 `step()` 不释放 GIL。Python 多实例训练应先采用“每个进程独立创建 factory/plant”，不要把同一对象在线程间传递。Rust 调度同样属于调用方应用层，而不属于 plant 内部 runner。

## Rust：显式记录轨迹

记录不是 `step()` 的隐式副作用。外部 Rust 工程需要同时依赖：

```toml
[dependencies]
apollo-core = { path = "../bevy_spaceship/crates/apollo-core" }
apollo-mujoco = { path = "../bevy_spaceship/crates/apollo-mujoco" }
```

下面的完整程序创建 `runs/`、保存 reset 后的 tick 0 初始快照，并逐步记录：

```rust
use apollo_core::{JsonlTrajectoryWriter, TelemetryFrame, TrajectoryHeader};
use apollo_mujoco::{ApolloPlantFactory, ApolloState, BodyWrench};
use std::error::Error;
use std::fs::{File, create_dir_all};
use std::io::{BufWriter, Write};

fn main() -> Result<(), Box<dyn Error>> {
    let factory = ApolloPlantFactory::apollo_touchdown()?;
    let mut plant = factory.spawn(ApolloState::ZERO)?;
    let initial_snapshot = plant.snapshot();
    let desired_attitude = ApolloState::ZERO.body_to_world;

    create_dir_all("runs")?;
    let output = BufWriter::new(File::create("runs/my_run.jsonl")?);
    let header = TrajectoryHeader::apollo(plant.timing(), initial_snapshot)
        .with_initial_desired_attitude(desired_attitude);
    let mut writer = JsonlTrajectoryWriter::new(output, header)?;

    for _ in 0..500 {
        let step = plant.step(BodyWrench::ZERO)?;
        writer.write_frame(
            &TelemetryFrame::from(step).with_desired_attitude(desired_attitude),
        )?;
    }
    writer.get_mut().flush()?;

    Ok(())
}
```

`TrajectoryHeader` v1 必须包含 reset 后、任何动作执行前的 tick 0 `initial_snapshot`。viewer 因而从 `t=0` 显示真实初态，而不是从第一个动作后的状态开始。`initial_attitude_reference` 与每帧的 `attitude_reference` 都是调用方可选遥测，不会进入 plant API；固定目标应在 header 和每帧都写入，时变目标则逐帧写入当时的期望姿态。

## Python：显式记录轨迹

Python writer 使用同一 schema，并要求显式传入初始快照和 plant 时序：

```python
from pathlib import Path

from apollo_sim import (
    ApolloPlantFactory,
    ApolloState,
    BodyWrench,
    JsonlTrajectoryWriter,
)

initial_state = ApolloState.identity()
plant = ApolloPlantFactory().spawn(initial_state)
initial_snapshot = plant.snapshot()
desired_wxyz = [1.0, 0.0, 0.0, 0.0]

path = Path("runs/my_python_run.jsonl")
path.parent.mkdir(parents=True, exist_ok=True)
with path.open("w", encoding="utf-8") as stream:
    writer = JsonlTrajectoryWriter(
        stream,
        initial_snapshot,
        plant.timing,
        initial_desired_attitude_wxyz=desired_wxyz,
    )
    for _ in range(500):
        step = plant.step(BodyWrench.zero())
        writer.write_step(step, desired_attitude_wxyz=desired_wxyz)
```

两种 writer 都只记录调用方明确提交的 step。连续记录时，viewer 可以显示每个控制区间的 requested/applied wrench。若只记录每隔若干 tick 的稀疏帧，帧间状态会用于视觉插值，但中间 action 无法从端点恢复；viewer 会显示 `unknown`，不会把某个已记录 action 错误延长到整段。

## 离线校验与回放

两种语言的轨迹都可交给同一个 viewer：

```bash
cargo run -p apollo-viewer --bin apollo-replay -- runs/closed_loop_attitude.jsonl
cargo run -p apollo-viewer --bin apollo-replay -- runs/python_closed_loop_attitude.jsonl
```

回放画面右侧叠加期望姿态与当前姿态：粗实线为调用方记录的期望姿态，细半透明线为 plant 当前姿态。没有记录目标的旧轨迹仍可读取，但粗实线会隐藏，状态栏显示 `desired attitude unavailable`。

CI 或跨语言契约检查无需打开窗口：

```bash
cargo run -p apollo-viewer --bin apollo-replay -- --validate-only runs/closed_loop_attitude.jsonl
cargo run -p apollo-viewer --bin apollo-replay -- --validate-only runs/python_closed_loop_attitude.jsonl
```

## Live 组合例程

```bash
source scripts/mujoco_env.zsh
cargo run -p apollo-viewer --features live --bin apollo-live-example
```

live 例程启动后默认暂停，确保窗口出现时仍显示显式初态。`Space` 继续或暂停；`R` reset 并保持暂停；暂停时右方向键只推进一个控制 tick。线程、wall-clock 节拍和控制器都只属于这个二进制例程，没有进入 plant API。

## 后续 Gym 适配

未来 Gymnasium 包装器位于纯 Python 层：

```text
ApolloPlant
  -> action adapter
  -> observation encoder
  -> reward / terminated / truncated
  -> Gymnasium Env
```

增加 Gym task 不应修改 Rust plant 的状态、动作或固定步进契约。

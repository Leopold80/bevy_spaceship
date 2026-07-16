# Apollo Sim Python API

`apollo_sim` 是 Rust/MuJoCo plant 的薄 Python 接口。它只负责构造、重置、读取状态和施加机体系 wrench；控制律、制导律、强化学习 task、奖励和可视化均由调用方组合。

## 开发安装与运行环境

本机直接复用现有 `cybernetic_env`，不需要新建环境。以下命令从仓库根目录执行。

首次构建或原生代码变化后：

```bash
conda activate cybernetic_env
source scripts/mujoco_env.zsh
maturin develop
pytest python/tests
```

每个新 shell 在使用 Python plant 前仍要执行：

```bash
conda activate cybernetic_env
source scripts/mujoco_env.zsh
```

`maturin develop` 不必每次运行，但 MuJoCo 动态库环境必须每个 shell 重新加载。不要用 `conda run` 代替：macOS 会过滤子进程的 `DYLD_LIBRARY_PATH`，使扩展找不到仓库内的 MuJoCo framework。

## 最小调用

`spawn()` 必须接收显式初始状态：

```python
from apollo_sim import ApolloPlantFactory, ApolloState, BodyWrench

initial_state = ApolloState.identity()
factory = ApolloPlantFactory()
plant = factory.spawn(initial_state)

snapshot = plant.snapshot()
for _ in range(100):
    action = BodyWrench.zero()
    snapshot = plant.step(action).snapshot
```

`step()` 恰好推进一个控制周期，不创建线程，也不按照 wall clock 等待。当前 API 不包含 Gym、reward、episode 或闭环 runner。
`factory.model_spec` 提供只读的 `mass_kg`、`center_of_mass_body_m` 和
`diagonal_inertia_body_kg_m2`，供调用方控制律和状态换算使用。

## ndarray 策略适配

学习算法通常处理扁平 ndarray；plant API 使用具名对象。边界应显式写成：

```python
import numpy as np

from apollo_sim import ApolloPlantFactory, ApolloState, BodyWrench


def policy(observation: np.ndarray) -> np.ndarray:
    return np.zeros(6, dtype=np.float64)


plant = ApolloPlantFactory().spawn(ApolloState.identity())
snapshot = plant.snapshot()

for _ in range(100):
    observation = snapshot.state.as_vector()
    action = BodyWrench.from_vector(policy(observation))
    snapshot = plant.step(action).snapshot
```

裸 ndarray 不能直接传给 `step()`。当前扁平顺序是：

```text
state = [position_body_origin_world_m(3), quaternion_body_to_world_wxyz(4),
         linear_velocity_body_origin_world_mps(3), angular_velocity_body_radps(3)]

wrench = [force_body_n(3), torque_about_com_body_nm(3)]
```

公共向量统一使用 `numpy.float64`。领域对象是 frozen dataclass，数组默认只读；调用方若主动改变 NumPy write flag 仍可修改底层数组，所以这里的“只读”是防误用约定，不是安全边界。

## 坐标和时序

本项目使用右手坐标系；单位姿态时机体系与世界系重合，Apollo 模型的 `+X` 向右、`+Y` 向上、`+Z` 向前。

- `position_body_origin_world_m` 和 `linear_velocity_body_origin_world_mps` 描述机体系原点，而不是质心。
- `quaternion_body_to_world_wxyz` 把机体系向量旋转到世界系，顺序始终为 wxyz。
- `angular_velocity_body_radps` 在机体系表达。
- `force_body_n` 在机体系表达，等效作用于质心。
- `torque_about_com_body_nm` 是关于质心、在机体系表达的力矩。

非零角速度时，零机体系原点速度不代表质心静止。若 `r_com_world` 是由姿态旋转后的
质心偏置，则 `v_com = v_origin + omega_world x r_com_world`。仓库的 Python 闭环例程
使用这个关系构造零质心速度初态，并在调用方增加一个限幅的质心定点 PD 环。

Python `SimulationTiming` 构造时接收 `physics_step_seconds`，但物理步长必须能由正整数纳秒表示。`physics_step_ns` 是与 Rust/JSONL 一致的权威整数值；默认值为 2,000,000 ns，`substeps_per_control` 默认为 10。

## 显式记录与回放

writer 必须接收 reset 后、任何动作执行前的 tick 0 初始快照，以及实际 plant 时序：

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
    for _ in range(100):
        step = plant.step(BodyWrench.zero())
        writer.write_step(step, desired_attitude_wxyz=desired_wxyz)
```

初始快照写在 JSONL header 中，回放因此从 `t=0` 开始。期望姿态是调用方可选遥测：提供时粗实线显示期望姿态、细半透明线显示当前姿态；不提供时 viewer 隐藏粗实线。writer 只记录调用方明确提交的 step；若调用方稀疏记录，未记录控制区间的 action 不可恢复，viewer 会显示 `unknown`。

完整 Python 控制例程位于仓库根目录的
[`examples/python/closed_loop_attitude.py`](../examples/python/closed_loop_attitude.py)。
逐项接口说明见 [API 参考手册](../docs/api-reference.md)。运行及回放：

```bash
python examples/python/closed_loop_attitude.py
cargo run -p apollo-viewer --bin apollo-replay -- runs/python_closed_loop_attitude.jsonl
cargo run -p apollo-viewer --bin apollo-replay -- --validate-only runs/python_closed_loop_attitude.jsonl
```

## 多实例与线程限制

当前原生 `ApolloPlantFactory` 和 `ApolloPlant` 是线程受限对象，`step()` 也不释放 GIL。不要把同一个实例跨 Python 线程传递。并行采样优先让每个 worker 进程在其内部创建并使用自己的 factory/plant；第一版不提供 batch/vector API。

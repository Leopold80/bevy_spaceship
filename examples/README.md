# 例程

这里集中放置可直接运行、可复制修改的调用方例程。控制器、目标、闭环循环和记录逻辑
都只属于例程，不会进入 plant 公共 API。

## Rust

源码：

- [rust/closed_loop_attitude.rs](rust/closed_loop_attitude.rs)：理想 wrench 闭环与 v1 轨迹；
- [rust/propulsion_pulse.rs](rust/propulsion_pulse.rs)：RCS 最小脉冲与 DPS 可调档。

```bash
source scripts/mujoco_env.zsh
cargo run -p apollo-mujoco --example closed_loop_attitude
cargo run -p apollo-mujoco --example propulsion_pulse
```

例程保留非零初始角速度，在调用方组合级联姿态控制与质心位置/速度 PD 控制，输出
`runs/closed_loop_attitude.jsonl`。

## Python

源码：

- [python/closed_loop_attitude.py](python/closed_loop_attitude.py)：理想 wrench 闭环；
- [python/propulsion_pulse.py](python/propulsion_pulse.py)：与 Rust 相同的推进接口链路。

```bash
conda activate cybernetic_env
source scripts/mujoco_env.zsh
python examples/python/closed_loop_attitude.py
python examples/python/propulsion_pulse.py
```

例程使用同一 Rust/MuJoCo plant，在普通 Python 循环中组合姿态 PD 与质心定点控制，
输出 `runs/python_closed_loop_attitude.jsonl`。

两个 `propulsion_pulse` 例程直接提交喷口命令，不包含控制器或自动分配器；它们演示
requested/applied 的区别，也不会把尚未定义的推进遥测写入 JSONL v1。DPS 摆角请求是
GDA 目标角；目标先限制在 6° 圆锥内，实际机构再以 `0.2°/s` 追踪。默认 20 ms 周期
最多移动 `0.004°`，因此例程调用方应读取 `step.applied.dps` 的周期末实际角，而不能
把原始命令当成已到达位置。`Off` 保持最后摆角，`reset` 才回中。

## Viewer 应用例程

交互推进 demo 与固定视觉 QA 也可直接运行：

```bash
source scripts/mujoco_env.zsh
cargo run -p apollo-viewer --features live --bin apollo-propulsion-demo
cargo run -p apollo-viewer --bin apollo-model-viewer -- \
  --capture-all target/visual-qa/apollo-propulsion
```

交互 demo 从 `AppliedDps` 读取周期末实际 GDA 角。截图命令固定生成 8 张全景、4 张安装座、
4 张喷流/导流板与 1 张 DPS，共 17 张；它不运行控制器或修改 plant。

## 回放

```bash
cargo run -p apollo-viewer --bin apollo-replay -- runs/closed_loop_attitude.jsonl
cargo run -p apollo-viewer --bin apollo-replay -- runs/python_closed_loop_attitude.jsonl
```

回放中的粗实线坐标系是例程写入遥测的期望姿态，细半透明坐标系是当前姿态。API 的
逐项说明见 [API 参考手册](../docs/api-reference.md)，组合模式见
[使用方式](../docs/usage.md)。

# 例程

这里集中放置可直接运行、可复制修改的调用方例程。控制器、目标、闭环循环和记录逻辑
都只属于例程，不会进入 plant 公共 API。

## Rust

源码：[rust/closed_loop_attitude.rs](rust/closed_loop_attitude.rs)

```bash
source scripts/mujoco_env.zsh
cargo run -p apollo-mujoco --example closed_loop_attitude
```

例程保留非零初始角速度，在调用方组合级联姿态控制与质心位置/速度 PD 控制，输出
`runs/closed_loop_attitude.jsonl`。

## Python

源码：[python/closed_loop_attitude.py](python/closed_loop_attitude.py)

```bash
conda activate cybernetic_env
source scripts/mujoco_env.zsh
python examples/python/closed_loop_attitude.py
```

例程使用同一 Rust/MuJoCo plant，在普通 Python 循环中组合姿态 PD 与质心定点控制，
输出 `runs/python_closed_loop_attitude.jsonl`。

## 回放

```bash
cargo run -p apollo-viewer --bin apollo-replay -- runs/closed_loop_attitude.jsonl
cargo run -p apollo-viewer --bin apollo-replay -- runs/python_closed_loop_attitude.jsonl
```

回放中的粗实线坐标系是例程写入遥测的期望姿态，细半透明坐标系是当前姿态。API 的
逐项说明见 [API 参考手册](../docs/api-reference.md)，组合模式见
[使用方式](../docs/usage.md)。

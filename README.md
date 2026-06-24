# Bevy Spacecraft

基于 Bevy 的 Apollo 风格登月舱可视化，以及四元数姿态控制实验。

## 分支

- `main`：基础的 Apollo 风格登月舱场景。
- `experiment/quaternion-attitude-control`：运动学四元数姿态控制演示，包含日志记录和可视化验证。

## 四元数姿态控制演示

实验分支当前代码用于验证如下简化的运动学外环控制律：

```text
q_e = q_d^-1 * q
if q_e0 < 0, q_e = -q_e
omega_c = -kp * q_e0 * q_ev
```

该实现有意只建模运动学部分。它不包含刚体动力学、转动惯量、执行器力矩、饱和约束，也不包含内层角速度控制环。

技术文档中同时整理了 `q_e0 q_ev` 型反馈证明，以及作为工程对照的常增益 `q_ev` 型反馈证明。可视化演示支持在两种控制律之间切换。

## 运行

```bash
cargo run
```

可视化演示中的控制按键：

- `Space` 或 `R`：重置当前场景。
- `1`、`2`、`3`：在几个可重复的初始姿态场景之间切换。
- `C`：在 `omega_c = -kp * q_e0 * q_ev` 与固定增益 `omega_c = -kp * q_ev` 两种控制律之间切换。
- `P`：暂停或继续收敛过程。

右侧叠加显示区域会在同一个原点处显示两个坐标系：

- 粗坐标系：期望姿态 `q_d`。
- 半透明细坐标系：当前姿态 `q`。

随着控制器收敛，当前坐标系应逐渐与期望坐标系重合。

## 无图形界面的日志验证

当无法使用 GPU 渲染时，可以使用无图形界面模式。该模式默认记录 `q_e0 q_ev` 型反馈，作为与早期实验一致的基准：

```bash
cargo run -- --headless-log
```

该命令会写入：

```text
logs/attitude_kinematics.csv
```

期望的变化趋势：

- `qe0 >= 0`，说明实现了 unwinding 避免机制。
- `qev_norm` 逐渐减小并趋近于零。
- `error_angle_rad` 逐渐减小并趋近于零。
- `omega_norm` 随着误差缩小而减小。

## 理论说明

修正后的推导保存在：

```text
docs/quaternion_attitude_control.md
```

与 Bevy 无关的控制代码位于：

```text
src/attitude_control.rs
```

场景、控制按键、HUD、日志记录和坐标系可视化位于：

```text
src/main.rs
```

## 检查

```bash
cargo fmt --check
cargo check
cargo test
```

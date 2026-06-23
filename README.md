# Bevy Spacecraft

基于 Bevy 的 Apollo 风格登月舱基础可视化场景。

## 分支

- `main`：基础的 Apollo 风格登月舱场景。
- `experiment/quaternion-attitude-control`：四元数姿态控制实验分支，包含修正后的理论说明、可视化坐标系叠加显示，以及 CSV 日志验证。

## 运行基础场景

```bash
cargo run
```

## 实验分支

切换到实验分支以运行运动学四元数姿态控制演示：

```bash
git switch experiment/quaternion-attitude-control
cargo run
```

该实验验证如下控制律：

```text
q_e = q_d^-1 * q
if q_e0 < 0, q_e = -q_e
omega_c = -kp * q_e0 * q_ev
```

实验分支还包含无图形界面的 CSV 日志模式：

```bash
cargo run -- --headless-log
```

## 检查

```bash
cargo fmt --check
cargo check
cargo test
```

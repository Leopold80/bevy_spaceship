# Bevy Spacecraft

基于 Bevy 的飞船强化学习/控制系统仿真可视化环境。当前版本包含 Apollo 风格登月舱资源、Starship-inspired 不锈钢火箭外形资源、MuJoCo Apollo 6DoF 动力学实验、四元数姿态误差计算、运动学姿态控制律、日志验证和可视化演示。

工程目标是把算法数学与 Bevy 场景细节分开：控制律和误差计算应能被控制/RL 实验直接调用，飞船造型资源应能单独查看和维护，演示程序则负责把二者组合成可视化验证环境。

## 运行入口

只查看飞船模型和基础场景，当前会并排展示 Apollo 风格登月舱和 Starship-inspired 火箭模型：

```bash
cargo run --bin model_viewer
```

运行四元数姿态控制可视化演示：

```bash
cargo run --bin attitude_demo
```

运行 MuJoCo Apollo 6DoF 动力学演示：

```bash
source scripts/mujoco_env.zsh
cargo run --bin mujoco_apollo_demo
```

默认入口仍然指向姿态控制演示，因此下面的旧命令也可用：

```bash
cargo run
```

## 四元数姿态控制演示

姿态控制演示用于验证如下简化的运动学外环控制律：

```text
q_e = q_d^-1 * q
if q_e0 < 0, q_e = -q_e
omega_c = -kp * q_e0 * q_ev
```

该实现有意只建模运动学部分。它不包含刚体动力学、转动惯量、执行器力矩、饱和约束，也不包含内层角速度控制环。

技术文档中同时整理了 `q_e0 q_ev` 型反馈证明，以及作为工程对照的常增益 `q_ev` 型反馈证明。可视化演示支持在两种控制律之间切换。

可视化演示中的控制按键：

- `Space` 或 `R`：重置当前场景。
- `1`、`2`、`3`：在几个可重复的初始姿态场景之间切换。
- `C`：在 `omega_c = -kp * q_e0 * q_ev` 与固定增益 `omega_c = -kp * q_ev` 两种控制律之间切换。
- `P`：暂停或继续收敛过程。

右侧叠加显示区域会在同一个原点处显示两个坐标系：

- 粗坐标系：期望姿态 `q_d`。
- 半透明细坐标系：当前姿态 `q`。

随着控制器收敛，当前坐标系应逐渐与期望坐标系重合。

## MuJoCo Apollo 动力学实验

MuJoCo 实验使用 `mujoco-rs`，macOS 下按 MuJoCo-rs 文档要求启用了 `renderer-winit-fallback`，并在 `Cargo.toml` 中 patch 了 `glutin`。MuJoCo 官方动态库不提交进仓库，推荐放在：

```text
.local/mujoco/3.9.0/macos/
```

目录结构应包含：

```text
.local/mujoco/3.9.0/macos/mujoco.framework/
.local/mujoco/3.9.0/macos/libmujoco.dylib
.local/mujoco/3.9.0/macos/libmujoco.3.9.0.dylib
```

其中 `libmujoco.dylib` 指向：

```text
mujoco.framework/Versions/A/libmujoco.3.9.0.dylib
```

从一个已下载好的 macOS framework 配置本仓库：

```bash
mkdir -p .local/mujoco/3.9.0/macos
mv mujoco.framework .local/mujoco/3.9.0/macos/
ln -sfn mujoco.framework/Versions/A/libmujoco.3.9.0.dylib .local/mujoco/3.9.0/macos/libmujoco.dylib
ln -sfn mujoco.framework/Versions/A/libmujoco.3.9.0.dylib .local/mujoco/3.9.0/macos/libmujoco.3.9.0.dylib
xattr -dr com.apple.quarantine .local/mujoco/3.9.0/macos/mujoco.framework
codesign --force --deep --sign - .local/mujoco/3.9.0/macos/mujoco.framework
source scripts/mujoco_env.zsh
```

Apollo 模型现在以 `src/apollo_spec.rs` 中的 Rust 部件规格作为单一来源。Bevy 的 Apollo 可视化模型和 MuJoCo MJCF 都由这份规格生成，避免维护两套尺寸和坐标。第一版 MuJoCo 模型使用零重力、一个 freejoint 刚体，以及 body-frame 6D 外力/力矩输入。

## 无图形界面的日志验证

当无法使用 GPU 渲染时，可以使用无图形界面模式。该模式默认记录 `q_e0 q_ev` 型反馈，作为与早期实验一致的基准：

```bash
cargo run --bin attitude_demo -- --headless-log
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

## 代码结构

面向算法和工程维护的主要入口：

- `src/attitude_control.rs`：与 Bevy 场景无关的控制律、四元数误差、误差角、积分函数和测试。
- `src/apollo_spec.rs`：Apollo 登月舱的统一部件规格，并生成 MuJoCo MJCF。
- `src/mujoco_dynamics.rs`：MuJoCo Apollo 6DoF 动力学封装、状态读取和外力/力矩输入。
- `src/spacecraft_model.rs`：Apollo 风格登月舱与 Starship-inspired 火箭的几何造型、材质和 `spawn_lander` / `spawn_starship` 入口；Apollo 视觉模型由 `apollo_spec` 生成。
- `src/visualization.rs`：相机、光照、星空、目标/当前坐标系等可视化工具。
- `src/attitude_log.rs`：CSV 日志和无图形界面验证。
- `src/attitude_demo.rs`：姿态控制演示的 Bevy 系统、按键、HUD 和场景组合。
- `src/bin/model_viewer.rs`：只展示模型的可执行入口。
- `src/bin/mujoco_apollo_demo.rs`：MuJoCo Apollo 6DoF 动力学和 Bevy 可视化绑定入口。
- `src/bin/attitude_demo.rs`：姿态控制演示的显式可执行入口。

## 理论说明

修正后的推导保存在：

```text
docs/quaternion_attitude_control.md
```

## 检查

```bash
cargo fmt --check
source scripts/mujoco_env.zsh
cargo check --bins
cargo test
cargo run --bin attitude_demo -- --headless-log
```

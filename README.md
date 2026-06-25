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

仓库的 `.vscode/settings.json` 已为 rust-analyzer 配置同一组 MuJoCo 环境变量，因此 VS Code 内部的 `cargo check` / `cargo clippy` 不需要额外手动 source 脚本。终端里直接运行 MuJoCo demo 或测试时仍建议先 source 上面的脚本。

Apollo 模型现在以 `src/apollo_spec.rs` 中的 Rust 部件规格作为单一来源。Bevy 的 Apollo 可视化模型和 MuJoCo MJCF 都由这份规格生成，避免维护两套尺寸和坐标。MuJoCo 模型使用零重力、一个 freejoint 刚体，以及 body-frame 6D 外力/力矩输入。

当前 MuJoCo Apollo demo 默认运行双层姿态控制：

```text
outer loop: q_e -> omega_c                 # 复用四元数运动学姿态控制律
inner loop: omega_c - omega_body -> tau    # PID 角速度环输出 body-frame 力矩
plant:      J * omega_dot + omega x Jomega = tau, integrated by MuJoCo
```

也就是说，PID 不再直接改运动学姿态积分，而是作为内层动力学控制器向 MuJoCo freejoint 刚体施加力矩。`R` 会重置 MuJoCo 状态和 PID 积分/微分历史。

## 动力学 TODO 索引

- MuJoCo/控制步长解耦（见 TODO section）
- RCS 喷口控制分配：docs/rcs_thruster_allocation_dynamics_todo.md

## TODO：将 MuJoCo 仿真/控制步长与 Bevy 渲染帧时间解耦

当前的 MuJoCo-Bevy 联合方式主要面向实时可视化 demo。在 `mujoco_apollo_demo` 中，MuJoCo 的仿真推进依赖 Bevy 每一帧的 `delta time`，随后再把 MuJoCo 状态同步到 Bevy 的可视模型上。这样做适合快速演示，因为屏幕上的运动大致跟随真实时间。

但是，这种设计不适合严格的控制算法验证。

对于控制系统仿真，被控对象应当具有明确、固定的离散时间推进方式：

```text
x[k+1] = F(x[k], u[k], Δt)
```

其中仿真步长或控制步长 `Δt` 应该由控制实验本身定义，而不应该由渲染帧率决定。控制算法的效果不应依赖 GPU 负载、窗口刷新率、操作系统调度或临时卡顿。

后续应当区分三种时间尺度：

```text
1. MuJoCo 仿真步长
2. 控制器更新步长
3. Bevy 渲染帧时间
```

例如：

```text
MuJoCo simulation dt:   0.002 s
Controller dt:          0.020 s
Control hold:           10 steps
```

在严格控制实验模式下，Bevy 不应该驱动物理时间，仅负责显示 MuJoCo 状态。

推荐结构：

```text
src/mujoco_dynamics.rs
src/control_env.rs
src/bin/control_headless_demo.rs
src/bin/control_visual_demo.rs
```

## 无图形界面日志验证

当无法使用 GPU 时使用 headless 模式记录控制效果。

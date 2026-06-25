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

## TODO：将 MuJoCo 仿真/控制步长与 Bevy 渲染帧时间解耦

当前的 MuJoCo-Bevy 联合方式主要面向实时可视化 demo。在 `mujoco_apollo_demo` 中，MuJoCo 的仿真推进依赖 Bevy 每一帧的 `delta time`，随后再把 MuJoCo 状态同步到 Bevy 的可视模型上。这样做适合快速演示，因为屏幕上的运动大致跟随真实时间。

但是，这种设计不适合严格的控制算法验证。

对于控制系统仿真，被控对象应当具有明确、固定的离散时间推进方式：

```text
x[k+1] = F(x[k], u[k], Δt)
```

其中仿真步长或控制步长 `Δt` 应该由控制实验本身定义，而不应该由渲染帧率决定。控制算法的效果不应依赖 GPU 负载、窗口刷新率、操作系统调度或临时卡顿。

如果直接用 Bevy 的帧时间推进 MuJoCo，那么同一个控制输入在不同机器、不同帧率，甚至同一次运行中的不同帧上，都会作用不同的物理时间。这会导致控制实验的可重复性下降，并且把被控对象动力学错误地耦合到可视化后端上。

后续应当区分三种时间尺度：

```text
1. MuJoCo 仿真步长
   较小且固定的物理积分步长，例如 0.001 s 或 0.002 s。

2. 控制器更新步长
   固定的控制输入更新周期，例如 0.01 s 或 0.02 s。

3. Bevy 渲染帧时间
   可变的显示刷新时间，只用于画面更新，不参与定义物理系统。
```

例如，后续控制实验可以采用：

```text
MuJoCo simulation dt:   0.002 s
Controller dt:          0.020 s
Control hold:           10 个 MuJoCo 小步 / 1 个控制周期
```

也就是说，每次控制器更新控制输入 `u[k]` 后，应当在固定数量的 MuJoCo 小步内保持该输入：

```text
state = read_mujoco_state()
control = controller(state, reference)

for _ in 0..control_hold:
    apply_control(control)
    mujoco_step_fixed_dt()

next_state = read_mujoco_state()
```

在严格控制实验模式下，Bevy 不应该驱动物理时间。Bevy 应该只读取 MuJoCo 的最新状态，并更新飞行器可视模型的 `Transform`。

推荐后续结构：

```text
src/mujoco_dynamics.rs
    MuJoCo 底层动力学封装。
    负责 reset、固定步长积分、施加外力/力矩或执行器输入、
    读取位置、姿态、速度和角速度等状态。

src/control_env.rs
    控制实验环境封装。
    负责固定控制步长、控制输入保持、参考信号生成、
    状态记录、误差计算和实验 reset。

src/bin/mujoco_apollo_demo.rs
    实时 Bevy 可视化 demo。
    可以继续使用 wall-clock time，用于交互式演示和模型观察。

src/bin/control_headless_demo.rs
    无窗口固定步长控制实验。
    不依赖 Bevy，用来验证控制算法、记录数据和检查可重复性。

src/bin/control_visual_demo.rs
    可选的控制可视化模式。
    固定步长控制实验作为状态源，Bevy 只负责同步显示。
```

简而言之：当前由 Bevy 帧时间驱动 MuJoCo 的写法适合作为可视化 demo，但不适合作为严格控制实验的时间基准。后续应当将渲染循环与物理/控制循环解耦，让 MuJoCo 和控制器按照固定步长推进，Bevy 仅作为可视化观察器。

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

- `src/attitude_control.rs`：与 Bevy 场景无关的控制律、四元数误差、误差角、积分函数和测试；内部使用 `glam` 数学类型。
- `src/apollo_spec.rs`：Apollo 登月舱的统一部件规格，并生成 MuJoCo MJCF。
- `src/control_law.rs`：与 Bevy 场景无关的控制器接口，以及“外层四元数运动学 + 内层角速度 PID 力矩”的双层姿态控制器。
- `src/mujoco_dynamics.rs`：MuJoCo Apollo 6DoF 动力学封装、状态读取和外力/力矩输入；不依赖 Bevy 显示类型。
- `src/control_env.rs`：固定控制周期环境，负责控制器更新、MuJoCo 小步保持、reset 和 snapshot；不依赖 Bevy。
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

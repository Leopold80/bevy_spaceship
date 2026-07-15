# Bevy Spacecraft

基于 Bevy 的飞船强化学习/控制系统仿真可视化环境。当前版本包含 Apollo 风格登月舱资源、零重力 Apollo 外形单刚体 6DoF 姿态控制实验、四元数姿态误差计算、运动学姿态控制律、日志验证和可视化演示。

工程目标是把算法数学与 Bevy 场景细节分开：控制律和误差计算应能被控制/RL 实验直接调用，飞船造型资源应能单独查看和维护，演示程序则负责把二者组合成可视化验证环境。

## 运行入口

项目通过 `rust-toolchain.toml` 固定使用 Rust 1.95.0。当前 `mujoco-rs` 是无条件依赖，因此即使只运行 Bevy 模型查看器或运动学姿态演示，在终端执行 Cargo 命令前也需要从仓库根目录加载 MuJoCo 链接环境。

只查看 Apollo 风格登月舱模型和基础场景：

```bash
source scripts/mujoco_env.zsh
cargo run --bin model_viewer
```

运行四元数姿态控制可视化演示：

```bash
source scripts/mujoco_env.zsh
cargo run --bin attitude_demo
```

运行 MuJoCo Apollo 外形单刚体 6DoF 姿态控制演示：

```bash
source scripts/mujoco_env.zsh
cargo run --bin mujoco_apollo_demo
```

默认入口仍然指向姿态控制演示，因此下面的旧命令也可用：

```bash
source scripts/mujoco_env.zsh
cargo run
```

## 四元数姿态控制演示

姿态控制演示用于验证如下简化的运动学外环控制律：

```text
q_e = q_d^-1 * q
if q_e0 < 0, q_e = -q_e
omega_c = -kp * q_ev
```

该实现有意只建模运动学部分。它不包含刚体动力学、转动惯量、执行器力矩、饱和约束，也不包含内层角速度控制环。

当前运动学演示和 MuJoCo 级联控制器都只使用常增益
`omega_c = -kp * q_ev` 反馈。`q_e0 q_ev` 缩放反馈仅保留在
`attitude_control::legacy` 中作为理论和回归对照，不再进入任何运行时控制路径。

可视化演示中的控制按键：

- `Space` 或 `R`：重置当前场景。
- `1`、`2`、`3`：在几个可重复的初始姿态场景之间切换。
- `P`：暂停或继续收敛过程。

右侧叠加显示区域会在同一个原点处显示两个坐标系：

- 粗坐标系：期望姿态 `q_d`。
- 半透明细坐标系：当前姿态 `q`。

随着控制器收敛，当前坐标系应逐渐与期望坐标系重合。

## MuJoCo Apollo 外形单刚体实验

MuJoCo 实验使用 `mujoco-rs`，macOS 下启用了 `renderer-winit-fallback`，并在 `Cargo.toml` 中 patch 了 `glutin`。当前仓库已跟踪 MuJoCo 3.9.0 的 macOS framework 和动态库，位置为：

```text
.local/mujoco/3.9.0/macos/
```

目录结构包含：

```text
.local/mujoco/3.9.0/macos/mujoco.framework/
.local/mujoco/3.9.0/macos/libmujoco.dylib
.local/mujoco/3.9.0/macos/libmujoco.3.9.0.dylib
```

其中两个 dylib 路径均为指向 framework 内实际动态库的符号链接：

```text
mujoco.framework/Versions/A/libmujoco.3.9.0.dylib
```

仓库的 `.vscode/settings.json` 已为 rust-analyzer 配置等效的 MuJoCo 链接环境，因此 VS Code 内部的 `cargo check` / `cargo clippy` 不需要额外手动 source 脚本。终端中的 Cargo 构建、运行和测试则应先从仓库根目录执行 `source scripts/mujoco_env.zsh`。

两端共用 `src/apollo_spec.rs` 中的 Apollo 部件表。Bevy 遍历全部部件生成可视外形；MuJoCo MJCF 只使用其中标记了物理质量的部件作为外形子集，整机惯性属性另由显式 `<inertial>` 指定。当前 MuJoCo 被控对象是零重力、一个 freejoint 的单刚体，接口接收 body-frame 6D 外力/力矩；默认姿态控制器只输出力矩。它尚不包含月球重力、月面接触、DPS/APS/RCS 执行器、推进剂消耗或变质量。
固定质量属性采用 NASA Apollo 11 实际轻载着陆工况：整机 `4932 kg`，项目轴 `X右/Y上/Z前` 下的对角惯量为 `(6332, 7953, 5879) kg·m²`。

当前 MuJoCo Apollo 外形单刚体 demo 默认运行双层姿态控制：

```text
outer loop: q_e -> omega_c                 # 复用四元数运动学姿态控制律
inner loop: omega_c - omega_body -> tau    # PI-D 角速度环输出 body-frame 力矩
plant:      J * omega_dot + omega x Jomega = tau, integrated by MuJoCo
```

也就是说，内环不再直接改运动学姿态积分，而是向 MuJoCo freejoint 刚体施加力矩。当前内环对角速度误差使用 PI，D 项作用于测量角速度微分，另有显式角速度阻尼、回算抗积分饱和和幅值限制。`R` 会重置 MuJoCo 状态和内环积分/微分历史。

### 坐标系约定与挑战收敛场景

MuJoCo freejoint 的位置、姿态和线速度在世界系中表达，`qvel[3..6]` 旋转速度则已在刚体局部坐标系中表达。`ApolloDynamicsState::angular_velocity` 因此明确表示机体系角速度，内环直接使用它计算误差，不再做姿态逆旋转。

为了不让坐标系错误被“恒等姿态到单轴目标”这个特例掩盖，MuJoCo demo 现在从固定的非恒等姿态和非零机体系角速度出发，两者轴向故意不对齐；按 `R` 也会恢复这个挑战初始状态。测试除了检查最终姿态与角速度收敛，还限定 2 s 时的短时收敛误差；恢复旧的重复旋转后，该短时验收会失败。另一个轴向测试检查在目标等于当前姿态时，单一机体系角速度必须产生同轴反向阻尼力矩。

## 固定步长控制架构

MuJoCo 仿真、控制器更新和 Bevy 渲染现在已经解耦：

```text
MuJoCo simulation dt:   0.002 s
Controller dt:          0.020 s
Control hold:           10 steps
```

`ApolloControlEnv` 每个控制周期只更新一次控制器，并将得到的 wrench 保持 10 个 MuJoCo 固定步长。`SharedApolloState` 在独立仿真线程中推进环境并发布快照；`mujoco_apollo_demo` 的 Bevy 系统只读取最新快照并同步可视模型，不使用渲染帧的 `delta time` 推进物理状态。因此 GPU 负载或窗口刷新率只影响画面刷新，不改变控制器和 MuJoCo 的离散时间基准。

当前仍未提供独立的 MuJoCo headless 实验入口和 MuJoCo CSV 日志。下一阶段的 RCS 喷口模拟、可视化及相平面控制 baseline 见下方 TODO 文档。

## 文档索引

- [四元数姿态控制推导与工程验证](docs/quaternion_attitude_control.md)
- [RCS 喷口模拟、可视化与相平面控制 baseline TODO](docs/rcs_thruster_allocation_dynamics_todo.md)

## 无图形界面日志验证

当前 headless 日志只验证简化的四元数运动学姿态控制，不运行 MuJoCo，也不是 RCS 或双层动力学实验。运行命令为：

```bash
source scripts/mujoco_env.zsh
cargo run --bin attitude_demo -- --headless-log
```

程序固定仿真 8 秒，运动学积分步长为 `1/60 s`，每 `0.1 s` 记录一次，并覆盖写入：

```text
logs/attitude_kinematics.csv
```

CSV 列为：

```text
time_s,qe0,qev_norm,error_angle_rad,omega_norm,omega_x,omega_y,omega_z
```

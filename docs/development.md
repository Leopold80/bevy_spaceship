# 构建、测试与开发

除非特别说明，本页命令都从仓库根目录执行。

## Rust 工具链

仓库通过 `rust-toolchain.toml` 固定 Rust 1.95.0，使用 edition 2024 workspace。

纯数据与接口测试不需要 MuJoCo：

```bash
cargo test -p apollo-core
```

MuJoCo crate 在 macOS 下需要先加载仓库内的链接环境：

```bash
source scripts/mujoco_env.zsh
cargo test -p apollo-mujoco
```

脚本按照自身文件位置定位仓库，不依赖调用时的 `PWD`。若从外部 Rust 工程使用 path dependency，可以 source 该脚本的实际路径，再运行外部工程。

默认 viewer 只负责模型和离线回放，不要求 MuJoCo：

```bash
cargo check -p apollo-viewer
cargo test -p apollo-viewer
```

## Python 开发环境

Python 包使用 PyO3 与 maturin。本机直接复用现有 `cybernetic_env`，不再为本仓库重复创建虚拟环境。

首次构建原生扩展，或者 Rust/PyO3 代码变化后，在同一个 shell 中执行：

```bash
conda activate cybernetic_env
source scripts/mujoco_env.zsh
maturin develop
pytest python/tests
```

当前 `cybernetic_env` 已包含 NumPy、maturin 和 pytest；通常不需要安装额外工具。若将来环境确实缺少构建或测试工具，再执行：

```bash
python -m pip install maturin pytest
```

`maturin develop` 只需要在首次安装或原生扩展变化后重新运行，但每个新 shell 在导入并使用 Python plant 前都必须重新激活环境并加载 MuJoCo 动态库路径：

```bash
conda activate cybernetic_env
source scripts/mujoco_env.zsh
python python/examples/closed_loop_attitude.py
```

不要用 `source ... && conda run ...` 代替。macOS 上 `conda run` 会过滤子进程的 `DYLD_LIBRARY_PATH`，导致已经构建的扩展仍找不到仓库内的 MuJoCo framework。

Python API 测试覆盖与 Rust 相同的 reset、step、确定性、输入校验和版本化轨迹契约。Python `spawn()` 必须显式接收初始状态；writer 必须显式接收 reset 后的 tick 0 初始快照与 `plant.timing`。

## 分层验证

提交前在已激活 `cybernetic_env` 且已加载 `scripts/mujoco_env.zsh` 的同一个 shell 中，按以下顺序执行：

1. `cargo test -p apollo-core`
2. `cargo test -p apollo-mujoco`
3. `cargo test -p apollo-viewer --features live --all-targets`
4. `maturin develop` 与 `pytest python/tests`
5. `cargo test --workspace --all-targets`
6. `cargo clippy --workspace --all-targets --all-features -- -D warnings`

脚本会把 MuJoCo 和当前 Conda 的 `libpython` 路径一起加入动态链接搜索路径。第 5、6 步覆盖 Python 和 live feature，因此也应保留同一个环境。

同时检查依赖边界：

```bash
cargo tree -p apollo-core
cargo tree -p apollo-mujoco
cargo tree -i glutin
```

`apollo-core` 不应出现 Bevy、MuJoCo 或 PyO3；MuJoCo 不启用自带 viewer/renderer，因此不应再通过这条依赖链引入 `glutin`。`cargo tree -i glutin` 报告“package ID ... did not match any packages”表示该依赖不存在；该命令在这种情况下会返回非零状态，不应直接放进要求所有命令均为零退出码的脚本。

## 平台边界

仓库当前跟踪并验证的是 macOS MuJoCo 3.9.0 framework。其他平台需要自行提供与 `mujoco-rs` 兼容的 MuJoCo 动态库与链接环境；现有脚本不承诺跨平台安装。

## 代码约定

- 动力学、坐标系、控制接口和工程说明优先使用中文注释。
- 公共字段名必须携带坐标系或单位信息。
- 不允许在 demo、viewer 和 Python 绑定中复制状态推进逻辑。
- 新控制器首先以外部示例接入，不向 plant 增加 controller 概念。
- 轨迹 header 必须保存 reset 后、任何动作执行前的 tick 0 初始快照。

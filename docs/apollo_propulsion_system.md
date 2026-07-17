# Apollo 11 月球舱推进系统与本项目实现

这篇文档面向第一次接触阿波罗月球舱（Lunar Module，LM）的读者。目标不是只列出
API 名称，而是解释：真实飞船上有哪些发动机、为什么需要 16 个 RCS 喷口、DPS
怎样控制下降，以及这些硬件在本项目中如何变成可调用、可观察、可验证的仿真接口。

## 1. 先认识 Apollo 月球舱

执行登月任务的 Apollo 飞船不是一个不可分割的整体。与本项目直接相关的部分是月球舱，
它又分为上下两级：

```text
完整月球舱（下降和着陆阶段）
├── 上升级 Ascent Stage
│   ├── 乘员舱
│   ├── 16 个 RCS 推力器（4 个 quad，每组 4 个）
│   └── APS 上升发动机（分级后使用）
└── 下降级 Descent Stage
    ├── 起落架
    ├── 下降推进剂
    └── DPS 下降发动机
```

完整月球舱下降和着陆时，主要使用：

- **DPS（Descent Propulsion System）** 提供沿月球舱竖直轴的大推力；
- **RCS（Reaction Control System）** 提供姿态控制和小范围平移。

从月面起飞前，上升级与下降级分离。下降级留在月面，DPS 也随之被抛下；上升级改用
固定推力的 APS（Ascent Propulsion System）起飞，16 个 RCS 仍跟随上升级。

本项目当前模型表示“完整、尚未分级的 Apollo 11 月球舱”，因此本轮实现 **RCS + DPS**。
APS 要等模型真正支持级间分离、上升级质量和新质心后再加入。Apollo 指令/服务舱的
SPS（Service Propulsion System）属于另一艘舱段，也不在当前模型中。

## 2. 坐标系：NASA 文档与本项目不一样

阅读 NASA 图纸时首先要转换轴，否则喷口方向很容易整体写错。

| 语义 | NASA LM 轴 | 本项目机体系 |
|---|---|---|
| 从下降级指向上升级 | `+X` | `+Y` |
| 航天员面对前窗时的右侧 | `+Y` | `+X` |
| 月球舱前方 | `+Z` | `+Z` |

本项目采用右手系：`+X` 向右、`+Y` 向上、`+Z` 向前。所有公共推进器位置和方向都已
转换到本项目机体系。字段 `force_direction_body` 表示**飞船受到的力方向**；Bevy 中的
尾焰方向与它相反。

## 3. RCS：为什么是 16 个小发动机

Apollo LM 的 RCS 有 16 个额定 100 lbf 推力器：

```text
16 thrusters = 4 quads × 4 thrusters/quad
100 lbf      = 444.822 N per thruster
```

四个 quad 安装在上升级外围。每个 quad 中：

- 两个推力器与 NASA `X` 轴平行，方向相反，负责竖直方向能力；
- 另外两个位于垂直于 `X` 的平面，分别平行于 NASA `Y`、`Z` 方向；
- 两个推力器属于供给系统 A，另外两个属于供给系统 B。

A/B 是两套相互独立的氦气增压和推进剂供给系统。真实飞船可以在特定情况下交叉供给，
本轮只保存每个喷口的 A/B 归属，不模拟压力、阀门故障或交叉供给管路。

### 3.1 水平喷口不是 quad 的局部切向喷口

从上方看，四个 quad 位于四个对角方位；但每组的两台水平发动机仍平行于飞船本体的
`X/Z` 轴，并不沿“绕机体一圈”的局部切线安装：

```text
                              +Z（前）

             Quad 1                            Quad 4
       水平羽流：-X、+Z                  水平羽流：+X、+Z
                    \                        /
                     \                      /
 -X（左）  <----------+------ 机体中心 ------+---------->  +X（右）
                     /                      \
                    /                        \
             Quad 2                            Quad 3
       水平羽流：-X、-Z                  水平羽流：+X、-Z

                              -Z（后）
```

所以每股水平羽流都严格沿本体 `±X/±Z`，不是 quad 的局部切向；在当前四个对角方位上，
它与所在 quad 的局部径向、局部切向各成 `45°`。同一 quad 两股水平羽流的角平分线才
指向径向外侧。历史后缀 `F/A/R/L` 分别直接表示本体 `+Z/-Z/+X/-X` 的
**羽流方向**，飞船受力仍取反。若把喷管改成局部纯切向，就会同时破坏历史标签、
Data Book 站位关系和力矩分配结果。

### 3.2 一个喷口同时造成平移和转动

设第 `i` 个喷口在机体系中的作用点为 `r_i`，飞船质心为 `r_com`，产生的机体系力为
`f_i`。它对刚体产生：

```text
force_i  = f_i
torque_i = (r_i - r_com) × f_i
```

所以单独点火一个喷口通常既会让月球舱平移，也会让它转动。若希望主要产生姿态力矩，
应选择一对或多对喷口，让平动力互相抵消、力矩相加：

```text
Σ f_i ≈ 0
Σ (r_i - r_com) × f_i = desired torque
```

本轮提供原始 16 路喷口接口和用于人工检查的六个正负轴力矩偶，但不把自动分配算法塞进
plant。未来的正确分层仍是：

```text
期望六维力 / 期望力矩
    -> thruster allocator
    -> 16 路 RCS 点火命令
    -> RCS actuator
    -> MuJoCo 点力
```

### 3.3 脉冲不是理想矩形

RCS 可以连续工作，也可以发出短脉冲。NASA 资料给出的最短脉冲约为 14 ms；如此短的
脉冲中，推力尚未来得及达到稳态 100 lbf。阀门和燃烧室存在启动、上升与关断过程。

本项目的命令使用整数纳秒：

```text
on_time_ns[0..16]
```

每个元素表示从当前控制周期起点开始请求开启该喷口阀门的时间。对默认 20 ms 控制周期：

- `0`：不点火；
- 若周期起点阀门原本关闭，`0 < t < 14 ms`：新脉冲提升到最小 14 ms；
- 若上一周期恰好在边界仍保持开启，`0 < t < 14 ms`：作为连续点火的末段，精确保留 `t`；
- `14 ms <= t <= 20 ms`：保留精确请求时间；
- `t > 20 ms`：命令非法，整个 step 不推进。

若工厂使用自定义时序，最后一条上限随 `timing.control_step_ns()` 改变，而不是永远固定
20 ms；控制周期仍不得短于 14 ms。连续点火可用 Rust 的
`RcsCommand::hold(ids, timing)` 生成，或在每个控制周期重复请求完整门控时长。执行器保存
跨周期状态，因此相邻完整周期不会重复启动上升沿。例如先请求完整 `20 ms`，下一 tick
再请求 `1 ms`，实际是一次连续的 `21 ms` 点火，不会错误地变成 `20 + 14 ms`；如果阀门
已经真正关闭，再发 `1 ms` 新脉冲，才会重新应用 14 ms 下限。

这里要区分“阀门开了多久”和“推力非零了多久”：`applied_gate_on_time_ns` 只报告前者；
`mean_thrust_n` 才是把启动与关断瞬态积分后得到的本周期平均推力。当前瞬态模型采用：

- 前 7 ms 建模为零推力延迟：依据资料中“约 7 ms 后阀门完全打开”的时间尺度所作的
  工程近似；资料并没有说前 7 ms 推力严格为零；
- 连续开启到 20 ms 达到 100 lbf：工程近似；
- 关闭后用 8 ms 线性尾迹衰减：工程近似。

因此发出全关闭命令后，下一个控制周期仍可能看到一小段关断尾迹和非零
`mean_thrust_n`；这不是命令被偷偷保持。

### 3.4 历史喷口标签与稳定顺序

Apollo Operations Handbook 使用“供给系统字母 + quad 编号 + 方向字母”标识喷口。
本项目的 16 路顺序固定为：

```text
Quad 1: A1U, B1D, A1F, B1L
Quad 2: B2U, A2D, A2A, B2L
Quad 3: A3U, B3D, B3A, A3R
Quad 4: B4U, A4D, B4F, A4R
```

特别容易误读的一点是：`U/D/F/A/R/L` 表示**喷管和羽流朝向**，不是飞船受到的力。
推力方向必然与羽流相反。转换到本项目坐标后：

| 历史后缀 | 羽流方向 | `force_direction_body` |
|---|---|---|
| `U` | `+Y` | `-Y` |
| `D` | `-Y` | `+Y` |
| `F` | `+Z` | `-Z` |
| `A` | `-Z` | `+Z` |
| `R` | `+X` | `-X` |
| `L` | `-X` | `+X` |

例如 Operations Handbook 用四枚 “downward-firing” 喷口产生 `+X`（NASA 轴）平移，
恰好说明 `D` 是向下喷，而飞船向上受力。数组索引是稳定传输顺序；Rust 可用
`RcsThrusterId::label()`，两种语言也都可从规格的 `label` 字段取得人类可读名称。

## 4. DPS：下降级主发动机与 GDA

DPS 位于下降级中心，是 Apollo LM 下降、制动和软着陆所用的大推力发动机。它与 RCS
有两个重要区别：

1. DPS 推力大得多，而且可以节流；
2. DPS 喷管可以绕两个横向方向摆动，从而改变推力矢量。

本项目固定使用 Apollo 11 LM-5 参数档，避免混用后期任务数据：

| 参数/工作状态 | 数值 |
|---|---:|
| 关闭 | `0 N` |
| 可调最小值 | `1,050 lbf = 4,670.633 N` |
| 可调最大值 | `6,300 lbf = 28,023.796 N` |
| 独立全推力档 | `9,870 lbf = 43,903.947 N` |
| 最大摆角 | 距名义轴任意方向 `6°` |
| GDA 额定摆速 | 二维摆角模长 `0.2°/s` |

6,300 lbf 到 9,870 lbf 之间不作为连续节流区间。接口因此显式区分：

```text
Off
Variable { thrust_n, gimbal_x_rad, gimbal_z_rad }
FullThrust { gimbal_x_rad, gimbal_z_rad }
```

两个摆角组成一个二维倾斜向量，并限制在半径 6° 的圆形锥内，而不是分别限制成一个
方形区域。`gimbal_x_rad > 0` 表示推力朝机体 `+X` 倾斜，`gimbal_z_rad > 0` 表示朝
机体 `+Z` 倾斜；字段名不是“绕 X/Z 轴按右手定则旋转”的意思。名义推力沿本项目
`+Y`，喷焰沿 `-Y`。

可调档中低于 1,050 lbf 的正推力请求会钳到下限，高于 6,300 lbf 会钳到上限；二维
摆角长度超过 6° 时也会沿原方向缩到 6°。`DpsCommand` 中的两个摆角是 **GDA 目标角**，
不是命令提交后立即达到的实际角。目标先沿原方向限制到 6° 圆锥，再由有状态 GDA 在
每个物理子步中以二维模长不超过 `0.2°/s` 的速度追踪。

默认物理步长为 2 ms，因此每个物理子步最多移动 `0.0004°`；10 个子步组成的 20 ms
控制周期最多移动 `0.004°`。每个子步都使用当时的实际摆角计算 DPS 点力，
`AppliedDps.gimbal_x_rad/gimbal_z_rad` 返回的是本控制周期末的实际角，而
`requested_command.dps` 保留调用方提交的目标角。调用方不能把二者混作同一个量。

`DpsCommand::Off/off()` 将推力置零，但带制动的 GDA 保持最后实际角，不自动回中；
`reset()` 才把 GDA 回到零位。零或负的可调推力不是“关闭”，而是非法输入。

本轮 DPS 推力档仍是稳态模型，但 GDA 已包含摆速受限的执行机构状态。仍未模拟真实发动机
启动、关机、推进剂流量、贮箱压力或燃烧室瞬态。

## 5. 喷口安装位置与数据可信度

NASA LM Data Book 给出了 Quad IV 四个 RCS 发动机的参考站位。单位为英寸：

| 发动机参考 | NASA X station | NASA Y station | NASA Z station |
|---|---:|---:|---:|
| Forward | 254.0 | 61.5 | 66.35 |
| Up | 258.8 | 66.1 | 66.1 |
| Down | 248.7 | 66.1 | 66.1 |
| Side (right) | 254.0 | 66.35 | 61.5 |

其余三个 quad 使用对应的符号对称关系。本项目保留这些相对尺寸和每个喷口的独立作用点，
但 NASA station 原点与当前简化 Bevy 模型的原点不同，因此将 NASA `X=254 in` 的 quad
中心平移对齐到本项目 `Y=3.0 m`。这是坐标原点对齐，不是把 NASA station 直接冒充本项目
绝对坐标。

DPS 摆动枢轴目前取当前下降级几何中心线 `(0, 1.24, 0) m`。它是与当前简化网格一致的
模型拟合值，文档和界面都会明确标注，不声称是 Apollo 11 发动机万向节的测绘坐标。

## 6. 仿真周期与执行顺序

推进器引入后仍使用：

```text
MuJoCo physics step = 2 ms
substeps/control    = 10
control step       = 20 ms (50 Hz)
```

14 ms 最小脉冲可以放进一个 20 ms 控制周期。7 ms 阀门特征点虽然不落在 2 ms 网格上，
但无需把它粗略取整。对每个物理子步使用区间平均推力：

```text
F_mean(k) = integral(F(t), t_k .. t_k + dt) / dt
```

这样可在保持 2 ms 刚体积分步长的同时，保留脉冲曲线的冲量。每个子步都重新计算：

1. 各喷口当前平均推力；
2. GDA 实际摆角朝经 6° 圆锥限幅后的目标移动，二维移动量不超过
   `0.2°/s × 2 ms`；
3. 当前姿态下的世界系喷口位置；
4. 当前 RCS 方向和 DPS 实际摆角对应的世界系推力方向；
5. MuJoCo 点力和相对于质心的力矩；
6. 一次 2 ms MuJoCo step。

Bevy 渲染帧率不参与上述过程。按键只生成“下一控制 tick 的操作意图”，worker 在 20 ms
边界消费命令。

实现已经用相同的一秒对称四喷口平移场景比较 2 ms × 10 与 1 ms × 20：速度差小于
`1e-8 m/s`、位置差小于 `3e-4 m`、角速度差小于 `1e-10 rad/s`，测试通过。因此本轮
没有缩短默认物理步长；控制周期也继续保持 20 ms。若今后加入接触、推进剂晃动或更快
阀门模型，应针对新快时间尺度重新做收敛测试，而不是沿用本次结论。

GDA 的 `0.2°/s` 摆速也直接在上述 2 ms 子步中实施；默认 20 ms 控制周期只决定一次
`step()` 包含十次追踪更新，并不要求改变仿真步长或控制周期。

## 7. 两套 plant 为什么同时保留

仓库现在有两种有意并存的输入层级：

```text
ApolloPlant
    input: ideal BodyWrench
    purpose: 验证控制律、刚体动力学和算法边界

ApolloPropulsionPlant
    input: PropulsionCommand(RCS + DPS)
    purpose: 验证真实执行器约束、安装点和耦合响应
```

理想 wrench plant 不会被删除或改成巨大的 `Action` 枚举。这样当闭环表现异常时，可以
交叉比较：理想 wrench 正常而推进器 plant 异常，问题通常在执行器或分配；两者都异常，
才更可能是控制律、坐标系或状态处理问题。

## 8. Rust 调用方式

推进器 plant 仍然是同步、显式动作接口：

```rust
use apollo_mujoco::{
    ApolloPropulsionPlantFactory, ApolloState, DpsCommand, PropulsionCommand,
    RcsCommand,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let factory = ApolloPropulsionPlantFactory::apollo11_touchdown()?;
    let mut plant = factory.spawn(ApolloState::ZERO)?;

    let thruster = factory.propulsion_spec().rcs_thrusters[0].id;
    let rcs = RcsCommand::single_pulse(thruster, 14_000_000);
    let dps = DpsCommand::Off;

    let step = plant.step(PropulsionCommand { rcs, dps })?;
    println!("{:?}", step.applied.mean_wrench_body);
    Ok(())
}
```

一次 `step()` 恰好推进一个 20 ms 控制周期。返回值同时包含：

- 调用方原始请求，其中 DPS 摆角是 GDA 目标角；
- 每个 RCS 喷口的控制周期平均推力；
- DPS 实际推力和周期末 GDA 实际摆角；
- 实际点力合成的平均机体系 wrench；
- 推进后的 `PlantSnapshot`。

## 9. Python 调用方式

Python 公共对象是 frozen dataclass，16 路时间数组是只读 `numpy.uint64`：

```python
from apollo_sim import (
    ApolloPropulsionPlantFactory,
    ApolloState,
    DpsCommand,
    PropulsionCommand,
    RcsCommand,
)

factory = ApolloPropulsionPlantFactory.apollo11_touchdown()
plant = factory.spawn(ApolloState.identity())

thruster = factory.propulsion_spec.rcs_thrusters[0].id
command = PropulsionCommand(
    rcs=RcsCommand.single_pulse(thruster, 14_000_000),
    dps=DpsCommand.off(),
)

step = plant.step(command)
print(step.applied.rcs.mean_thrust_n)
print(step.applied.mean_wrench_body)
```

Python 层只负责校验、数据对象和错误映射；状态积分、执行器限制和点力计算仍全部使用
Rust/MuJoCo 实现。

## 10. Bevy 中能看到什么

Bevy 外形增加：

- 四个 RCS 吊臂和 quad 壳体；
- 16 个方向正确的喷管；
- Apollo 11 在各 quad 下方使用的喷流挡板；
- DPS 安装区、喉部、金属外壁、深色内壁和扩张喷管；
- 与实际平均推力同步的 RCS/DPS 尾焰。

交互 demo 提供单喷口、六个正负轴力矩偶和 DPS 模式。尾焰读取 `applied` 结果而不是
键盘状态，所以最小脉冲、启动曲线、节流限幅和 GDA 慢速追踪都会反映到画面；DPS
喷管姿态读取周期末 `AppliedDps` 实际角，不会瞬间跳到键盘设置的目标角。运行方式：

```bash
source scripts/mujoco_env.zsh
cargo run -p apollo-viewer --features live --bin apollo-propulsion-demo
```

暂停/单步、模式切换、脉冲、连续点火和 DPS 档位等完整键位见[使用方式](usage.md)。

模型查看器的最终视觉 QA 固定输出 17 张图：8 张全景（正面、背面、左右、俯视、仰视、
两个三分之四视角），4 张不显示尾焰的 RCS 安装座近景，4 张每次只显示当前 quad 的
喷流/导流板斜视近景，以及 1 张 DPS 喷管近景。把安装座与喷流诊断拆开，可以避免透明
尾焰相互叠加掩盖底座法向或导流板交汇关系。

### 10.1 安装面与尾焰净空怎样核查

每一台 RCS 都有自己的薄安装板和圆法兰。安装板的法向、喷管轴和飞船受力方向共线，
喷口与尾焰则沿相反方向伸出。因此侧向喷口不再借用 quad 壳体的斜面来表达安装关系；
即使从左右或俯视特写观察，喷管也应当严格垂直于它自己的底座。

尾焰需要分成两类理解：

- 12 个自由喷流喷口中，4 个 `U` 向上排气，8 个 `F/A/R/L` 严格沿本体 `±X/±Z`
  排气；完整有限诊断锥体应避开 `apollo_visual_parts()` 返回的共享简化外形件；
- 4 个 `D` 喷口向下排气，Apollo 11 的实物构型本来就让羽流有意撞击各 quad 下方的
  导流板，再沿下降级外侧排走。画面中的直线锥必须在导流板处终止，导流后的短段必须
  向外离开机体，不能把尾焰继续画穿下降级。

所以这里不能笼统声称“16 路尾焰都不接触任何结构”。更准确的说法是：12 路具有直接
净空，4 路与专门的导流板发生有意的羽流撞击，并由导流板保护机体。NASA 的 Apollo 11
任务报告曾预测这种导流板撞击会使每台向下喷发动机损失约 `10.4 lbf` 有效推力。当前
MuJoCo 模型仍施加喷口自身的名义点力；导流板载荷、上述推力损失、热流和远场羽流都只
在画面与文档中说明，尚未进入动力学计算。

代码测试同时核查 16 个安装板法向、8 个水平喷流的本体 `±X/±Z` 轴向、12 个自由羽流
完整锥体与共享简化外形件的净空、4 个向下羽流与导流板的交汇，以及导流后短段离开
机体。17 张固定视角截图用于补充检查网格穿插、遮挡和视觉比例；解析测试与截图必须
同时通过。

## 11. 当前保真边界

本轮真实化的是**推进执行器层**，不是完整月面飞行环境。仍然不包含：

- 月球重力和月面接触；
- 推进剂消耗；
- 随推进剂变化的质量、质心和惯量；
- APS 和级间分离；
- RCS A/B 系统压力、交叉供给和故障；
- DPS 点火/关机热流体瞬态；
- GDA 伺服闭环、加速度、间隙、负载、电源波动与故障；当前只实现额定摆速运动学限制；
- 喷流撞击和热效应；
- 自动喷口分配器和姿态控制器；
- 推进器 JSONL v2 遥测。

因此它应被描述为“具有 Apollo 11 RCS/DPS 拓扑和执行器约束的固定质量零重力刚体”，
而不是完整高保真 Apollo 11 登月仿真。

## 12. 主要资料来源

- NASA, *Apollo 11 Press Kit*：Apollo 11 DPS 推力范围和摆角。
  <https://www.nasa.gov/wp-content/uploads/static/apollo50th/pdf/A11_PressKit.pdf>
- NASA, *Apollo Lunar Module News Reference — Reaction Control*：RCS 数量、推力、
  quad 布局、A/B 供给和脉冲特性。
  <https://www.nasa.gov/wp-content/uploads/static/history/alsj/LM10_Reaction_Control_ppRC1-12.pdf>
- Grumman/NASA, *LM Data Book, Volume II — RCS*, section 4.8.7：喷口参考站位。
  <https://ntrs.nasa.gov/api/citations/19730066752/downloads/19730066752.pdf>
- NASA, *Apollo Lunar Module News Reference — Main Propulsion*：DPS/APS 结构背景。
  <https://www.nasa.gov/wp-content/uploads/static/history/alsj/LM09_Main_Propulsion_ppMP1-22.pdf>
- NASA, *Apollo Lunar Module News Reference — Guidance, Navigation, and Control*：GDA
  `±6°` 行程与 `0.2°/s` 额定摆速。
  <https://www.nasa.gov/wp-content/uploads/static/history/alsj/LM08_Guidance-Navigation-Control_ppGN1-48.pdf>
- NASA, *Apollo 11 Image Library*：Apollo 11 LM 多角度照片与 RCS 喷流挡板。
  <https://www.nasa.gov/wp-content/uploads/static/history/alsj/a11/images11.html>
- NASA, *Apollo 11 Mission Report Supplement 5 — RCS System Performance*：向下喷口羽流
  撞击导流板造成的预测推力损失。
  <https://ntrs.nasa.gov/api/citations/19720018196/downloads/19720018196.pdf?attachment=true>

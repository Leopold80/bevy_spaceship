# 四元数姿态控制

## 前置知识

见开源教程：https://github.com/Krasjet/quaternion

## 四元数表示与双重覆盖

### 四元数表示

一个四元数可以表示为$q = \begin{bmatrix} q_0, q_v \end{bmatrix}$，其中$q_0$代表实部、$q_v$代表向量部。进一步，四元数表示姿态时通常被约束为单位四元数，即：
$$
q_0^2 + \Vert q_v \Vert^2 = 1
$$
与轴角的关系为：
$$
q = [\cos \dfrac{\theta}{2}, u \sin \dfrac{\theta}{2}]
$$
其中$u \in \{ u \in \mathbb{R}^3 \ | \  \Vert u \Vert = 1 \}$是单位向量的旋转轴，$\theta \in [0, 2 \pi]$是绕旋转轴旋转的角度。

### 双覆盖特性与长短路径

类比二维的零旋转可以同时用$0$与$2 \pi$表示，四元数也有同样的双覆盖特性。同样的旋转可以用$q$与$-q$表示：$q$对应旋转轴$u$、旋转角$\theta$；$-q$对应旋转轴$-u$、旋转角$2 \pi - \theta$（代入四元数轴角形式，结合三角函数诱导公式可以很容易得到）。为了避免重复表示带来的诸多不便（状态数值不一致、数值不稳定等），所以我们对四元数实施解缠绕（unwind avoidance)：

$$
\mathrm{if} \  q_0 < 0, \  \mathrm{then} \  q \leftarrow -q
$$

使用更加数学的表述：
$$
q_e \leftarrow \mathrm{sgn}^+ (q_0) q_e
$$
其中要注意，$\mathrm{sgn}^+(q_0)$在$q_0 = 0$时不能取值为0，一般定义$\mathrm{sgn}^+(0) = 1$。

现在，四元数与旋转一一对应。这对于后续的误差跟踪控制律设计至关重要，毕竟谁也不想让$q_e$一会儿收敛到$\begin{bmatrix} 1, 0, 0, 0 \end{bmatrix}$，另一会儿收敛到$\begin{bmatrix} -1, 0, 0, 0 \end{bmatrix}$。另外，在姿态的表示中，从一个姿态到另一个姿态存在两个路径：较远路径和较近的路径。在实行四元数姿态控制时，解缠绕通过强制$q_{e0} = \cos \dfrac{\theta_e}{2} \geq 0$使得$\theta_e \in [0, \pi]$从而达到只能表示短路径的效果，避免了控制律选择长路径的问题。

## 四元数姿态控制

### 四元数姿态运动学方程

根据四元数微分可以得到**姿态微分方程**：

$$
\boxed{\dot q = \dfrac{1}{2} q \otimes \begin{bmatrix} 0, \omega \end{bmatrix}}
$$

以及**期望姿态微分方程**：
$$
\boxed{\dot q_d = \dfrac{1}{2} q_d \otimes \begin{bmatrix} 0, \omega_d \end{bmatrix}}
$$

接下来讨论四元数误差动态。对$q_e = q_d^{-1} \otimes q$求时间导数得到：

$$
\dot q_e = \dot q_d^{-1} \otimes q + q_d^{-1} \otimes \dot q
$$

首先处理第一项，先求解$\dot q_d^{-1}$。对等式$q_d \otimes q_d^{-1} = 1$两边求导得到：

$$
\dot q_d \otimes q_d^{-1} + q_d \otimes \dot q_d^{-1} = 0
$$

代入期望姿态微分$$\dot q_d = \dfrac{1}{2} q_d \otimes \begin{bmatrix} 0, \omega_d \end{bmatrix}$$得到：

> **修正：** 这里保留原推导结构，仅补回期望姿态微分方程中的系数$\dfrac{1}{2}$。

$$
\dfrac{1}{2} q_d \otimes \begin{bmatrix} 0, \omega_d \end{bmatrix} \otimes q_d^{-1} + q_d \otimes \dot q_d^{-1}=0
$$

同时左乘$q_d^{-1}$，整理得到：

$$
\dot q_d^{-1} = -\dfrac{1}{2} \begin{bmatrix} 0, \omega_d \end{bmatrix} \otimes q_d^{-1}
$$

所以：
$$
\dot q_d^{-1} \otimes q = -\dfrac{1}{2} \begin{bmatrix} 0, \omega_d \end{bmatrix} \otimes q_d^{-1} \otimes q=-\dfrac{1}{2} \begin{bmatrix} 0, \omega_d \end{bmatrix} \otimes q_e
$$

接着处理第二项：

$$
q_d^{-1} \otimes \dot q = q_d^{-1} \otimes \dfrac{1}{2} q \otimes \begin{bmatrix} 0, \omega \end{bmatrix}=\dfrac{1}{2}q_e \otimes \begin{bmatrix} 0, \omega \end{bmatrix}
$$

> **修正：** 第二项最后也需要保留$\dfrac{1}{2}$。

合并整理：

$$
\dot q_e = -\dfrac{1}{2} \begin{bmatrix} 0, \omega_d \end{bmatrix} \otimes q_e + \dfrac{1}{2} q_e \otimes \begin{bmatrix} 0, \omega \end{bmatrix}
$$

最终得到**误差动态方程**如下:

$$
\dot q_e = \dfrac{1}{2} q_e \otimes \begin{bmatrix}0,  \omega - \mathrm{vec} \left( q_e^{-1} \otimes \begin{bmatrix} 0, \omega_d \end{bmatrix} \otimes q_e \right) \end{bmatrix}
$$

更简洁的：

$$
\boxed{\dot q_e = \dfrac{1}{2} q_e \otimes \begin{bmatrix}0, \omega^\prime  \end{bmatrix}}
$$
，其中$\omega^\prime = \omega-R^\top \left(q_e \right) \omega_d$

进一步，把左右两边拆分成实部-向量部的形式：

$$
\boxed{\begin{bmatrix} \dot q_{e0}\\ \dot q_{ev} \end{bmatrix} = \dfrac{1}{2}\begin{bmatrix} -q_{ev}^\top \omega^\prime\\ q_{e0} \omega^\prime + q_{ev} \times \omega^\prime \end{bmatrix}}
$$

> **修正：** 误差动态拆分式整体需要乘以$\dfrac{1}{2}$，与上面的四元数运动学方程保持一致。

### 四元数姿态环控制律

假设内层角速度控制律能使得角速度向量$\omega \in \mathbb R^3$够完美跟踪角速度指令$\omega_c \in \mathbb R^3$，并且所有姿态信息都经过上文提到的unwind avoidance，那么控制律：

$$
\boxed{w_c = -K_p \left( q_{e0} \right) q_{ev} + R^\top \left( q_e \right) \omega_d}
$$
，$K_p \in \mathbb S^3_{++}$

> **修正：** 前馈项应依赖完整误差四元数$q_e$对应的旋转矩阵，即$R^\top(q_e)\omega_d$，而不是$R^\top(q_{e0})\omega_d$。本文后续工程验证取$\omega_d=0$，因此不实现该前馈项。

可以使得姿态误差$q_e$半全局渐进稳定收敛到原点。

证明：

设计Lyapunov函数：

$$
V = 1 - q_{e0}^2= q_{ev}^\top q_{ev}
$$

> [!NOTE]
>
> 姿态误差四元数 \( q_e = [q_{e0},\, q_{ev}^\top]^\top \) 为单位四元数，满足约束
> $$q_{e0}^2 + q_{ev}^\top q_{ev} = 1$$
> 因此，函数
> $$ V = 1 - q_{e0}^2 = q_{ev}^\top q_{ev}$$
> 在该约束下可作为姿态误差大小的度量。该函数对所有姿态误差状态均满足 \(V \ge 0\)，并且
> $$ V = 0 \;\Longleftrightarrow\; q_{ev} = 0
> \;\Longleftrightarrow\;
> q_e = \pm [1,\,0,\,0,\,0]^\top $$
> 即系统处于零姿态误差状态。
>
> 随着 \(q_e\) 偏离误差零点集合 \(\{ \pm [1,\,0,\,0,\,0]^\top \}\)（可以把他理解成0度和360度），其向量部分 \(q_{ev}\) 的范数增大，从而 \(V\) 单调增大。因而，\(V\) 可以直观地理解为姿态误差四元数到该平衡点集合的“距离平方”度量，用于刻画姿态误差偏离零误差状态的程度。

对其求导得到：
$$
\begin{aligned}
\dot V &= 2q_{ev}^\top \dot q_{ev}\\
&= q_{ev}^\top \left( q_{e0} \omega^\prime + q_{ev} \times \omega^\prime \right)
\end{aligned}
$$

> **修正：** 因为$V=q_{ev}^\top q_{ev}$，所以$\dot V=2q_{ev}^\top\dot q_{ev}$；同时由于$\dot q_{ev}$中已有$\dfrac{1}{2}$，两者相乘后恢复为上式。

其中$q_{ev}^\top \left( q_{ev} \times \omega^\prime \right) = 0$，并使得$\omega$等于角速度指令$\omega_c$，所以有：
$$
\begin{aligned}
\dot V &= q_{ev}^\top  q_{e0} \omega^\prime \\
&= q_{e0} q_{ev}^\top \left( -K_p q_{ev} \right) \\
&= -  q_{e0} q_{ev}^\top K_p q_{ev} \\
&= -q_{e0} \left\Vert q_{ev} \right\Vert_{K_p}^2
\end{aligned}
$$
由于我们已经对于姿态信息进行了unwind avoidance，所以$q_{e0} \geq 0$恒成立。所以有$\dot V \leq 0$恒成立。

另外，根据$q_{e0} = \cos \dfrac{\theta_e}{2}$，所以当误差姿态的旋转角$\theta_e = \pi$时，Lyapunov沿轨线的微分$\dot V = -\cos \dfrac{\theta_e}{2} \left\Vert q_{ev} \right\Vert_{K_p}^2 = 0$。但是我们进一步分析，在$\theta_e = \pi$的邻域有进一步说明，当姿态误差旋转角 $\theta_e = \pi$ 时，有
$$
q_{e0} = \cos \frac{\theta_e}{2} = 0, \qquad \| q_{ev} \| = 1,
$$
此时 Lyapunov 函数沿系统轨线的导数满足
$$
\dot V = - \cos \frac{\theta_e}{2} \, \| q_{ev} \|_{K_p}^2 = 0.
$$
因此，$\theta_e=\pi$ 属于 Lyapunov 意义下的一阶不可判定点。

然而，$\theta_e=\pi$ 并非吸引点。考虑其邻域内的状态演化：若存在任意小扰动使得
$$
\theta_e = \pi - \varepsilon, \quad \varepsilon > 0,
$$
则有
$$
q_{e0} = \cos\!\left(\frac{\pi}{2} - \frac{\varepsilon}{2}\right)
= \sin \frac{\varepsilon}{2} > 0,
$$
从而
$$
\dot V = - q_{e0} \, \| q_{ev} \|_{K_p}^2 < 0.
$$
这表明系统状态将立即离开 $\theta_e=\pi$，并继续向零姿态误差平衡点演化。

另一方面，当系统精确处于
$$
q_e = [0,\; u]^\top, \quad \|u\|=1,
$$
且 $\omega^\prime = 0$ 时，有 $\dot q_e = 0$，该状态在理想模型下可以保持不变。因此，$\theta_e=\pi$ 对应的平衡点既不具有吸引性，也不会主动发散，属于中性平衡点。

综上，$\theta_e=\pi$ 是由四元数双重覆盖特性引入的中性平衡点，而在实施 unwind avoidance 后，系统的所有非平凡轨线均满足 $q_{e0}>0$，从而保证姿态误差在半全局意义下渐进收敛至零姿态误差平衡点。

> [!NOTE]
>
> 其实上面一段对于$\theta_e=\pi$情况的轨线分析是一种朴素的LaSalle不变集分析，但由于篇幅原因不展开论述。

> [!NOTE]
>
> 对于更一般化的做法，会忽略期望姿态变化率，即$\dot q_{d}^{-1}=0$。从而控制律简化为：$\omega_c = -K_p q_{e0} q_{ev}$。稳定性证明类似，不加论述。
>
> **修正：** 工程验证中取$\omega_d=0$且$K_p = k_p I$，因此使用$\omega_c = -k_p q_{e0} q_{ev}$。

$\blacksquare$

## 展望

上面基于一个极强假设：角速度环控制律可以完美跟踪角速度指令。这在工程中极其常见。但是为了理论的严谨，考虑后续使用反步法设计，包含内层角速度环的动态。

## 工程验证设置

本文配套的 Bevy Apollo 模型只做运动学外环验证：忽略刚体动力学、惯量矩阵、力矩执行器与内层角速度环，并假设实际角速度能够完美跟踪角速度指令$\omega_c$。实验日志记录$q_{e0}$、$\lVert q_{ev}\rVert$、误差角和角速度指令范数，用于检查姿态误差是否按短路径收敛。

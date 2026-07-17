# 四元数姿态控制

> 本文保留控制律推导与历史验证背景。当前架构不把控制器封装进 plant；
> 可执行闭环位于 Rust/Python 示例中，调用方以普通循环计算 `BodyWrench`
> 并调用 `step()`。接口与示例入口见 [使用方式](usage.md)。

## 前置知识

见开源教程：https://github.com/Krasjet/quaternion

## 四元数表示与双重覆盖

### 四元数表示

一个四元数可以表示为：

$$
q =
\begin{bmatrix}
q_0\\
q_v
\end{bmatrix},
$$

其中 \(q_0\) 代表实部，\(q_v \in \mathbb{R}^3\) 代表向量部。进一步，四元数表示姿态时通常被约束为单位四元数，即：

$$
q_0^2+\Vert q_v \Vert^2=1.
$$

四元数与轴角的关系为：

$$
q=
\begin{bmatrix}
\cos \dfrac{\theta}{2}\\
u\sin \dfrac{\theta}{2}
\end{bmatrix},
$$

其中 \(u\in\mathbb{R}^3\) 是单位旋转轴，满足 \(\Vert u\Vert=1\)，\(\theta\in[0,2\pi]\) 是绕该轴旋转的角度。

因此，对于姿态误差四元数：

$$
q_e=
\begin{bmatrix}
q_{e0}\\
q_{ev}
\end{bmatrix}
=
\begin{bmatrix}
\cos \dfrac{\theta_e}{2}\\
e_e\sin \dfrac{\theta_e}{2}
\end{bmatrix},
$$

其中 \(\theta_e\) 是误差旋转角，\(e_e\) 是误差旋转轴。由此可见：

$$
q_{ev}=e_e\sin \dfrac{\theta_e}{2}.
$$

也就是说，四元数误差向量部 \(q_{ev}\) 同时包含误差方向和误差大小信息。

### 双覆盖特性与长短路径

类比二维平面中零旋转可以同时用 \(0\) 与 \(2\pi\) 表示，四元数也有双覆盖特性。同一个空间旋转可以同时由 \(q\) 与 \(-q\) 表示：\(q\) 对应旋转轴 \(u\)、旋转角 \(\theta\)；\(-q\) 对应旋转轴 \(-u\)、旋转角 \(2\pi-\theta\)。

为了避免重复表示带来的数值不一致和路径选择问题，对误差四元数实施解缠绕，即 unwind avoidance：

$$
\mathrm{if}\ q_{e0}<0,\quad \mathrm{then}\ q_e\leftarrow -q_e.
$$

也可以写成：

$$
q_e\leftarrow \mathrm{sgn}^{+}(q_{e0})q_e,
$$

其中 \(\mathrm{sgn}^{+}(0)=1\)。

实施该处理后，有：

$$
q_{e0}=\cos\dfrac{\theta_e}{2}\ge 0,
$$

从而：

$$
\theta_e\in[0,\pi].
$$

这意味着误差四元数只表示从当前姿态到期望姿态的短路径，避免控制律选择长路径。

---

## 四元数姿态运动学方程

根据四元数微分可以得到姿态微分方程：

$$
\boxed{
\dot q=\dfrac{1}{2}q\otimes
\begin{bmatrix}
0\\
\omega
\end{bmatrix}
}
$$

以及期望姿态微分方程：

$$
\boxed{
\dot q_d=\dfrac{1}{2}q_d\otimes
\begin{bmatrix}
0\\
\omega_d
\end{bmatrix}
}
$$

其中 \(\omega\) 是当前角速度，\(\omega_d\) 是期望角速度。

定义误差四元数：

$$
q_e=q_d^{-1}\otimes q.
$$

对其求时间导数：

$$
\dot q_e=\dot q_d^{-1}\otimes q+q_d^{-1}\otimes \dot q.
$$

首先，由：

$$
q_d\otimes q_d^{-1}=1
$$

两边求导可得：

$$
\dot q_d\otimes q_d^{-1}+q_d\otimes \dot q_d^{-1}=0.
$$

代入：

$$
\dot q_d=\dfrac{1}{2}q_d\otimes
\begin{bmatrix}
0\\
\omega_d
\end{bmatrix},
$$

得到：

$$
\dfrac{1}{2}q_d\otimes
\begin{bmatrix}
0\\
\omega_d
\end{bmatrix}
\otimes q_d^{-1}
+
q_d\otimes \dot q_d^{-1}=0.
$$

左乘 \(q_d^{-1}\)，整理得到：

$$
\dot q_d^{-1}
=
-\dfrac{1}{2}
\begin{bmatrix}
0\\
\omega_d
\end{bmatrix}
\otimes q_d^{-1}.
$$

所以：

$$
\dot q_d^{-1}\otimes q
=
-\dfrac{1}{2}
\begin{bmatrix}
0\\
\omega_d
\end{bmatrix}
\otimes q_d^{-1}\otimes q
=
-\dfrac{1}{2}
\begin{bmatrix}
0\\
\omega_d
\end{bmatrix}
\otimes q_e.
$$

第二项为：

$$
q_d^{-1}\otimes \dot q
=
q_d^{-1}\otimes
\dfrac{1}{2}q\otimes
\begin{bmatrix}
0\\
\omega
\end{bmatrix}
=
\dfrac{1}{2}q_e\otimes
\begin{bmatrix}
0\\
\omega
\end{bmatrix}.
$$

合并整理：

$$
\dot q_e
=
-\dfrac{1}{2}
\begin{bmatrix}
0\\
\omega_d
\end{bmatrix}
\otimes q_e
+
\dfrac{1}{2}q_e\otimes
\begin{bmatrix}
0\\
\omega
\end{bmatrix}.
$$

进一步可写为：

$$
\boxed{
\dot q_e
=
\dfrac{1}{2}q_e\otimes
\begin{bmatrix}
0\\
\omega'
\end{bmatrix}
}
$$

其中：

$$
\omega'=\omega-R^\top(q_e)\omega_d.
$$

拆分成实部和向量部形式：

$$
\boxed{
\begin{bmatrix}
\dot q_{e0}\\
\dot q_{ev}
\end{bmatrix}
=
\dfrac{1}{2}
\begin{bmatrix}
-q_{ev}^\top\omega'\\
q_{e0}\omega'+q_{ev}\times\omega'
\end{bmatrix}
}
$$

这个结果是后续两个控制律稳定性证明的共同基础。

---

## 控制律一：\(q_{e0}q_{ev}\) 缩放反馈（理论参考）

### 控制律

假设内层角速度控制器足够快，可以使实际角速度 \(\omega\) 完美跟踪角速度指令 \(\omega_c\)。在经过 unwind avoidance 后，可先写出包含时变目标和矩阵增益的理论形式：

$$
\boxed{
\omega_c=-K_p q_{e0}q_{ev}+R^\top(q_e)\omega_d
}
$$

其中：

$$
K_p\in\mathbb{S}_{++}^{3}.
$$

若只考虑固定期望姿态 \(\omega_d=0\) 和标量增益 \(K_p=k_pI\)，控制律退化为：

$$
\boxed{
\omega_c=-k_p q_{e0}q_{ev},\qquad k_p>0.
}
$$

对应的代数表达式可以写成：

```rust
let omega = -kp * error.w * qev;
```

当前仓库没有保留这条缩放反馈的 Rust 运行时实现；它只作为理论对照，避免把已经删除的旧模块误认为公共 API。

### 轴角解释

由于：

$$
q_{e0}=\cos\dfrac{\theta_e}{2},
\qquad
q_{ev}=e_e\sin\dfrac{\theta_e}{2},
$$

所以：

$$
\omega_c
=
-k_p\cos\dfrac{\theta_e}{2}\sin\dfrac{\theta_e}{2}e_e
=
-\dfrac{k_p}{2}\sin\theta_e\,e_e.
$$

因此，该控制律可以理解为对 \(q_{ev}\) 的变增益反馈，其等效增益为：

$$
k_{\mathrm{eff}}(\theta_e)=k_pq_{e0}=k_p\cos\dfrac{\theta_e}{2}.
$$

当误差角接近 \(180^\circ\) 时，\(q_{e0}\rightarrow 0\)，因此角速度指令会被明显软化。

### Lyapunov 函数

沿用原文档的思路，取：

$$
\boxed{
V_1=1-q_{e0}^2=q_{ev}^\top q_{ev}.
}
$$

因为 \(q_e\) 是单位四元数，有：

$$
q_{e0}^2+q_{ev}^\top q_{ev}=1.
$$

所以：

$$
V_1\ge 0.
$$

并且：

$$
V_1=0
\Longleftrightarrow
q_{ev}=0
\Longleftrightarrow
q_e=\pm
\begin{bmatrix}
1\\0\\0\\0
\end{bmatrix}.
$$

在实施 unwind avoidance 后，\(q_{e0}\ge0\)，因此零误差姿态对应：

$$
q_e=
\begin{bmatrix}
1\\0\\0\\0
\end{bmatrix}.
$$

### 导数分析

由：

$$
V_1=q_{ev}^\top q_{ev}
$$

可得：

$$
\dot V_1=2q_{ev}^\top\dot q_{ev}.
$$

代入误差动态中的向量部：

$$
\dot q_{ev}
=
\dfrac{1}{2}\left(q_{e0}\omega'+q_{ev}\times\omega'\right),
$$

得到：

$$
\begin{aligned}
\dot V_1
&=
2q_{ev}^\top
\dfrac{1}{2}
\left(q_{e0}\omega'+q_{ev}\times\omega'\right)\\
&=
q_{ev}^\top
\left(q_{e0}\omega'+q_{ev}\times\omega'\right).
\end{aligned}
$$

由于：

$$
q_{ev}^\top(q_{ev}\times\omega')=0,
$$

所以：

$$
\dot V_1=q_{e0}q_{ev}^\top\omega'.
$$

在控制律：

$$
\omega_c=-K_pq_{e0}q_{ev}+R^\top(q_e)\omega_d
$$

和完美角速度跟踪假设 \(\omega=\omega_c\) 下，有：

$$
\begin{aligned}
\omega'
&=\omega-R^\top(q_e)\omega_d\\
&=\omega_c-R^\top(q_e)\omega_d\\
&=-K_pq_{e0}q_{ev}.
\end{aligned}
$$

因此：

$$
\begin{aligned}
\dot V_1
&=
q_{e0}q_{ev}^\top(-K_pq_{e0}q_{ev})\\
&=
-q_{e0}^2q_{ev}^\top K_pq_{ev}\\
&=
-q_{e0}^2\Vert q_{ev}\Vert_{K_p}^{2}.
\end{aligned}
$$

由于：

$$
q_{e0}\ge0,
\qquad
K_p\in\mathbb{S}_{++}^{3},
$$

所以：

$$
\boxed{
\dot V_1\le0.
}
$$

### \(180^\circ\) 姿态误差处的性质

当：

$$
\theta_e=\pi
$$

时，有：

$$
q_{e0}=0,
\qquad
\Vert q_{ev}\Vert=1.
$$

此时缩放反馈给出：

$$
\omega_c=-K_pq_{e0}q_{ev}=0.
$$

同时：

$$
\dot V_1
=
-q_{e0}^2\Vert q_{ev}\Vert_{K_p}^{2}=0.
$$

因此，所有精确 \(180^\circ\) 误差姿态在该运动学模型中构成一组非期望平衡。它们不是中性平衡：由轴角动力学 \(\dot\theta_e=-\frac{k_p}{2}\sin\theta_e\) 可知，\(\theta_e=\pi\) 在小扰动下是排斥的。它们出现不是由于四元数双覆盖造成的 unwinding 问题，而是由于控制律本身包含 \(q_{e0}\) 因子：在 \(q_{e0}=0\) 时，控制指令被压为零。

在工程仿真中，只要初始误差不是严格精确的 \(180^\circ\)，而是：

$$
\theta_e=\pi-\varepsilon,
\qquad
\varepsilon>0,
$$

则：

$$
q_{e0}=\cos\dfrac{\pi-\varepsilon}{2}
=
\sin\dfrac{\varepsilon}{2}>0.
$$

此时：

$$
\dot V_1<0.
$$

所以误差仍会下降，只是由于 \(q_{e0}\) 很小，大角度附近的收敛速度会变慢。

### 稳定性结论

对于 \(q_{e0}q_{ev}\) 缩放反馈：

$$
\boxed{
\omega_c=-K_pq_{e0}q_{ev}+R^\top(q_e)\omega_d
}
$$

在以下假设下：

1. 姿态误差定义为 \(q_e=q_d^{-1}\otimes q\)；
2. 对误差四元数实施 unwind avoidance，使 \(q_{e0}\ge0\)；
3. 内层角速度环可使 \(\omega=\omega_c\)；
4. \(K_p\in\mathbb{S}_{++}^{3}\)；
5. 期望角速度项通过 \(R^\top(q_e)\omega_d\) 前馈抵消；

可以得到：

$$
\dot V_1=-q_{e0}^2\Vert q_{ev}\Vert_{K_p}^{2}\le0.
$$

在排除精确 \(180^\circ\) 的不稳定非期望平衡集后，姿态误差沿短路径收敛到零误差姿态：

$$
q_e\rightarrow
\begin{bmatrix}
1\\0\\0\\0
\end{bmatrix}.
$$

该控制律的优点是形式平滑，并且和 \(V_1=1-q_{e0}^2\) 的证明结构贴合；缺点是接近 \(180^\circ\) 时会因为 \(q_{e0}\approx0\) 而显著降低控制指令。

---

## 控制律二：常增益 \(q_{ev}\) 反馈（当前例程外环）

### 控制律

去掉 \(q_{e0}\) 因子后，包含时变目标和矩阵增益的理论形式为：

$$
\boxed{
\omega_c=-K_pq_{ev}+R^\top(q_e)\omega_d
}
$$

若期望姿态固定，即 \(\omega_d=0\)，并取 \(K_p=k_pI\)，则为：

$$
\boxed{
\omega_c=-k_pq_{ev},\qquad k_p>0.
}
$$

当前 Rust `closed_loop_attitude` 例程只覆盖固定目标和标量增益，并把该反馈用作例程私有级联控制器的外环。Python 例程以单位姿态为固定目标，采用等价的误差方向。仓库不再提供旧运动学可视化运行模式，也没有把该控制律放入 plant 库 API。

### 轴角解释

由于：

$$
q_{ev}=e_e\sin\dfrac{\theta_e}{2},
$$

所以：

$$
\omega_c=-k_pe_e\sin\dfrac{\theta_e}{2}.
$$

该控制律仍然是非线性姿态误差反馈，只是相对于 \(q_{ev}\) 本身，反馈增益为常数 \(k_p\)。

当：

$$
\theta_e\rightarrow\pi
$$

时，有：

$$
\Vert\omega_c\Vert
=
k_p\sin\dfrac{\theta_e}{2}
\rightarrow k_p.
$$

因此，它不会在接近 \(180^\circ\) 姿态误差时自动把角速度指令压低。

### 为什么改用 \(V_2=1-q_{e0}\)

如果仍然使用：

$$
V_1=1-q_{e0}^2=q_{ev}^\top q_{ev},
$$

则其导数中会自然出现 \(q_{e0}\)：

$$
\dot V_1=q_{e0}q_{ev}^\top\omega'.
$$

代入 \(\omega'=-K_pq_{ev}\) 后：

$$
\dot V_1=-q_{e0}q_{ev}^\top K_pq_{ev}.
$$

这虽然仍然可以说明在 \(q_{e0}>0\) 的光滑区域内 \(\dot V_1\le0\)，但在 \(q_{e0}=0\) 处导数仍为零，不能直接体现常增益控制律在 \(180^\circ\) 处仍有非零角速度指令的性质。

因此，对于常增益 \(q_{ev}\) 型反馈，更自然的 Lyapunov 函数是：

$$
\boxed{
V_2=1-q_{e0}.
}
$$

这个改动只针对常增益反馈的证明。缩放反馈的证明仍然保留 \(V_1=1-q_{e0}^2\) 的路线。

### 正定性

经过 unwind avoidance 后：

$$
q_{e0}\in[0,1].
$$

因此：

$$
V_2=1-q_{e0}\ge0.
$$

当且仅当：

$$
V_2=0
$$

时，有：

$$
q_{e0}=1.
$$

又因为单位四元数满足：

$$
q_{e0}^2+q_{ev}^\top q_{ev}=1,
$$

所以：

$$
q_{ev}=0.
$$

因此：

$$
V_2=0
\Longleftrightarrow
q_e=
\begin{bmatrix}
1\\0\\0\\0
\end{bmatrix}.
$$

也就是说，在选定的短路径半球上，\(V_2=1-q_{e0}\) 是关于零姿态误差的正定函数。但若把它写成原始四元数的姿态函数，实际上是 \(V_2=1-|q_{e0}^{\rm raw}|\)，因而在 \(q_{e0}^{\rm raw}=0\) 的 \(180^\circ\) 切换边界不可微。

从轴角角度看：

$$
V_2=1-\cos\dfrac{\theta_e}{2}.
$$

当 \(\theta_e=0\) 时，\(V_2=0\)；当 \(\theta_e\) 从 \(0\) 增加到 \(\pi\) 时，\(V_2\) 单调增大到 \(1\)。

### 导数分析

由误差动态实部：

$$
\dot q_{e0}
=
-\dfrac{1}{2}q_{ev}^\top\omega'
$$

可得：

$$
\dot V_2
=
-\dot q_{e0}
=
\dfrac{1}{2}q_{ev}^\top\omega'.
$$

在控制律：

$$
\omega_c=-K_pq_{ev}+R^\top(q_e)\omega_d
$$

和完美角速度跟踪假设 \(\omega=\omega_c\) 下，有：

$$
\begin{aligned}
\omega'
&=\omega-R^\top(q_e)\omega_d\\
&=\omega_c-R^\top(q_e)\omega_d\\
&=-K_pq_{ev}.
\end{aligned}
$$

因此：

$$
\begin{aligned}
\dot V_2
&=
\dfrac{1}{2}q_{ev}^\top(-K_pq_{ev})\\
&=
-\dfrac{1}{2}q_{ev}^\top K_pq_{ev}\\
&=
-\dfrac{1}{2}\Vert q_{ev}\Vert_{K_p}^{2}.
\end{aligned}
$$

由于 \(K_p\in\mathbb{S}_{++}^{3}\)，所以：

$$
\boxed{
\dot V_2\le0.
}
$$

并且当 \(q_{ev}\ne0\) 时：

$$
\dot V_2<0.
$$

### 半球内的 LaSalle 不变集分析

以下标准光滑 LaSalle 论证严格适用于 \(q_{e0}>0\) 的半球内部。代码在 \(q_{e0}<0\) 时同时反号实部和向量部；对常增益 \(q_{ev}\) 反馈而言，这会使控制指令在精确 \(180^\circ\) 的表示切换边界不连续。因此，包含该边界的完整结论需要非光滑或混合系统论证，不能只依赖下面的光滑推导。

令：

$$
\dot V_2=0.
$$

由于 \(K_p\) 正定，可得：

$$
q_{ev}=0.
$$

由单位四元数约束：

$$
q_{e0}^2+q_{ev}^\top q_{ev}=1,
$$

得到：

$$
q_{e0}=\pm1.
$$

又因为 unwind avoidance 保证：

$$
q_{e0}\ge0,
$$

所以最大不变集为：

$$
\mathcal{M}
=
\left\{
q_e=
\begin{bmatrix}
1\\0\\0\\0
\end{bmatrix}
\right\}.
$$

因此，对于从 \(q_{e0}>0\) 出发且不触及切换边界的轨迹，由 LaSalle 不变性原理，姿态误差四元数渐近收敛到零误差姿态：

$$
q_e\rightarrow
\begin{bmatrix}
1\\0\\0\\0
\end{bmatrix}.
$$

即：

$$
\theta_e\rightarrow0.
$$

### \(180^\circ\) 姿态误差处的性质

当：

$$
\theta_e=\pi
$$

时，有：

$$
q_{e0}=0,
\qquad
\Vert q_{ev}\Vert=1.
$$

对于常增益控制律：

$$
\omega_c=-K_pq_{ev},
$$

只要 \(q_{ev}\ne0\)，角速度指令就不为零。

此时：

$$
\dot V_2
=
-\dfrac{1}{2}q_{ev}^\top K_pq_{ev}
<0.
$$

同时：

$$
\dot q_{e0}
=
-\dfrac{1}{2}q_{ev}^\top\omega'
=
\dfrac{1}{2}q_{ev}^\top K_pq_{ev}
>0.
$$

这说明，对代码在精确 \(180^\circ\) 处当前选定的四元数表示，\(q_{e0}\) 会从 \(0\) 向正方向增大，误差角会离开 \(\pi\) 并减小。该点的两个转动方向等长，代码的选择取决于当前四元数表示，并非一个连续且唯一的“短路径”选择。

### 稳定性结论

对于常增益反馈：

$$
\boxed{
\omega_c=-K_pq_{ev}+R^\top(q_e)\omega_d
}
$$

在同样假设下，可以得到：

$$
\dot V_2
=
-\dfrac{1}{2}\Vert q_{ev}\Vert_{K_p}^{2}
\le0.
$$

并且在光滑半球内部除零误差姿态外严格小于零。因此，该控制律可以使从 \(q_{e0}>0\) 出发的姿态误差渐近收敛到：

$$
q_e=
\begin{bmatrix}
1\\0\\0\\0
\end{bmatrix}.
$$

相对于缩放反馈，它的优势是接近 \(180^\circ\) 误差时仍然保留非零角速度指令；代价是角速度指令在大角度时更激进，并且在 \(180^\circ\) 的四元数表示切换边界不光滑。工程实现中通常需要角速度限幅或内层执行器约束；当前 Rust `closed_loop_attitude` 例程的私有控制器已对外环角速度指令限幅。

---

## 两种控制律的对比总结

| 项目 | \(q_{e0}q_{ev}\) 缩放反馈 | 常增益 \(q_{ev}\) 反馈 |
|---|---|---|
| 控制律 | \(\omega_c=-k_pq_{e0}q_{ev}\) | \(\omega_c=-k_pq_{ev}\) |
| 等效增益 | \(k_pq_{e0}=k_p\cos(\theta_e/2)\) | \(k_p\) |
| 轴角形式 | \(-\frac{k_p}{2}\sin\theta_e\,e_e\) | \(-k_p\sin(\theta_e/2)e_e\) |
| 推荐 Lyapunov 函数 | \(V_1=1-q_{e0}^2=\Vert q_{ev}\Vert^2\) | \(V_2=1-q_{e0}\) |
| Lyapunov 导数 | \(-q_{e0}^2\Vert q_{ev}\Vert_{K_p}^{2}\) | \(-\frac{1}{2}\Vert q_{ev}\Vert_{K_p}^{2}\) |
| \(180^\circ\) 误差处 | 指令为零，不稳定非期望平衡 | 指令非零，按当前四元数表示选择方向 |
| 工程特性 | 大角度更柔和 | 大角度更积极 |
| 当前仓库状态 | 仅作为本文理论参考 | Rust/Python 闭环例程采用该误差方向 |

---

## 工程验证设置

下面先记录运动学外环的历史验证设置：它忽略刚体动力学、惯量矩阵、力矩执行器、饱和约束和内层角速度环，并假设实际角速度能够完美跟踪角速度指令 \(\omega_c\)。这部分现在只用于解释推导，不再对应一个库内运行模式。

历史运动学验证使用 \(k_p=2.4\)，并且只使用常增益控制律：

$$
\boxed{
\omega_c=-k_pq_{ev}.
}
$$

当前 Rust `closed_loop_attitude` 例程把旧级联控制基线迁到了调用方代码中。其外环使用常增益反馈 \(k_p=5.0\)，并对角速度指令做幅值限制：

$$
\boxed{
\omega_c=\operatorname{sat}_{1.35}\!\left(-5q_{ev}\right).
}
$$

随后，例程私有的内层角速度控制器向 MuJoCo plant 输出 body-frame、about-CoM 力矩。它不是公共 `ControlLaw` API，也不是对跟踪误差求导的标准 PID，而是带测量微分、显式角速度阻尼和回算抗饱和的 PI-D。先按控制器的意图写为：

$$
\boxed{
\begin{aligned}
\tau_{\rm raw}={}&42\,000e_\omega+680I\\
&-5\,000\,\operatorname{LPF}_{5\,\mathrm{Hz},\,Q=0.62}(\dot\omega_b)\\
&-14\,000\omega_b,\\
e_\omega={}&\omega_c-\omega_b.
\end{aligned}
}
$$

其中 \(\dot\omega_b\) 由相邻控制周期的测量角速度差分获得，微分对测量而非误差作用，以避免外环指令变化带来的 derivative kick；二阶 biquad 低通则用于滤除差分测量中的高频成分。积分向量的幅值上限为 \(6.4\)；力矩限幅前后的差值以 \(0.25\,\mathrm{s}\) 跟踪时间常数回算到积分状态，最终力矩向量限幅为 \(52\,000\)。

上式中的 \(\omega_b\) 是机体系角速度。MuJoCo freejoint 的旋转 `qvel[3..6]` 原生就在刚体局部坐标系中表达，因此例程直接使用 \(\omega_b=\omega_{qvel}\)，不再乘以 \(R(q)^\top\)。例程从非恒等姿态和与其旋转轴不对齐的非零机体系角速度出发，并在普通 `for` 循环中依次执行“读快照—算 wrench—调用 `step()`—显式记录”。运行入口和回放方式见 [使用方式](usage.md)。

这个双层控制例程已经不再假设角速度可以完美跟踪，而是把 \(\tau\) 施加到 MuJoCo freejoint 刚体，由 MuJoCo 积分刚体姿态动力学。它仍然是工程验证：当前还没有给出完整级联系统稳定性证明。

当前架构不再由 plant 隐式写 CSV。Rust 例程把 reset 后的 tick 0 初始快照写入版本化 JSONL header，并把调用方选择的每个 `PlantStep` 显式写为 telemetry frame。轨迹包含状态、请求 wrench、实际 wrench、`control_tick` 和 `physics_tick`，但**不包含**目标姿态、控制器内部的 \(\omega_c\)、积分状态或滤波状态。

因此，单靠 JSONL 可以直接分析姿态、测量机体系角速度和实际 wrench；若要计算：

$$
q_{e0},\quad
\Vert q_{ev}\Vert,\quad
\theta_e,\quad
\Vert\omega_c\Vert,
$$

还必须从例程源码或独立实验配置取得目标姿态和控制器参数。`apollo-replay` 不会声称能从轨迹恢复这些未记录的控制器内部量。

当前 Rust 例程自身实际自动检查的门槛是：

1. 2 s 时姿态误差小于 \(0.10\,\mathrm{rad}\)；
2. 2 s 时机体系角速度模长小于 \(0.50\,\mathrm{rad/s}\)；
3. 最终姿态误差小于 \(0.05\,\mathrm{rad}\)，且小于初始误差的 20%；
4. 最终机体系角速度模长小于 \(0.05\,\mathrm{rad/s}\)。

此外，记录器会在写入时校验 v1 header、tick 对齐和帧数值；仓库的独立验证流程再使用 `apollo-replay --validate-only` 检查完整文件可被读取。

误差是否逐 tick 单调、\(q_{e0}\) 是否在全部采样点保持非负、以及内部 \(\omega_c\) 的时间历程，目前不是例程的自动验收项。它们可以在同时保存目标与控制器诊断量的上层实验中进一步验证，不能仅凭现有 telemetry schema 作出承诺。

---

## 展望

上面的 Lyapunov 推导基于一个较强假设：角速度环可以完美跟踪角速度指令。这在运动学外环验证中是合理的简化。当前 MuJoCo plant 与调用方例程已经引入刚体姿态动力学：

$$
J\dot\omega+\omega\times J\omega=\tau,
$$

plant 实现转动惯量和连续 body-frame/about-CoM wrench 输入；例程实现角速度内环、角速度指令限幅、积分限幅和力矩限幅。二者通过公共状态/动作契约组合，而不是由库内 runner 绑定。下一步应当考虑：

1. 推进器执行层已经包含离散 RCS、最小脉宽和点力；下一步把本例程的连续期望力矩接到
   独立控制分配器，再输出当前 16 路 `PropulsionCommand`，而不是绕过执行器约束；
2. 扰动力矩、测量噪声和模型不确定性；
3. 对现有离散时间外环、PI-D 内环和限幅环节给出完整的级联或非线性稳定性分析；
4. 对常增益反馈在 \(180^\circ\) 四元数切换边界给出严格的非光滑或混合系统论证。

也就是说，当前文档的光滑 Lyapunov 证明覆盖运动学外环，而不覆盖已实现的完整 MuJoCo 刚体级联闭环，也不覆盖常增益反馈的 \(180^\circ\) 非光滑切换边界。

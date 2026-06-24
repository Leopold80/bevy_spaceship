# 四元数姿态控制

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

## 控制律一：当前代码使用的 \(q_{e0}q_{ev}\) 型反馈

### 控制律

假设内层角速度控制器足够快，可以使实际角速度 \(\omega\) 完美跟踪角速度指令 \(\omega_c\)。在经过 unwind avoidance 后，当前代码使用的姿态外环控制律为：

$$
\boxed{
\omega_c=-K_p q_{e0}q_{ev}+R^\top(q_e)\omega_d
}
$$

其中：

$$
K_p\in\mathbb{S}_{++}^{3}.
$$

若期望姿态固定，即 \(\omega_d=0\)，并取 \(K_p=k_pI\)，则实际工程验证中的控制律为：

$$
\boxed{
\omega_c=-k_p q_{e0}q_{ev},\qquad k_p>0.
}
$$

这与当前 Rust 代码保持一致，即：

```rust
let omega = -kp * error.w * qev;
```

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

此时旧控制律给出：

$$
\omega_c=-K_pq_{e0}q_{ev}=0.
$$

同时：

$$
\dot V_1
=
-q_{e0}^2\Vert q_{ev}\Vert_{K_p}^{2}=0.
$$

因此，精确 \(180^\circ\) 误差点在该运动学模型中是一个特殊中性点。这个点不是由于四元数双覆盖造成的 unwinding 问题，而是由于控制律本身包含 \(q_{e0}\) 因子：在 \(q_{e0}=0\) 时，控制指令被压为零。

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

对于当前代码使用的控制律：

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

在排除精确 \(180^\circ\) 中性误差点后，姿态误差沿短路径收敛到零误差姿态：

$$
q_e\rightarrow
\begin{bmatrix}
1\\0\\0\\0
\end{bmatrix}.
$$

该控制律的优点是形式平滑，并且和 \(V_1=1-q_{e0}^2\) 的证明结构贴合；缺点是接近 \(180^\circ\) 时会因为 \(q_{e0}\approx0\) 而显著降低控制指令。

---

## 控制律二：工程对照的常增益 \(q_{ev}\) 型反馈

### 控制律

作为对照，可以考虑去掉 \(q_{e0}\) 因子的控制律：

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

注意：该控制律最初作为文档中的工程对照方案；当前代码已实现该方案，并可在可视化演示中通过 `C` 键与旧控制律切换。

$$
\omega_c=-k_pq_{e0}q_{ev}.
$$

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

这虽然仍然可以说明 \(\dot V_1\le0\)，但在 \(q_{e0}=0\) 处导数仍为零，不能直接体现新控制律在 \(180^\circ\) 处仍有非零角速度指令的性质。

因此，对于常增益 \(q_{ev}\) 型反馈，更自然的 Lyapunov 函数是：

$$
\boxed{
V_2=1-q_{e0}.
}
$$

这个改动只针对新控制律的证明。旧控制律的证明仍然保留 \(V_1=1-q_{e0}^2\) 的原文档路线。

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

也就是说，在实施 unwind avoidance 后，\(V_2=1-q_{e0}\) 是关于零姿态误差的正定函数。

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

### LaSalle 不变集分析

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

由 LaSalle 不变性原理，姿态误差四元数渐近收敛到零误差姿态：

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

这说明在精确 \(180^\circ\) 误差处，\(q_{e0}\) 会从 \(0\) 向正方向增大，误差角会离开 \(\pi\) 并沿短路径减小。

### 稳定性结论

对于工程对照控制律：

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

并且除零误差姿态外严格小于零。因此，该控制律可以使姿态误差在短路径半球 \(q_{e0}\ge0\) 内渐近收敛到：

$$
q_e=
\begin{bmatrix}
1\\0\\0\\0
\end{bmatrix}.
$$

相对于旧控制律，它的优势是接近 \(180^\circ\) 误差时仍然保留非零角速度指令；代价是角速度指令在大角度时更激进，工程实现中通常需要进一步加入角速度限幅或内层执行器约束。

---

## 两种控制律的对比总结

| 项目 | 旧控制律，当前代码使用 | 新控制律，文档对照方案 |
|---|---|---|
| 控制律 | \(\omega_c=-k_pq_{e0}q_{ev}\) | \(\omega_c=-k_pq_{ev}\) |
| 等效增益 | \(k_pq_{e0}=k_p\cos(\theta_e/2)\) | \(k_p\) |
| 轴角形式 | \(-\frac{k_p}{2}\sin\theta_e\,e_e\) | \(-k_p\sin(\theta_e/2)e_e\) |
| 推荐 Lyapunov 函数 | \(V_1=1-q_{e0}^2=\Vert q_{ev}\Vert^2\) | \(V_2=1-q_{e0}\) |
| Lyapunov 导数 | \(-q_{e0}^2\Vert q_{ev}\Vert_{K_p}^{2}\) | \(-\frac{1}{2}\Vert q_{ev}\Vert_{K_p}^{2}\) |
| \(180^\circ\) 误差处 | 指令为零，中性点 | 指令非零，可离开该点 |
| 工程特性 | 大角度更柔和 | 大角度更积极 |
| 当前代码状态 | 已实现，可视化默认模式 | 已实现，可按 `C` 切换 |

---

## 工程验证设置

本文配套的 Bevy Apollo 模型只做运动学外环验证：忽略刚体动力学、惯量矩阵、力矩执行器、饱和约束和内层角速度环，并假设实际角速度能够完美跟踪角速度指令 \(\omega_c\)。

当前代码同时实现两种控制律。可视化演示默认使用旧控制律：

$$
\boxed{
\omega_c=-k_pq_{e0}q_{ev}.
}
$$

并支持按 `C` 切换到常增益控制律：

$$
\boxed{
\omega_c=-k_pq_{ev}.
}
$$

实验日志记录：

$$
q_{e0},\quad
\Vert q_{ev}\Vert,\quad
\theta_e,\quad
\Vert\omega_c\Vert.
$$

期望现象为：

1. \(q_{e0}\ge0\) 始终成立，说明 unwind avoidance 生效；
2. \(\Vert q_{ev}\Vert\) 逐渐减小并趋近于零；
3. \(\theta_e\) 逐渐减小并趋近于零；
4. \(\Vert\omega_c\Vert\) 随着误差缩小而减小；
5. 对接近但不等于 \(180^\circ\) 的初始误差，旧控制律仍会收敛，但大角度附近会更慢。

---

## 展望

上面的推导基于一个较强假设：角速度环可以完美跟踪角速度指令。这在运动学外环验证中是合理的简化，但如果要进一步接近真实航天器姿态控制，需要继续引入刚体姿态动力学：

$$
J\dot\omega+\omega\times J\omega=\tau,
$$

并进一步考虑：

1. 转动惯量矩阵 \(J\)；
2. 力矩输入 \(\tau\)；
3. 执行器饱和；
4. 角速度内环；
5. 扰动力矩；
6. 反步法或级联稳定性证明。

也就是说，当前文档证明的是运动学外环控制律的稳定性，而不是完整刚体动力学闭环系统的稳定性。

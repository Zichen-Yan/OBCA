# OBCA

System:

$$
x_{k+1}=f(x_k,u_k)\\
h(x_k,u_k) \leq 0\\
x_0=x_s\\
x_{N+1}=x_F
$$

运动学建模采用前轮驱动的车辆运动模型（即$\delta_r=0$）

![](https://pic3.zhimg.com/v2-90c5d4da07dea2cf033f18f21bc0bfba_r.jpg)

运动学方程：

$$
\begin{aligned}
\dot{x}&=v \cos(\psi +\beta)\\
\dot{y}&=v \sin (\psi +\beta)\\
\dot{\psi}&=\frac{v \sin(\beta)}{l_r}\\
\dot{v}&=a\\
\beta&=\tan^{-1}(\frac{l_r}{l_r+l_f}\tan \delta_f)
\end{aligned}
$$

障碍物用Polyhedron来表示:

$$
\mathbb{O}^m=\{ y\in \mathbb{R}^n:A^my \leq b^m \}
$$

定义当前自车位置$\mathbb{E}(x_k)=R(x_k)e^{\prime}+t(x_k)$，$e^{\prime}$表示自车坐标系下车的位置，是一个固定值。R表示旋转变换，t表示平移变换。

避障约束:

$$
\mathbb{E}(x_k) \cap \mathbb{O}^m = \empty
$$

定义自车与障碍物距离:

$$
\begin{aligned}
dist(\mathbb{E}(x),\mathbb{O})&=\min_{e,o}\{\|e-o\|: Ao \leq b,e\in \mathbb{E}(x)\}\\
&=\min_{e,o}\{\|R(x)e^{\prime}+t(x)-o\|: Ao \leq b,Ge^{\prime} \leq g\}\\
\end{aligned}
$$

由此该优化问题为

$$
\begin{aligned}
\underset{e,o}{\min} \ & \| w \|\\
\text{s.t.} & Ao\leq b\\
& Ge^{\prime} \leq g\\
& R(x)e^{\prime}+t(x)-o=w
\end{aligned}
$$

**Lagrangian函数的对偶:**

$$
\begin{aligned}
g(\lambda,z,\mu)&=\underset{o,e^{\prime},w}{\inf} \left( \|w\|+\lambda^T(Ao-b)+\mu^T(Ge^{\prime}-g)
+z^T(R(x)e^{\prime}+t(x)-o-w) \right)\\
&=\underset{o}{\inf}(\lambda^TAo-z^To)+\underset{e^{\prime}}{\inf}(\mu^T Ge^{\prime}
+z^TR(x)e^{\prime})+\underset{w}{\inf}(\|w\|-z^Tw)
\end{aligned}
$$

可得

$$
A^T\lambda=z\\
G^T \mu+R(x)^Tz=0\\
\|z\|_* \leq 1
$$

**第三项$\underset{x}{\inf}(\|x\|-y^Tx) \rightarrow |y|_* \leq 1$的推导如下:**

1）

共轭函数定义:

$$
f^*(y)=\underset{x \in f(\cdot)}{\sup}\{y^Tx-f(x)\}=\underset{x \in f(\cdot)}{\sup}\{y^Tx-\|x\|\}
$$

对偶函数定义:

$$
\|y\|_*=\underset{z}{\sup}\{z^Ty \mid \|z\| \leq 1 \}
$$

2）

当$\|y\|_* > 1时$，$z^Ty >1$。令$x=tz$，

$$
y^Tx-\|x\|=t(y^Tz-\|z\|)\\
\because z^Ty >1,\|z\| \leq 1 \\
\therefore y^Tz-\|z\|>0\\
\text{when} \quad t \rightarrow \infin, y^Tx-\|x\| \rightarrow \infin
$$

此时极小值不存在。

3）

当$\|y\|_* \leq 1$时，根据对偶范数的定义，有

$$
y^Tx \leq \|x\|\|y\|_*\\
y^T\frac{x}{\|x\|} \leq \|y\|_* \\
y^Tz \leq \|y\|_* \quad (z=\frac{x}{\|x\|},\|z\| \leq 1)
$$

所以

$$
y^Tx-\|x\| \leq \|x\|(\|y\|_*-1)\leq 0
$$

此时

$$
\underset{x}{\inf}(\|x\|-y^Tx)在x=0处取得最小值0
$$

**对偶函数：**

$$
\begin{aligned}
g(\lambda,z,\mu)&=-\lambda^Tb-\mu^Tg+z^Tt(x)\\
&=\lambda^T(At(x)-b)-\mu^Tg
\end{aligned}
$$

**对偶问题：**

$$
\begin{aligned}
\underset{\mu,\lambda}{\max} & \quad \lambda^T(At(x)-b)-\mu^Tg\\
\text{s.t.} & \quad G^T\mu+R(x)^TA^T\lambda=0\\
& \quad \|A^T \lambda \|_* \leq 1\\
& \quad \lambda \geq 0\\
& \quad \mu \geq 0
\end{aligned}
$$

目标函数可理解为两个凸集的distance，可当作不等式约束。且由于有$m$个障碍物，优化问题变为

$$
\begin{aligned}
\underset{x,u,\mu,\lambda}{\min} & \quad \sum_{k=0}^N l(x_k,u_k)\\
\text{s.t.} & \quad x_0=x_s, x_{N+1}=x_F\\
& \quad x_{k+1}=f(x_k,u_k),h(x_k,u_k) \leq 0\\
& \quad (A^mt(x_k)-b^m)^T\lambda_k^m-g^T \mu_k^{m}>0\\
& \quad G^T\mu_k^{m}+R(x_k)^TA^{(m)T}\lambda_k^{m}=0\\
& \quad \|A^{(m)T} \lambda_k^m \|_* \leq 1\\
& \quad \lambda_k^{m} \geq 0, \mu_k^{m} \geq 0\\
\end{aligned}
$$

对于无法避免碰撞的情况，引入松弛变量转化为软约束，即

$$
\begin{aligned}
\underset{x,u,\mu,\lambda}{\min} & \quad \sum_{k=0}^N [ l(x_k,u_k)+\kappa \cdot
\sum_{m=1}^Ms_k^m ]\\
\text{s.t.} & \quad x_0=x_s, x_{N+1}=x_F\\
& \quad x_{k+1}=f(x_k,u_k),h(x_k,u_k) \leq 0\\
& \quad (A^mt(x_k)-b^m)^T\lambda_k^m-g^T \mu_k^{m}>-s_k^m\\
& \quad G^T\mu_k^{m}+R(x_k)^TA^{(m)T}\lambda_k^{m}=0\\
& \quad \|A^{(m)T} \lambda_k^m \|_* = 1\\
& \quad s_k^{m} \geq 0, \lambda_k^{m} \geq 0, \mu_k^{m} \geq 0\\
\end{aligned}
$$

但在泊车场景下，不考虑碰撞情况。



# 1. OBCA

#### 1.1 系统建模

$$
x_{k+1}=f(x_k,u_k)\\
\text{s.t.} \ h(x_k,u_k) \leq 0, x_0=x_s, x_{N+1}=x_F
$$

运动学建模采用前轮驱动的车辆运动模型（即$\delta_r=0$）

![](https://pic3.zhimg.com/v2-90c5d4da07dea2cf033f18f21bc0bfba_r.jpg)

连续时间运动学方程：

$$
\begin{aligned}
\dot{x}&=v \cos(\psi +\beta)\\
\dot{y}&=v \sin (\psi +\beta)\\
\dot{\psi}&=\frac{v \sin(\beta)}{l_r}\\
\dot{v}&=a\\
\beta&=\tan^{-1}(\frac{l_r}{l_r+l_f}\tan \delta_f)
\end{aligned}
$$

优化问题中用的离散运动学方程（弧线运动）：

$$
\begin{aligned}
R&=\frac{WB}{\tan(steering)+0.00001}\\
x_{k+1}&=x_k-R \sin \theta+R\sin (\theta+D / R)\\
y_{k+1}&=y_k+R \cos \theta-R\cos (\theta+D / R)\\
\theta_{k+1} &= \theta_{k}+D/R
\end{aligned}
$$

#### 1.2 避障问题建模

障碍物用多面体Polyhedron来表示，如长方形Box：

$$
\mathbb{O}^m=\{ y\in \mathbb{R}^n:A^my \leq b^m \}
$$

定义当前自车位置$\mathbb{E}(x_k)=R(x_k)e^{\prime}+t(x_k)$，$e^{\prime}$表示自车坐标系下车的位置，是一个固定值，$R(\cdot)$表示旋转变换，$t(\cdot)$表示平移变换。

避障约束可表示为（即自车和障碍物不相交）:

$$
\mathbb{E}(x_k) \cap \mathbb{O}^m = \empty
$$

定义自车和障碍物的距离:

$$
\begin{align}
dist(\mathbb{E}(x),\mathbb{O})&=\min_{e,o}\{\|e-o\|: Ao \leq b,e\in \mathbb{E}(x)\}\notag\\
&=\min_{e,o}\{\|R(x)e^{\prime}+t(x)-o\|: Ao \leq b,Ge^{\prime} \leq g\}\\
\end{align}
$$

**1.2.1 令$w=e-o$，式(1)可写为优化问题**

$$
\begin{equation}
\begin{aligned}
\underset{e,o}{\min} \ & \| w \|\\
\text{s.t.} & Ao\leq b\\
& Ge^{\prime} \leq g\\
& R(x)e^{\prime}+t(x)-o=w
\end{aligned}
\end{equation}
$$

**1.2.2 式(2)的Lagrangian函数：**

$L(w,o,e^{\prime},\lambda,z,\mu)=|w|+\lambda^T(Ao-b)+\mu^T(Ge^{\prime}-g)
+z^T(R(x)e^{\prime}+t(x)-o-w)$

**1.2.3 Lagrangian函数的对偶:**

$$
\begin{equation}
\begin{aligned}
g(\lambda,z,\mu)&=\underset{o,e^{\prime},w}{\inf} L(w,o,e^{\prime},\lambda,z,\mu)\\
&=\underset{o}{\inf}(\lambda^TAo-z^To)+\underset{e^{\prime}}{\inf}(\mu^T Ge^{\prime}
+z^TR(x)e^{\prime})+\underset{w}{\inf}(\|w\|-z^Tw)
\end{aligned}
\end{equation}
$$

对偶函数存在极小值的条件为

$$
\begin{equation}
A^T\lambda=z, \ G^T \mu+R(x)^Tz=0, \ \|z\|_* \leq 1
\end{equation}
$$

=======================================================================

**式(4)中第三项$\underset{w}{\inf}(\|w\|-z^Tw) \rightarrow |z|_* \leq 1$的推导如下:**

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

此时$\underset{x}{\inf}(|x|-y^Tx)$在$x=0$处取得最小值0。

=======================================================================

**1.2.4 对偶函数：**

$$
\begin{aligned}
g(\lambda,z,\mu)&=-\lambda^Tb-\mu^Tg+z^Tt(x)\\
&=\lambda^T(At(x)-b)-\mu^Tg
\end{aligned}
$$

**1.2.5 对偶问题：**

$$
\begin{aligned}
\underset{\mu,\lambda}{\max} & \quad \lambda^T(At(x)-b)-\mu^Tg\\
\text{s.t.} & \quad G^T\mu+R(x)^TA^T\lambda=0\\
& \quad \|A^T \lambda \|_* \leq 1\\
& \quad \lambda \geq 0\\
& \quad \mu \geq 0
\end{aligned}
$$

目标函数可理解为最大化两个box的距离，这里把它当作不等式约束，大于0表示无碰撞。

**1.2.6 最后的优化问题：**

由于有$m$个障碍物，优化问题变为

$$
\begin{align}
\underset{x,u,\mu,\lambda}{\min} & \quad \sum_{k=0}^N l(x_k,u_k)\\
\text{s.t.} & \quad x_0=x_s, x_{N+1}=x_F\\
& \quad x_{k+1}=f(x_k,u_k),h(x_k,u_k) \leq 0\\
& \quad (A^mt(x_k)-b^m)^T\lambda_k^m-g^T \mu_k^{m}>0\\
& \quad G^T\mu_k^{m}+R(x_k)^TA^{(m)T}\lambda_k^{m}=0\\
& \quad \|A^{(m)T} \lambda_k^m \|_* \leq 1\\
& \quad \lambda_k^{m} \geq 0, \mu_k^{m} \geq 0\\
\end{align}

$$

式（5）中目标函数

$l(x_k,u_k)=u_k^T Ru_k+(x_k-x_k^*)^TQ(x_k-x_k^*)+(u_{k}-u_{k+1})^T R(u_{k}-u_{k+1})$

# 2. TDR-OBCA

#### 2.1 避障约束中加入松弛变量

不等式（8）中$(A^mt(x_k)-b^m)^T\lambda_k^m-g^T \mu_k^{m}>0$可松弛为$(A^mt(x_k)-b^m)^T\lambda_k^m-g^T \mu_k^{m}+d_k^m=0,d_k^m<0$，其中$d_k^m$越小说明安全距离越大。

#### 2.2 对偶变量初始化（warm start）提高收敛速度

利用Hybrid A*生成的初始路径$x^*$来计算对偶变量的初始值。

$$
\begin{equation}
\begin{aligned}
\underset{\mu,\lambda,d}{\min} & \quad \sum_{m=1}^M \sum_{k=1}^K d_k^m\\
\text{s.t.} & \quad (A^mt(x^*_k)-b^m)^T\lambda_k^m-g^T \mu_k^{m}+d_k^m = 0\\
& \quad G^T\mu_k^{m}+R(x^*_k)^TA^{(m)T}\lambda_k^{m}=0\\
& \quad \|A^{(m)T} \lambda_k^m \|_2^2 \leq 1\\
& \quad \lambda_k^{m} \geq 0, \mu_k^{m} \geq 0, d_k^m<0\\
\end{aligned}
\end{equation}
$$

为了进一步简化问题，将二次约束$\|A^{(m)T} \lambda_k^m \|_2 \leq 1$放到目标函数中去，即

$$
\begin{equation}
\begin{aligned}
\underset{\mu,\lambda,d}{\min} & \quad \frac{1}{\beta} 
\sum_{m=1}^M \|A^{(m)T} \lambda_k^m \|_2^2 
+\sum_{m=1}^M \sum_{k=1}^K d_k^m\\
\text{s.t.} & \quad (A^mt(x^*_k)-b^m)^T\lambda_k^m-g^T \mu_k^{m}+d_k^m = 0\\
& \quad G^T\mu_k^{m}+R(x^*_k)^TA^{(m)T}\lambda_k^{m}=0\\
& \quad \lambda_k^{m} \geq 0, \mu_k^{m} \geq 0, d_k^m<0\\
\end{aligned}
\end{equation}
$$

其中$\beta$为系数。由此可以计算出初始对偶变量$\mu,\lambda,d$。

#### 2.3 最终问题

$$
\begin{equation}
\begin{aligned}
\underset{x,u,\mu,\lambda,d}{\min} & \quad \sum_{k=0}^N l(x_k,u_k)\\
\text{s.t.} & \quad x_0=x_s, x_{N+1}=x_F\\
& \quad x_{k+1}=f(x_k,u_k),h(x_k,u_k) \leq 0\\
& \quad (A^mt(x_k)-b^m)^T\lambda_k^m-g^T \mu_k^{m}+d_k^m=0\\
& \quad G^T\mu_k^{m}+R(x_k)^TA^{(m)T}\lambda_k^{m}=0\\
& \quad \|A^{(m)T} \lambda_k^m \|_2^2 \leq 1\\
& \quad \lambda_k^{m} \geq 0, \mu_k^{m} \geq 0, d_k^m<0\\
\end{aligned}
\end{equation}
$$

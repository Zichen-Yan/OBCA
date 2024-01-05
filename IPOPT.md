### IPOPT

$$
\min f(x)\\
s.t. \quad g(x) \leq 0
$$

引入对数障碍物函数：$h(x)=-\log (-g(x))$

特点：光滑函数，具有连续的一阶和二阶导数。



原问题变为无约束形式，目标函数为

$F(x,\mu)=f(x)+\mu h(x)=f(x)-\mu \log (-g(x))$

通过减小$\mu \rightarrow 0$逼近原问题的最优解。

### 牛顿法：

一阶导数：

$\nabla F(x_k,\mu_k)=\nabla f(x_k)+\mu_k \frac{\nabla g(x_k)}{-g(x_k)}$

二阶导数：

$\nabla^2 F(x_k,\mu_k)=\nabla^2 f(x_k)+\mu_k \nabla (\frac{\nabla g(x_k)}{-g(x_k)})$

 计算更新方向：

$d_k=-\frac{\nabla F(x_k,\mu_k)}{\nabla^2 F(x_k,\mu_k)}$

更新方式($\alpha_k$为更新步长)：

$$
x_{k+1}=x_k+\alpha_k d_k\\
\mu_{k+1}=\rho \mu_k\\
0<\rho<1
$$

直到收敛$\mu_k*h(x_k)<\epsilon$。

##### 牛顿法更新方向：

当最小化目标函数$f(x)$时，对$f(x)$在$x_k$处进行二阶Taylor展开。

$$
f(x)=f(x_k)+\nabla f(x_k)^T(x-x_k)+\frac{1}{2}(x-x_k)^T\nabla^2 f(x_k)(x-x_k)
$$

令$f(x)$导数为0，

$$
\nabla_x f(x) = \nabla f(x_k)+\nabla^2 f(x_k)(x-x_k)=0\\
x-x_k=-\frac{\nabla f(x_k)}{\nabla^2 f(x_k)}\\
x_{k+1}=x_k-\frac{\nabla f(x_k)}{\nabla^2 f(x_k)}
$$



已知三个点

$$
p_1=(x_1,y_1,\theta_1,steering_1,D_1)\\
p_2=(x_2,y_2,\theta_2,steering_2,D_2)\\
p_3=(x_3,y_3,\theta_3,steering_3,D_3)
$$

状态量：$s=[x,y,\theta]$

控制量：$u=[steering,D]$

目标函数：$J=J_1+J_2$

$$
J_1=\sum_{i=1}^3u_i^T \begin{bmatrix} 1000 & 0 \\ 0 & 10 \end{bmatrix} u_i\\
J_2=\sum_{i=1}^2(u_i-u_{i+1})^T \begin{bmatrix} 10 & 0 \\ 0 & 1000 \end{bmatrix} (u_i-u_{i+1})
$$

约束方程：

$$
s_1=(x_1,y_1,\theta_1)\\
s_2=f(s_1,u_1)\\
s_3=f(s_2,u_2)\\
s_3=(x_3,y_3,\theta_3)\\
-12.5 \leq x \leq 12.5\\
-12.5 \leq y \leq 12.5\\
-2\pi \leq \theta \leq 2\pi\\
-0.4768 \leq steering \leq 0.4768\\
-0.2 \leq D \leq 0.2
$$

长方形可行驶区域表达式：

$$
Ax \leq b
$$

自车四个角点位置计算：

车辆中心坐标：

$$
x_c=x+offset*\cos \theta \\
y_c=y+offset*\sin\theta
$$

旋转矩阵：

$$
R=\begin{bmatrix}\cos \theta & -\sin \theta\\ \sin \theta& \cos \theta
\end{bmatrix}\\
$$

左前：

$$
p_{lf}=R*\begin{bmatrix} l/2 \\ 
w/2 \end{bmatrix}+\begin{bmatrix} x_c \\ y_c \end{bmatrix}
$$

左后：

$$
p_{lb}=R*\begin{bmatrix} -l/2 \\ 
w/2 \end{bmatrix}+\begin{bmatrix} x_c \\ y_c \end{bmatrix}
$$

右前：

$$
p_{rf}=R*\begin{bmatrix} -l/2 \\ 
-w/2 \end{bmatrix}+\begin{bmatrix} x_c \\ y_c \end{bmatrix}
$$

右后：

$$
p_{rb}=R*\begin{bmatrix} l/2 \\ 
-w/2 \end{bmatrix}+\begin{bmatrix} x_c \\ y_c \end{bmatrix}
$$

安全约束，四个角点都在可行区域内：

$$
Ap_{lf} \leq b\\
Ap_{lb} \leq b\\
Ap_{rf} \leq b\\
Ap_{rb} \leq b\\
$$



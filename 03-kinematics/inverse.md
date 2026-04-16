# 逆向运动学解算

> 轮子速度 → 机器人速度

---

## 什么是逆向运动学？

**逆向运动学**：已知四个轮子的实际速度，计算机器人的实际运动速度。

```
输入：V_fl, V_fr, V_rl, V_rr（四个轮子的线速度）
输出：Vx, Vy, ω（机器人实际速度）
```

---

## 为什么需要逆向运动学？

### 用途

**外环控制需要知道机器人的实际速度**。

```
外环控制流程：
1. 读取四个轮子的实际转速 actual_rpm[4]
2. 转换为线速度
3. 逆向运动学解算 → current_Vx, current_Vy, current_ω
4. 与目标速度比较，计算误差
5. PID计算修正量
```

### 为什么不直接用轮速？

外环控制的是机器人整体速度，不是单个轮子的速度。

```
例如：
  目标：机器人前进 0.5 m/s
  不是：四个轮子都转 100 RPM

因为：
  - 轮子打滑时，轮速不代表机器人速度
  - 需要综合四个轮子的信息
```

---

## 运动学公式

### 矩阵形式

从四个轮子速度反推机器人速度：

$$
\begin{aligned}
\begin{bmatrix} 
V_x \cr V_y \cr \omega 
\end{bmatrix} 
= \frac{r}{4} 
\begin{bmatrix} 
1 & 1 & 1 & 1 \cr 
-1 & 1 & 1 & -1 \cr 
-\frac{1}{L} & \frac{1}{L} & -\frac{1}{L} & \frac{1}{L} 
\end{bmatrix} 
\begin{bmatrix} 
V_{fl} \cr V_{fr} \cr V_{rl} \cr V_{rr} 
\end{bmatrix}
\end{aligned}
$$

### 展开形式

逐项展开的计算公式：

| 速度分量 | 公式 |
|:--------:|:-----|
| 前进速度 $V_x$ | $V_x = \frac{r}{4}(V_{fl} + V_{fr} + V_{rl} + V_{rr})$ |
| 平移速度 $V_y$ | $V_y = \frac{r}{4}(-V_{fl} + V_{fr} + V_{rl} - V_{rr})$ |
| 旋转角速度 $\omega$ | $\omega = \frac{r}{4L}(-V_{fl} + V_{fr} - V_{rl} + V_{rr})$ |

### 物理意义

| 公式 | 物理意义 |
|------|---------|
| Vx | 四个轮速的平均（前进分量） |
| Vy | 对角轮速的差（平移分量） |
| ω | 左右轮速的差（旋转分量） |

---

## 公式推导

### 推导思路

正向运动学公式是线性方程组，逆向运动学是其逆运算。

### 简化推导

从正向公式（假设 $r=1$ 简化）：

| 轮子 | 正向公式 |
|:----:|:---------|
| FL | $V_{fl} = V_x - V_y - \omega L$ |
| FR | $V_{fr} = V_x + V_y + \omega L$ |
| RL | $V_{rl} = V_x + V_y - \omega L$ |
| RR | $V_{rr} = V_x - V_y + \omega L$ |

**求 $V_x$**：四式相加

$$
\begin{aligned}
V_{fl} + V_{fr} + V_{rl} + V_{rr} &= 4V_x
\end{aligned}
$$

$$
\begin{aligned}
V_x &= \frac{V_{fl} + V_{fr} + V_{rl} + V_{rr}}{4}
\end{aligned}
$$

**求 $V_y$**：$(V_{fr} + V_{rl}) - (V_{fl} + V_{rr})$

$$
\begin{aligned}
V_{fr} + V_{rl} - V_{fl} - V_{rr} &= 4V_y
\end{aligned}
$$

$$
\begin{aligned}
V_y &= \frac{-V_{fl} + V_{fr} + V_{rl} - V_{rr}}{4}
\end{aligned}
$$

**求 $\omega$**：$(V_{fr} + V_{rr}) - (V_{fl} + V_{rl})$

$$
\begin{aligned}
V_{fr} + V_{rr} - V_{fl} - V_{rl} &= 4\omega L
\end{aligned}
$$

$$
\begin{aligned}
\omega &= \frac{-V_{fl} + V_{fr} - V_{rl} + V_{rr}}{4L}
\end{aligned}
$$

---

## 代码实现

### 逆向运动学函数

```c
// 逆向运动学解算
RobotVelocity_t inverse_kinematics(WheelVelocity_t wheel_vel) {
    RobotVelocity_t robot_vel;
    
    float r = WHEEL_RADIUS;
    float factor = r / 4.0f;
    
    // Vx = r/4 * (V_fl + V_fr + V_rl + V_rr)
    robot_vel.Vx = factor * (wheel_vel.fl + wheel_vel.fr + 
                             wheel_vel.rl + wheel_vel.rr);
    
    // Vy = r/4 * (-V_fl + V_fr + V_rl - V_rr)
    robot_vel.Vy = factor * (-wheel_vel.fl + wheel_vel.fr + 
                             wheel_vel.rl - wheel_vel.rr);
    
    // ω = r/(4L) * (-V_fl + V_fr - V_rl + V_rr)
    robot_vel.omega = factor / L * (-wheel_vel.fl + wheel_vel.fr + 
                                    -wheel_vel.rl + wheel_vel.rr);
    
    return robot_vel;
}
```

### 从RPM计算

```c
// RPM转线速度
float rpm_to_velocity(float rpm) {
    // V(m/s) = RPM × 2πr / 60
    return rpm * 2.0f * 3.14159f * WHEEL_RADIUS / 60.0f;
}

// 从四个轮子的RPM计算机器人速度
RobotVelocity_t get_robot_velocity(float actual_rpm[4]) {
    WheelVelocity_t wheel_vel;
    
    wheel_vel.fl = rpm_to_velocity(actual_rpm[0]);
    wheel_vel.fr = rpm_to_velocity(actual_rpm[1]);
    wheel_vel.rl = rpm_to_velocity(actual_rpm[2]);
    wheel_vel.rr = rpm_to_velocity(actual_rpm[3]);
    
    return inverse_kinematics(wheel_vel);
}
```

---

## 传感器融合

### 问题

从轮速推算的机器人速度可能有误差：
- 轮子打滑
- 地面不平
- 编码器误差

### 解决方法：传感器融合

**ω（旋转角速度）**：用陀螺仪测量

```c
// 使用MPU6050陀螺仪
float get_omega_gyro(void) {
    // 读取陀螺仪Z轴角速度
    return mpu6050_get_gyro_z();
}
```

**Vx（前进速度）**：用轮速平均

```c
// Vx用轮速平均（相对可靠）
float get_vx_wheels(float actual_rpm[4]) {
    RobotVelocity_t vel = get_robot_velocity(actual_rpm);
    return vel.Vx;
}
```

**Vy（平移速度）**：通常不控制

```c
// Vy噪声大，通常设为0
float get_vy(void) {
    return 0;  // 不控制
}
```

### 融合后的机器人速度

```c
RobotVelocity_t get_robot_velocity_fused(float actual_rpm[4]) {
    RobotVelocity_t vel;
    
    // Vx用轮速
    vel.Vx = get_vx_wheels(actual_rpm);
    
    // Vy不控制
    vel.Vy = 0;
    
    // ω用陀螺仪
    vel.omega = get_omega_gyro();
    
    return vel;
}
```

---

## 验证方法

### 正逆运动学验证

正向和逆向应该互为逆运算：

```c
// 验证
RobotVelocity_t target = {0.5, 0.0, 0.0};  // 前进0.5m/s

// 正向：机器人速度 → 轮速
WheelVelocity_t wheel = forward_kinematics(target);

// 逆向：轮速 → 机器人速度
RobotVelocity_t result = inverse_kinematics(wheel);

// 检查
printf("Vx: %.2f -> %.2f\n", target.Vx, result.Vx);
printf("Vy: %.2f -> %.2f\n", target.Vy, result.Vy);
printf("omega: %.2f -> %.2f\n", target.omega, result.omega);

// 应该相等
```

---

## 本节要点

1. **逆向运动学**：轮子速度 → 机器人速度
2. **用途**：外环控制需要知道机器人实际速度
3. **公式**：Vx是平均，Vy是差分，ω是差分
4. **传感器融合**：ω用陀螺仪，Vx用轮速，Vy不控制
5. **验证**：正逆运动学应该互为逆运算

---

下一节：[典型运动模式](patterns.md) - 学习前进、平移、旋转的轮速配合

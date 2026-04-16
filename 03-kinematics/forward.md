# 正向运动学解算

> 机器人速度 → 轮子速度

---

## 什么是正向运动学？

**正向运动学**：已知机器人的目标速度（Vx, Vy, ω），计算四个轮子应该转多快。

```
输入：Vx, Vy, ω（机器人目标速度）
输出：V_fl, V_fr, V_rl, V_rr（四个轮子的线速度）
```

---

## 麦氏轮结构

### 轮子布局

```
        前进方向 (X轴)
            ↑
            │
    FL ╲    │    ╱ FR
         ╲  │  ╱
    ────────┼───────  ← Y轴（右方向）
         ╱  │  ╲
    RL ╱    │    ╲ RR
            │

FL: 左前轮（Left Front）
FR: 右前轮（Right Front）
RL: 左后轮（Rear Left）
RR: 右后轮（Rear Right）
```

### 轮子朝向

麦氏轮的辊子呈45°倾斜：

```
FL轮：辊子朝向 ╲（左后-右前方向）
FR轮：辊子朝向 ╱（左前-右后方向）
RL轮：辊子朝向 ╱（左前-右后方向）
RR轮：辊子朝向 ╲（左后-右前方向）
```

这种布局使得机器人可以全向移动。

---

## 运动学公式

### 矩阵形式

将机器人速度映射到四个轮子速度的矩阵方程：

$$
\begin{bmatrix} 
V_{fl} \\ V_{fr} \\ V_{rl} \\ V_{rr} 
\end{bmatrix} 
= \frac{1}{r} 
\begin{bmatrix} 
1 & -1 & -(l_x + l_y) \\ 
1 & 1 & (l_x + l_y) \\ 
1 & 1 & -(l_x + l_y) \\ 
1 & -1 & (l_x + l_y) 
\end{bmatrix} 
\begin{bmatrix} 
V_x \\ V_y \\ \omega 
\end{bmatrix}
$$

### 展开形式

逐轮展开后的速度计算公式：

| 轮子 | 公式 |
|:----:|:-----|
| 左前轮 (FL) | $V_{fl} = \frac{1}{r}(V_x - V_y - \omega(l_x + l_y))$ |
| 右前轮 (FR) | $V_{fr} = \frac{1}{r}(V_x + V_y + \omega(l_x + l_y))$ |
| 左后轮 (RL) | $V_{rl} = \frac{1}{r}(V_x + V_y - \omega(l_x + l_y))$ |
| 右后轮 (RR) | $V_{rr} = \frac{1}{r}(V_x - V_y + \omega(l_x + l_y))$ |

### 参数说明

| 符号 | 含义 | 单位 |
|------|------|------|
| V_fl, V_fr, V_rl, V_rr | 四个轮子的线速度 | m/s |
| Vx | 机器人前进速度 | m/s |
| Vy | 机器人平移速度 | m/s |
| ω | 机器人旋转角速度 | rad/s |
| r | 轮子半径 | m |
| lx | 轮子到中心的X方向距离 | m |
| ly | 轮子到中心的Y方向距离 | m |

### 几何参数

```
        lx
    ◄────────►
    ┌────────┐
    │  FL  FR│  ▲
    │        │  │
    │        │  ly
    │        │  │
    │  RL  RR│  ▼
    └────────┘
```

通常定义：$L = l_x + l_y$

---

## 公式推导

### 推导思路

1. 机器人速度分解到每个轮子
2. 考虑轮子辊子的朝向
3. 考虑旋转产生的切向速度

### 示例：前进运动

```
机器人前进 Vx，Vy=0，ω=0

V_fl = Vx/r
V_fr = Vx/r
V_rl = Vx/r
V_rr = Vx/r

结论：四个轮子同向同速
```

### 示例：平移运动

```
机器人向右平移 Vy，Vx=0，ω=0

V_fl = -Vy/r
V_fr = +Vy/r
V_rl = +Vy/r
V_rr = -Vy/r

结论：对角轮子同向，两侧反向
```

### 示例：旋转运动

```
机器人原地旋转 ω，Vx=0，Vy=0

V_fl = -ωL/r
V_fr = +ωL/r
V_rl = -ωL/r
V_rr = +ωL/r

结论：左侧轮子反向，右侧轮子正向
```

---

## 代码实现

### 结构体定义

```c
typedef struct {
    float Vx;    // 前进速度 (m/s)
    float Vy;    // 平移速度 (m/s)
    float omega; // 旋转角速度 (rad/s)
} RobotVelocity_t;

typedef struct {
    float fl;    // 左前轮线速度 (m/s)
    float fr;    // 右前轮线速度 (m/s)
    float rl;    // 左后轮线速度 (m/s)
    float rr;    // 右后轮线速度 (m/s)
} WheelVelocity_t;
```

### 正向运动学函数

```c
// 机器人参数
#define WHEEL_RADIUS  0.05f   // 轮子半径 5cm
#define LX            0.15f   // X方向距离 15cm
#define LY            0.15f   // Y方向距离 15cm
#define L             (LX + LY)  // L = 0.3m

// 正向运动学解算
WheelVelocity_t forward_kinematics(RobotVelocity_t robot_vel) {
    WheelVelocity_t wheel_vel;
    
    float factor = 1.0f / WHEEL_RADIUS;
    float L_omega = L * robot_vel.omega;
    
    wheel_vel.fl = factor * (robot_vel.Vx - robot_vel.Vy - L_omega);
    wheel_vel.fr = factor * (robot_vel.Vx + robot_vel.Vy + L_omega);
    wheel_vel.rl = factor * (robot_vel.Vx + robot_vel.Vy - L_omega);
    wheel_vel.rr = factor * (robot_vel.Vx - robot_vel.Vy + L_omega);
    
    return wheel_vel;
}
```

### 线速度转转速

```c
// 线速度转RPM
float velocity_to_rpm(float velocity) {
    // RPM = V(m/s) / (2πr) × 60
    return velocity * 60.0f / (2.0f * 3.14159f * WHEEL_RADIUS);
}

// 获取四个轮子的目标RPM
void get_target_rpm(RobotVelocity_t robot_vel, float target_rpm[4]) {
    WheelVelocity_t wheel_vel = forward_kinematics(robot_vel);
    
    target_rpm[0] = velocity_to_rpm(wheel_vel.fl);
    target_rpm[1] = velocity_to_rpm(wheel_vel.fr);
    target_rpm[2] = velocity_to_rpm(wheel_vel.rl);
    target_rpm[3] = velocity_to_rpm(wheel_vel.rr);
}
```

---

## 参数校准

### 为什么需要校准？

理论参数和实际参数可能有差异：
- 轮子半径测量误差
- 轮距测量误差
- 轮子安装角度误差

### 校准方法

**方法1：前进测试**

```
1. 设置 Vx = 0.5 m/s，Vy = 0，ω = 0
2. 让机器人前进1米
3. 测量实际距离
4. 如果实际距离偏小，增大轮子半径参数
```

**方法2：旋转测试**

```
1. 设置 Vx = 0，Vy = 0，ω = 1 rad/s
2. 让机器人旋转360°
3. 测量实际角度
4. 如果实际角度偏小，增大L参数
```

---

## 本节要点

1. **正向运动学**：机器人速度 → 轮子速度
2. **公式核心**：考虑Vx、Vy、ω三者的贡献
3. **几何参数**：轮子半径r、轮距L需要实测
4. **参数校准**：通过实际测试校准参数
5. **代码实现**：先算线速度，再转RPM

---

下一节：[逆向运动学解算](inverse.md) - 学习如何从轮速计算机器人速度

# 典型运动模式

> 前进、平移、旋转的轮速配合

---

## 概述

麦氏轮机器人有三种基本运动模式：

| 模式 | 速度设置 | 特点 |
|------|---------|------|
| 前进/后退 | Vx ≠ 0, Vy = 0, ω = 0 | 四轮同向同速 |
| 左右平移 | Vx = 0, Vy ≠ 0, ω = 0 | 对角同向，两侧反向 |
| 原地旋转 | Vx = 0, Vy = 0, ω ≠ 0 | 左侧反向，右侧正向 |

任何复杂运动都可以分解为这三种基本运动的叠加。

---

## 前进/后退运动

### 速度设置

```
Vx = 0.5 m/s（前进）
Vy = 0
ω = 0
```

### 轮速计算

```
V_fl = Vx = 0.5 m/s
V_fr = Vx = 0.5 m/s
V_rl = Vx = 0.5 m/s
V_rr = Vx = 0.5 m/s
```

### 轮子配合

```
        前进方向
            ↑
            │
    FL ↑    │    ↑ FR
            │
    ────────┼───────
            │
    RL ↑    │    ↑ RR
            │

四个轮子同向同速
```

### 代码

```c
// 前进运动
void move_forward(float speed) {
    RobotVelocity_t vel = {speed, 0, 0};
    WheelVelocity_t wheel = forward_kinematics(vel);
    set_wheel_velocity(wheel);
}
```

---

## 左右平移运动

### 速度设置

```
Vx = 0
Vy = 0.3 m/s（向右平移）
ω = 0
```

### 轮速计算

```
V_fl = -Vy = -0.3 m/s
V_fr = +Vy = +0.3 m/s
V_rl = +Vy = +0.3 m/s
V_rr = -Vy = -0.3 m/s
```

### 轮子配合

```
        前进方向
            ↑
            │
    FL ↓    │    ↑ FR
            │
    ────────┼─────── → 平移方向
            │
    RL ↑    │    ↓ RR
            │

对角轮子同向，两侧反向
形成X型交叉
```

### 代码

```c
// 向右平移
void move_right(float speed) {
    RobotVelocity_t vel = {0, speed, 0};
    WheelVelocity_t wheel = forward_kinematics(vel);
    set_wheel_velocity(wheel);
}

// 向左平移
void move_left(float speed) {
    RobotVelocity_t vel = {0, -speed, 0};
    WheelVelocity_t wheel = forward_kinematics(vel);
    set_wheel_velocity(wheel);
}
```

---

## 原地旋转运动

### 速度设置

```
Vx = 0
Vy = 0
ω = 1 rad/s（顺时针旋转）
```

### 轮速计算

```
V_fl = -ωL = -0.3 m/s（假设L=0.3m）
V_fr = +ωL = +0.3 m/s
V_rl = -ωL = -0.3 m/s
V_rr = +ωL = +0.3 m/s
```

### 轮子配合

```
        前进方向
            ↑
            │
    FL ↓    │    ↑ FR
            │
    ────────┼───────
            │
    RL ↓    │    ↑ RR
            │

左侧轮子反向，右侧轮子正向
形成旋转力矩
```

### 代码

```c
// 顺时针旋转
void rotate_cw(float omega) {
    RobotVelocity_t vel = {0, 0, omega};
    WheelVelocity_t wheel = forward_kinematics(vel);
    set_wheel_velocity(wheel);
}

// 逆时针旋转
void rotate_ccw(float omega) {
    RobotVelocity_t vel = {0, 0, -omega};
    WheelVelocity_t wheel = forward_kinematics(vel);
    set_wheel_velocity(wheel);
}
```

---

## 组合运动

### 前进+旋转

```
Vx = 0.5 m/s
Vy = 0
ω = 0.5 rad/s

效果：边前进边转弯（弧线运动）
```

### 平移+旋转

```
Vx = 0
Vy = 0.3 m/s
ω = 0.5 rad/s

效果：边平移边旋转
```

### 前进+平移+旋转

```
Vx = 0.5 m/s
Vy = 0.3 m/s
ω = 0.5 rad/s

效果：任意方向运动
```

### 代码

```c
// 任意运动
void move_robot(float Vx, float Vy, float omega) {
    RobotVelocity_t vel = {Vx, Vy, omega};
    WheelVelocity_t wheel = forward_kinematics(vel);
    set_wheel_velocity(wheel);
}
```

---

## 运动模式验证

### 验证方法

通过观察轮速判断运动模式：

```c
void print_wheel_velocity(WheelVelocity_t wheel) {
    printf("FL: %.2f  FR: %.2f\n", wheel.fl, wheel.fr);
    printf("RL: %.2f  RR: %.2f\n", wheel.rl, wheel.rr);
    
    // 判断运动模式
    if (wheel.fl == wheel.fr && wheel.fr == wheel.rl && wheel.rl == wheel.rr) {
        printf("模式：前进/后退\n");
    } else if (wheel.fl == -wheel.fr && wheel.fr == wheel.rl && wheel.rl == -wheel.rr) {
        printf("模式：左右平移\n");
    } else if (wheel.fl == -wheel.fr && wheel.fr == -wheel.rr && wheel.rr == wheel.rl) {
        printf("模式：原地旋转\n");
    } else {
        printf("模式：组合运动\n");
    }
}
```

---

## 实际应用

### 路径跟踪

```c
// 跟踪直线
void follow_line(float distance) {
    float Vx = 0.5;  // 前进速度
    float time = distance / Vx;
    
    move_forward(Vx);
    delay(time);
    stop();
}

// 跟踪圆弧
void follow_arc(float radius, float angle) {
    float Vx = 0.5;  // 前进速度
    float omega = Vx / radius;  // 角速度
    float time = angle / omega;
    
    move_robot(Vx, 0, omega);
    delay(time);
    stop();
}
```

### 避障

```c
// 向左避障
void avoid_obstacle_left(void) {
    // 1. 向左平移
    move_left(0.3);
    delay(1000);
    
    // 2. 前进
    move_forward(0.5);
    delay(2000);
    
    // 3. 向右平移回到原路径
    move_right(0.3);
    delay(1000);
    
    stop();
}
```

---

## 常见问题

### 问题1：平移时轨迹弯曲

**原因**：
- 运动学参数不准确
- 四个轮子转速不一致

**解决**：
- 校准L和r参数
- 单独调整每个轮子的PID参数

### 问题2：旋转时偏离中心

**原因**：
- 左右轮速不对称
- 重心不在几何中心

**解决**：
- 校准轮速
- 调整旋转中心

### 问题3：前进时跑偏

**原因**：
- 左右轮速不一致
- 轮子安装角度有差异

**解决**：
- 校准每个轮子的PID
- 检查机械安装

---

## 本节要点

1. **三种基本运动**：前进、平移、旋转
2. **前进**：四轮同向同速
3. **平移**：对角同向，两侧反向
4. **旋转**：左侧反向，右侧正向
5. **组合运动**：三种基本运动的叠加
6. **参数校准**：确保运动轨迹准确

---

麦氏轮运动学模块完成！下一模块：[双层闭环控制](../04-dual-loop/README.md)

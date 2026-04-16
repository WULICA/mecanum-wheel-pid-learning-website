# 外环整体控制

> 机器人整体速度PID控制

---

## 什么是外环？

**外环**：控制机器人整体的运动速度，是决策层。

```
输入：目标机器人速度 target_Vx, target_Vy, target_ω
反馈：实际机器人速度 current_Vx, current_Vy, current_ω
输出：目标轮速修正量
```

---

## 外环的作用

### 决策层

外环根据机器人整体速度的误差，决定每个轮子应该转多快。

```
目标：机器人前进 0.5 m/s
实际：机器人前进 0.48 m/s
外环：误差 0.02 m/s，需要加速
外环决策：把目标轮速从 100 RPM 提高到 105 RPM
```

### 整体控制

外环关注的是机器人整体的运动，不是单个轮子。

```
内环：让左前轮转到 105 RPM
外环：让机器人前进 0.5 m/s
```

---

## 外环结构

### 控制框图

```
target_Vx ──→ (+) ──→ PID ──→ 运动学 ──→ target_rpm[4] ──→ 内环
              ↑                                      │
              │                                      │
              └←── 逆运动学 ←── actual_rpm[4] ←──────┘
```

### 执行流程

```
每50ms执行一次：
1. 读取四个轮子的实际转速 actual_rpm[4]
2. 逆运动学解算 → current_Vx, current_Vy, current_ω
3. 计算误差 → error_Vx = target_Vx - current_Vx
4. PID计算 → 修正量 dVx
5. 修正后的速度 → corrected_Vx = target_Vx + dVx
6. 正运动学解算 → target_rpm[4]
7. 内环跟踪新的目标
```

---

## 外环PID参数

### 典型参数

| 参数 | 典型范围 | 说明 |
|------|---------|------|
| Kp | 0.05 ~ 0.5 | 比内环小 |
| Ki | 0.001 ~ 0.05 | 比内环小 |
| Kd | 0 ~ 0.05 | 通常不用 |
| 周期 | 50ms | 比内环长 |

### 参数特点

**Kp较小**：外环响应要平滑，不能太激进

**Ki很小**：外环积分要慢，避免振荡

**周期较长**：外环要等内环稳定后再调整

### 与内环的关系

```
外环Kp ≈ 内环Kp / 10
外环周期 = 内环周期 × 5~10
```

---

## 代码实现

### 外环PID结构体

```c
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float target;
    float actual;
    float error;
    float last_error;
    float integral;
    
    float output;
} OuterLoopPID_t;
```

### 外环控制结构

```c
typedef struct {
    OuterLoopPID_t pid_Vx;
    OuterLoopPID_t pid_Vy;
    OuterLoopPID_t pid_omega;
    
    float target_Vx;
    float target_Vy;
    float target_omega;
    
    float current_Vx;
    float current_Vy;
    float current_omega;
} OuterLoop_t;
```

### 外环初始化

```c
OuterLoop_t outer_loop;

void outer_loop_init(void) {
    // Vx PID
    outer_loop.pid_Vx.Kp = 0.1f;
    outer_loop.pid_Vx.Ki = 0.01f;
    outer_loop.pid_Vx.Kd = 0.0f;
    
    // Vy PID（通常不控制）
    outer_loop.pid_Vy.Kp = 0.0f;
    outer_loop.pid_Vy.Ki = 0.0f;
    outer_loop.pid_Vy.Kd = 0.0f;
    
    // ω PID
    outer_loop.pid_omega.Kp = 0.1f;
    outer_loop.pid_omega.Ki = 0.01f;
    outer_loop.pid_omega.Kd = 0.0f;
}
```

### 外环执行函数

```c
// 外环更新
void outer_loop_update(void) {
    // 1. 获取机器人当前速度
    RobotVelocity_t current = get_robot_velocity_fused(actual_rpm);
    outer_loop.current_Vx = current.Vx;
    outer_loop.current_Vy = current.Vy;
    outer_loop.current_omega = current.omega;
    
    // 2. 计算误差
    float error_Vx = outer_loop.target_Vx - outer_loop.current_Vx;
    float error_Vy = outer_loop.target_Vy - outer_loop.current_Vy;
    float error_omega = outer_loop.target_omega - outer_loop.current_omega;
    
    // 3. PID计算修正量
    float dVx = pid_update(&outer_loop.pid_Vx, 
                          outer_loop.target_Vx, 
                          outer_loop.current_Vx, 
                          0.05f);  // dt = 50ms
    
    float dVy = pid_update(&outer_loop.pid_Vy, 
                           outer_loop.target_Vy, 
                           outer_loop.current_Vy, 
                           0.05f);
    
    float d_omega = pid_update(&outer_loop.pid_omega, 
                               outer_loop.target_omega, 
                               outer_loop.current_omega, 
                               0.05f);
    
    // 4. 修正后的速度
    float corrected_Vx = outer_loop.target_Vx + dVx;
    float corrected_Vy = outer_loop.target_Vy + dVy;
    float corrected_omega = outer_loop.target_omega + d_omega;
    
    // 5. 正运动学解算
    RobotVelocity_t corrected = {corrected_Vx, corrected_Vy, corrected_omega};
    WheelVelocity_t wheel = forward_kinematics(corrected);
    
    // 6. 更新内环目标
    target_rpm[0] = velocity_to_rpm(wheel.fl);
    target_rpm[1] = velocity_to_rpm(wheel.fr);
    target_rpm[2] = velocity_to_rpm(wheel.rl);
    target_rpm[3] = velocity_to_rpm(wheel.rr);
}
```

### 外环定时器中断

```c
// 50ms定时器中断
void TIM_OuterLoop_IRQHandler(void) {
    outer_loop_update();
    
    // 清除中断标志
    TIM->SR &= ~TIM_SR_UIF;
}
```

---

## 外环调参

### 前提条件

**内环必须先调好**：内环能稳定跟踪目标轮速。

### 调参步骤

**步骤1：设置外环周期**

```
外环周期 = 内环周期 × 5~10
内环周期 = 10ms
外环周期 = 50ms
```

**步骤2：设置初始参数**

```
外环Kp = 内环Kp / 10 = 1.0 / 10 = 0.1
外环Ki = 内环Ki / 10 = 0.05 / 10 = 0.005
```

**步骤3：前进测试**

```
1. 设置 target_Vx = 0.2 m/s
2. 观察 current_Vx 是否跟踪 target_Vx
3. 如果跟踪慢，增大Kp
4. 如果有静差，增大Ki
5. 如果振荡，减小Kp和Ki
```

**步骤4：旋转测试**

```
1. 设置 target_omega = 0.5 rad/s
2. 观察 current_omega 是否跟踪
3. 调整 ω 的PID参数
```

---

## 外环与内环的配合

### 时序关系

```
时间轴 →

0ms     10ms    20ms    30ms    40ms    50ms
│       │       │       │       │       │
内环    内环    内环    内环    内环    内环
│       │       │       │       │       │
└───────┴───────┴───────┴───────┴───────┘
                                        │
                                       外环
                                        │
                                 更新target_rpm
                                        │
┌───────────────────────────────────────┘
│
内环读取新的target_rpm，继续跟踪
```

### 关键理解

```
内环：永远在追 target_rpm 这个目标
外环：每50ms重新设定这个目标值

外环设定目标的原则：
  如果实际速度慢了，就把目标调高
  如果实际速度快了，就把目标调低
```

---

## 本节要点

1. **外环作用**：控制机器人整体速度（决策层）
2. **外环周期**：50ms，比内环长
3. **外环参数**：比内环小，响应平滑
4. **调参前提**：内环必须先调好
5. **配合关系**：外环设定目标，内环跟踪目标

---

下一节：[数据流与时序](data-flow.md) - 学习内外环的数据传递和执行时序

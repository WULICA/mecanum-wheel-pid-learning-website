# 数据流与时序

> 内外环的数据传递和执行时序

---

## 数据流概览

### 完整数据流

```
用户输入
  │
  ▼
target_Vx, target_Vy, target_ω
  │
  ▼
┌─────────────────────────────────┐
│         外环（50ms）             │
│                                 │
│ 1. 读取 actual_rpm[4]           │
│ 2. 逆运动学 → current_Vx...     │
│ 3. PID计算修正量                │
│ 4. 正运动学 → target_rpm[4]     │
└─────────────────────────────────┘
  │
  ▼
target_rpm[4]（全局变量）
  │
  ▼
┌─────────────────────────────────┐
│         内环（10ms）             │
│                                 │
│ 1. 读取编码器 → actual_rpm[4]   │
│ 2. PID计算 → output[4]          │
│ 3. 输出PWM → 电机转动           │
└─────────────────────────────────┘
  │
  ▼
电机转动 → 编码器反馈
  │
  └→ actual_rpm[4]（全局变量）
```

---

## 全局变量

### 数据桥梁

全局变量是外环和内环之间的数据桥梁。

```c
// 外环写入，内环读取
volatile float target_rpm[4];   // 四个轮子的目标转速

// 内环写入，外环读取
volatile float actual_rpm[4];   // 四个轮子的实际转速
```

### volatile关键字

使用 `volatile` 防止编译器优化，确保数据实时更新。

```c
// 正确
volatile float target_rpm[4];

// 错误（可能被优化）
float target_rpm[4];
```

---

## 执行时序

### 时序图

```
时间轴 →

0ms      10ms     20ms     30ms     40ms     50ms     60ms
│        │        │        │        │        │        │
│        ▼        ▼        ▼        ▼        ▼        ▼
│      内环     内环     内环     内环     内环     内环
│        │        │        │        │        │        │
│        │        │        │        │        │        │
│     读target  读target  读target  读target  读target  读target
│     算PID     算PID     算PID     算PID     算PID     算PID
│     输出PWM   输出PWM   输出PWM   输出PWM   输出PWM   输出PWM
│        │        │        │        │        │        │
│        └────────┴────────┴────────┴────────┴────────┘
│                                             │
│                                            外环
│                                             │
│                                      读actual_rpm
│                                      算current_Vx
│                                      算修正量
│                                      更新target_rpm
│                                             │
└─────────────────────────────────────────────┘
```

### 时序说明

```
内环：每10ms执行一次，持续运行
外环：每50ms执行一次，更新目标

内环永远在追 target_rpm 这个目标
外环每50ms重新设定这个目标值
```

---

## 详细执行流程

### 外环执行流程（50ms一次）

```c
void outer_loop_execute(void) {
    // ========== 步骤1：读取实际轮速 ==========
    // actual_rpm[4] 由内环更新，外环直接读取
    float rpm_fl = actual_rpm[0];
    float rpm_fr = actual_rpm[1];
    float rpm_rl = actual_rpm[2];
    float rpm_rr = actual_rpm[3];
    
    // ========== 步骤2：逆运动学解算 ==========
    // 轮速 → 线速度
    float v_fl = rpm_to_velocity(rpm_fl);
    float v_fr = rpm_to_velocity(rpm_fr);
    float v_rl = rpm_to_velocity(rpm_rl);
    float v_rr = rpm_to_velocity(rpm_rr);
    
    // 线速度 → 机器人速度
    float current_Vx = (v_fl + v_fr + v_rl + v_rr) / 4.0f;
    float current_Vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0f;
    float current_omega = get_omega_gyro();  // 用陀螺仪
    
    // ========== 步骤3：计算误差 ==========
    float error_Vx = target_Vx - current_Vx;
    float error_omega = target_omega - current_omega;
    
    // ========== 步骤4：PID计算修正量 ==========
    float dVx = pid_Vx.Kp * error_Vx + pid_Vx.Ki * integral_Vx;
    float d_omega = pid_omega.Kp * error_omega + pid_omega.Ki * integral_omega;
    
    // ========== 步骤5：修正后的速度 ==========
    float corrected_Vx = target_Vx + dVx;
    float corrected_Vy = 0;  // Vy不控制
    float corrected_omega = target_omega + d_omega;
    
    // ========== 步骤6：正运动学解算 ==========
    // 机器人速度 → 轮速
    float corrected_v_fl = corrected_Vx - corrected_Vy - corrected_omega * L;
    float corrected_v_fr = corrected_Vx + corrected_Vy + corrected_omega * L;
    float corrected_v_rl = corrected_Vx + corrected_Vy - corrected_omega * L;
    float corrected_v_rr = corrected_Vx - corrected_Vy + corrected_omega * L;
    
    // ========== 步骤7：更新内环目标 ==========
    target_rpm[0] = velocity_to_rpm(corrected_v_fl);
    target_rpm[1] = velocity_to_rpm(corrected_v_fr);
    target_rpm[2] = velocity_to_rpm(corrected_v_rl);
    target_rpm[3] = velocity_to_rpm(corrected_v_rr);
}
```

### 内环执行流程（10ms一次）

```c
void inner_loop_execute(void) {
    for (int i = 0; i < 4; i++) {
        // ========== 步骤1：读取编码器 ==========
        actual_rpm[i] = read_encoder_rpm(i);
        
        // ========== 步骤2：计算误差 ==========
        float error = target_rpm[i] - actual_rpm[i];
        
        // ========== 步骤3：PID计算 ==========
        integral[i] += error * 0.01f;
        float output = Kp * error + Ki * integral[i];
        
        // ========== 步骤4：输出PWM ==========
        int pwm = base_pwm + (int)output;
        pwm = CLAMP(pwm, 0, 100);
        set_motor_pwm(i, pwm);
    }
}
```

---

## 数据流示例

### 示例：前进运动

```
初始状态：
  target_Vx = 0.5 m/s
  current_Vx = 0 m/s

第1次外环（0ms）：
  current_Vx = 0
  error_Vx = 0.5 - 0 = 0.5
  dVx = 0.1 × 0.5 = 0.05
  corrected_Vx = 0.5 + 0.05 = 0.55
  target_rpm = {110, 110, 110, 110}  // 略高于目标

内环（0~50ms）：
  轮子加速，actual_rpm 上升到 100, 102, 104...

第2次外环（50ms）：
  current_Vx = 0.48  // 从轮速推算
  error_Vx = 0.5 - 0.48 = 0.02
  dVx = 0.1 × 0.02 = 0.002
  corrected_Vx = 0.5 + 0.002 = 0.502
  target_rpm = {101, 101, 101, 101}  // 接近目标

内环（50~100ms）：
  轮速稳定在 100 RPM 左右

第3次外环（100ms）：
  current_Vx ≈ 0.5
  error_Vx ≈ 0
  dVx ≈ 0
  corrected_Vx ≈ 0.5
  target_rpm ≈ {100, 100, 100, 100}

稳定状态：
  current_Vx ≈ target_Vx
```

---

## 周期关系

### 为什么外环周期要比内环长？

```
内环周期短（10ms）：
  - 快速响应
  - 让轮子快速达到目标

外环周期长（50ms）：
  - 等待内环稳定
  - 避免频繁调整目标
  - 减少振荡

如果外环周期太短：
  - 内环还没稳定，外环就调整目标
  - 系统容易振荡
```

### 周期比例

```
外环周期 / 内环周期 = 5~10

例如：
  内环周期 = 10ms
  外环周期 = 50ms
  比例 = 5
```

---

## 本节要点

1. **全局变量**：target_rpm 和 actual_rpm 是数据桥梁
2. **外环流程**：读轮速 → 逆解算 → PID → 正解算 → 更新目标
3. **内环流程**：读编码器 → PID → 输出PWM
4. **时序关系**：内环持续运行，外环周期性更新目标
5. **周期比例**：外环周期 = 内环周期 × 5~10

---

下一节：[传感器选择](sensors.md) - 学习如何获取机器人的实际速度

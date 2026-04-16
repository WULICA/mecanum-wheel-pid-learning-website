# 执行流程详解

> PID每个周期做什么

---

## 执行周期

PID控制是周期性执行的，每个周期执行一次计算。

### 周期选择

| 应用 | 典型周期 | 说明 |
|------|---------|------|
| 速度环 | 10~20ms | 响应要快 |
| 位置环 | 20~50ms | 可以慢一点 |
| 温度环 | 100~1000ms | 温度变化慢 |

**麦氏轮速度环**：通常选择 **10ms**。

---

## 执行流程

### 流程图

```
┌─────────────────────────────────────┐
│         PID执行流程（每10ms）         │
└─────────────────────────────────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 1. 读取编码器    │
        │   current_rpm   │
        └────────┬────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 2. 计算误差      │
        │ error = target  │
        │       - current │
        └────────┬────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 3. 计算P项       │
        │ P = Kp × error  │
        └────────┬────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 4. 更新积分      │
        │ integral +=     │
        │   error × dt    │
        └────────┬────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 5. 计算I项       │
        │ I = Ki ×        │
        │     integral    │
        └────────┬────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 6. 计算D项       │
        │ D = Kd ×        │
        │   (error -      │
        │    last_error)  │
        └────────┬────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 7. 计算总输出    │
        │ output = P+I+D  │
        └────────┬────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 8. 输出限幅      │
        │ output = clamp  │
        │   (output, min, │
        │    max)         │
        └────────┬────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 9. 输出到PWM     │
        │ PWM = base +    │
        │       output    │
        └────────┬────────┘
                  │
                  ▼
        ┌─────────────────┐
        │ 10. 保存误差     │
        │ last_error =    │
        │    error        │
        └─────────────────┘
```

---

## 详细步骤

### 步骤1：读取编码器

```c
// 读取编码器计数值
int16_t count = TIM2->CNT;

// 计算转速
int16_t delta = count - last_count;
current_rpm = delta * 6.0f;  // PPR=1000, dt=10ms

// 保存
last_count = count;
```

### 步骤2：计算误差

```c
// 误差 = 目标 - 实际
error = target_rpm - current_rpm;

// 示例
// target_rpm = 100
// current_rpm = 85
// error = 100 - 85 = 15
```

### 步骤3：计算P项

```c
// P项 = Kp × 误差
P_out = Kp * error;

// 示例
// Kp = 1.0
// error = 15
// P_out = 1.0 × 15 = 15
```

### 步骤4：更新积分

```c
// 积分累积
integral += error * dt;

// 示例
// error = 15
// dt = 0.01s
// integral = integral + 15 × 0.01 = integral + 0.15

// 积分限幅
if (integral > integral_max) {
    integral = integral_max;
} else if (integral < -integral_max) {
    integral = -integral_max;
}
```

### 步骤5：计算I项

```c
// I项 = Ki × 积分
I_out = Ki * integral;

// 示例
// Ki = 0.05
// integral = 3.0
// I_out = 0.05 × 3.0 = 0.15
```

### 步骤6：计算D项

```c
// D项 = Kd × (误差变化 / dt)
D_out = Kd * (error - last_error) / dt;

// 示例
// Kd = 0（速度环通常不用）
// D_out = 0
```

### 步骤7：计算总输出

```c
// 总输出 = P + I + D
output = P_out + I_out + D_out;

// 示例
// P_out = 15
// I_out = 0.15
// D_out = 0
// output = 15 + 0.15 + 0 = 15.15
```

### 步骤8：输出限幅

```c
// 限制输出范围
if (output > output_max) {
    output = output_max;
} else if (output < -output_max) {
    output = -output_max;
}

// 示例
// output_max = 1000
// output = 15.15（在范围内，不变）
```

### 步骤9：输出到PWM

```c
// PWM = 基础值 + PID输出
pwm = base_pwm + output;

// 限制PWM范围
if (pwm > 100) {
    pwm = 100;
} else if (pwm < 0) {
    pwm = 0;
}

// 设置PWM
__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, pwm);
```

### 步骤10：保存误差

```c
// 保存当前误差，供下次计算D项使用
last_error = error;
```

---

## 完整代码

### PID结构体

```c
typedef struct {
    // 参数
    float Kp;
    float Ki;
    float Kd;
    
    // 状态
    float target;
    float actual;
    float error;
    float last_error;
    float integral;
    
    // 输出
    float output;
    
    // 限幅
    float integral_limit;
    float output_limit;
} PID_t;
```

### PID初始化

```c
void pid_init(PID_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    pid->target = 0;
    pid->actual = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
    
    pid->integral_limit = 1000;
    pid->output_limit = 1000;
}
```

### PID计算

```c
float pid_update(PID_t *pid, float target, float actual, float dt) {
    // 更新目标值和实际值
    pid->target = target;
    pid->actual = actual;
    
    // 计算误差
    pid->error = target - actual;
    
    // 更新积分
    pid->integral += pid->error * dt;
    
    // 积分限幅
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    // 计算各项
    float P = pid->Kp * pid->error;
    float I = pid->Ki * pid->integral;
    float D = pid->Kd * (pid->error - pid->last_error) / dt;
    
    // 总输出
    pid->output = P + I + D;
    
    // 输出限幅
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }
    
    // 保存误差
    pid->last_error = pid->error;
    
    return pid->output;
}
```

### 定时器中断

```c
// 10ms定时器中断
void TIM_IRQHandler(void) {
    // 读取编码器
    int16_t count = TIM2->CNT;
    int16_t delta = count - last_count;
    float current_rpm = delta * 6.0f;
    last_count = count;
    
    // PID计算
    float output = pid_update(&pid, target_rpm, current_rpm, 0.01f);
    
    // 输出到PWM
    int pwm = base_pwm + (int)output;
    pwm = CLAMP(pwm, 0, 100);
    __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, pwm);
    
    // 清除中断标志
    TIM->SR &= ~TIM_SR_UIF;
}
```

---

## 时序图

```
时间轴 →

0ms     10ms    20ms    30ms    40ms    50ms
│       │       │       │       │       │
▼       ▼       ▼       ▼       ▼       ▼
读编码器 读编码器 读编码器 读编码器 读编码器 读编码器
│       │       │       │       │       │
▼       ▼       ▼       ▼       ▼       ▼
算误差   算误差   算误差   算误差   算误差   算误差
│       │       │       │       │       │
▼       ▼       ▼       ▼       ▼       ▼
PID计算  PID计算  PID计算  PID计算  PID计算  PID计算
│       │       │       │       │       │
▼       ▼       ▼       ▼       ▼       ▼
输出PWM  输出PWM  输出PWM  输出PWM  输出PWM  输出PWM
```

---

## 本节要点

1. **执行周期**：速度环通常10ms
2. **执行流程**：读编码器 → 算误差 → PID计算 → 输出PWM
3. **积分限幅**：防止积分饱和
4. **输出限幅**：防止PWM超限
5. **保存误差**：供下次计算D项

---

PID控制原理模块完成！下一模块：[麦氏轮运动学](../03-kinematics/README.md)

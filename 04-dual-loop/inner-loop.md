# 内环速度控制

> 单轮速度PID控制

---

## 什么是内环？

**内环**：控制单个轮子的转速，是执行层。

```
输入：目标转速 target_rpm
反馈：实际转速 actual_rpm
输出：PWM调整量
```

---

## 内环的作用

### 执行层

内环负责让轮子达到外环设定的目标转速。

```
外环设定：target_rpm = 105
内环任务：让轮子转到 105 RPM
```

### 快速响应

内环周期短（10ms），能快速响应目标变化。

```
外环每50ms更新一次目标
内环每10ms执行一次
内环能快速跟踪外环的目标变化
```

---

## 内环结构

### 控制框图

```
target_rpm ──→ (+) ──→ PID ──→ PWM ──→ 电机 ──→ actual_rpm
                ↑                              │
                └──────────────────────────────┘
                         编码器反馈
```

### 执行流程

```
每10ms执行一次：
1. 读取编码器 → actual_rpm
2. 计算误差 → error = target_rpm - actual_rpm
3. PID计算 → output
4. 输出PWM → 电机转动
```

---

## 内环PID参数

### 典型参数

| 参数 | 典型范围 | 说明 |
|------|---------|------|
| Kp | 0.5 ~ 3.0 | 根据电机特性调整 |
| Ki | 0.01 ~ 0.2 | 消除静差 |
| Kd | 0 ~ 0.1 | 速度环通常不用 |
| 周期 | 10ms | 快速响应 |

### 参数特点

**Kp较大**：快速响应目标变化

**Ki适中**：消除静差，但不过调

**Kd通常为0**：速度信号噪声大，D项敏感

---

## 代码实现

### 内环PID结构体

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
    
    float integral_limit;
    float output_limit;
} InnerLoopPID_t;
```

### 四个轮子的内环

```c
// 四个轮子的内环PID
InnerLoopPID_t inner_pid[4];

// 初始化
void inner_loop_init(void) {
    for (int i = 0; i < 4; i++) {
        inner_pid[i].Kp = 1.0f;
        inner_pid[i].Ki = 0.05f;
        inner_pid[i].Kd = 0.0f;
        inner_pid[i].integral_limit = 1000;
        inner_pid[i].output_limit = 1000;
    }
}
```

### 内环执行函数

```c
// 内环PID计算
float inner_loop_update(InnerLoopPID_t *pid, float target, float actual) {
    pid->target = target;
    pid->actual = actual;
    
    // 误差
    pid->error = target - actual;
    
    // 积分
    pid->integral += pid->error * 0.01f;  // dt = 10ms
    
    // 积分限幅
    pid->integral = CLAMP(pid->integral, -pid->integral_limit, pid->integral_limit);
    
    // PID计算
    float P = pid->Kp * pid->error;
    float I = pid->Ki * pid->integral;
    float D = pid->Kd * (pid->error - pid->last_error) / 0.01f;
    
    pid->output = P + I + D;
    
    // 输出限幅
    pid->output = CLAMP(pid->output, -pid->output_limit, pid->output_limit);
    
    // 保存误差
    pid->last_error = pid->error;
    
    return pid->output;
}
```

### 内环定时器中断

```c
// 全局变量
volatile float target_rpm[4] = {0};  // 外环设定的目标转速
volatile float actual_rpm[4] = {0};  // 实际转速

// 10ms定时器中断
void TIM_InnerLoop_IRQHandler(void) {
    // 读取四个编码器
    for (int i = 0; i < 4; i++) {
        actual_rpm[i] = read_encoder_rpm(i);
    }
    
    // 四个内环PID计算
    for (int i = 0; i < 4; i++) {
        float output = inner_loop_update(&inner_pid[i], target_rpm[i], actual_rpm[i]);
        
        // 输出到PWM
        int pwm = base_pwm + (int)output;
        pwm = CLAMP(pwm, 0, 100);
        set_motor_pwm(i, pwm);
    }
    
    // 清除中断标志
    TIM->SR &= ~TIM_SR_UIF;
}
```

---

## 内环调参

### 调参步骤

**步骤1：单轮测试**

```
1. 固定其他轮，单独调一个电机
2. 给定 target_rpm = 100
3. 观察 actual_rpm 响应
4. 调 Kp 到响应快但无超调
5. 调 Ki 消除静差
```

**步骤2：四轮同步**

```
1. 设置 Vx = 0.2, Vy = 0, ω = 0
2. 四个轮子目标应相等
3. 观察四个轮子的 actual_rpm
4. 调参使四轮转速一致
```

**步骤3：旋转测试**

```
1. 设置 Vx = 0, Vy = 0, ω = 1
2. 对角轮子反向，速度应相等
3. 调参确保旋转平稳
```

### 调参技巧

**如果响应慢**：增大Kp

**如果有静差**：增大Ki

**如果振荡**：减小Kp和Ki

**如果四轮不一致**：单独调整每个轮子的参数

---

## 内环与外环的关系

### 目标来源

内环的 target_rpm 由外环设定：

```
外环：根据机器人速度误差，计算修正后的目标轮速
内环：让轮子达到这个目标轮速
```

### 数据传递

```c
// 外环写入，内环读取
volatile float target_rpm[4];  // 全局变量作为桥梁

// 外环更新目标
void outer_loop_update(void) {
    // ... 外环计算 ...
    target_rpm[0] = new_target_fl;
    target_rpm[1] = new_target_fr;
    target_rpm[2] = new_target_rl;
    target_rpm[3] = new_target_rr;
}

// 内环读取目标
void inner_loop_update(void) {
    for (int i = 0; i < 4; i++) {
        float output = pid_update(&inner_pid[i], target_rpm[i], actual_rpm[i]);
        // ...
    }
}
```

---

## 本节要点

1. **内环作用**：让轮子达到目标转速（执行层）
2. **内环周期**：10ms，快速响应
3. **内环参数**：Kp较大，Ki适中，Kd通常为0
4. **调参顺序**：先单轮，再四轮同步，最后旋转测试
5. **数据传递**：target_rpm 是外环和内环的桥梁

---

下一节：[外环整体控制](outer-loop.md) - 学习外环如何控制机器人整体速度

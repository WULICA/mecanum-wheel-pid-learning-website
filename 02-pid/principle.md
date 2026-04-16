# PID核心原理

> 理解P、I、D三个参数的物理意义

---

## 我们要解决什么问题？

### 场景描述

```
目标：让电机以 100 RPM 运行
实际：电机只有 85 RPM
原因：负载、摩擦、电压波动...
```

**问题**：如何让实际转速达到目标转速？

### 解决思路

```
1. 测量当前转速：85 RPM
2. 计算误差：100 - 85 = 15 RPM
3. 根据误差调整PWM
4. 重复步骤1-3
```

**PID就是第3步的具体方法**：根据误差计算PWM调整量。

---

## PID公式

### 标准公式（连续形式）

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
$$

### 离散形式（代码实现）

$$
u = K_p \cdot e + K_i \cdot \sum e \cdot \Delta t + K_d \cdot \frac{e - e_{last}}{\Delta t}
$$

### 参数说明

| 符号 | 含义 | 说明 |
|:----:|:----:|:-----|
| $u$ | 输出 | PWM调整量 |
| $e$ | 误差 | target - current |
| $K_p$ | 比例系数 | P项权重 |
| $K_i$ | 积分系数 | I项权重 |
| $K_d$ | 微分系数 | D项权重 |
| $\Delta t$ | 采样周期 | 两次计算的时间间隔 |

---

## P项：比例控制

### 公式

$$
P_{out} = K_p \cdot e
$$

### 物理意义

**误差越大，调整力度越大**。

### 示例

```
目标转速：100 RPM
实际转速：85 RPM
误差：e = 100 - 85 = 15 RPM

如果 Kp = 0.5：
  P_out = 0.5 × 15 = 7.5

意思是：你慢了15 RPM，我给你加7.5的PWM
```

### 特点

| 优点 | 缺点 |
|------|------|
| 响应快 | 有静差（达不到目标） |
| 简单 | Kp太大会振荡 |

### 静差问题

```
为什么会有静差？

假设目标100 RPM，Kp = 0.5

如果实际达到99 RPM：
  误差 = 1
  P_out = 0.5 × 1 = 0.5

这个0.5的PWM可能刚好抵消摩擦力
系统稳定在99 RPM，达不到100 RPM

这就是静差
```

---

## I项：积分控制

### 公式

$$
I_{out} = K_i \cdot \sum e \cdot \Delta t
$$

### 物理意义

**累积历史误差，消除静差**。

### 示例

```
假设误差一直是15 RPM，采样周期10ms

第1次：integral = 15 × 0.01 = 0.15
第2次：integral = 0.15 + 0.15 = 0.30
第3次：integral = 0.30 + 0.15 = 0.45
...

如果 Ki = 10：
  I_out = 10 × 0.45 = 4.5

意思是：你一直慢，我持续加力
```

### 特点

| 优点 | 缺点 |
|------|------|
| 消除静差 | 响应慢 |
| 稳态精度高 | 可能引起振荡 |
| | 有积分饱和问题 |

### 积分饱和

```
问题：如果误差长时间存在，积分项会变得很大

例如：电机堵转
  误差一直很大
  积分项持续增加
  PWM输出饱和

当堵转解除后：
  积分项还很大
  PWM还是饱和
  电机飞车

解决方法：积分限幅
```

---

## D项：微分控制

### 公式

$$
D_{out} = K_d \cdot \frac{e - e_{last}}{\Delta t}
$$

### 物理意义

**预测趋势，抑制超调**。

### 示例

```
上次误差：10 RPM
这次误差：15 RPM
误差变化：15 - 10 = 5 RPM
采样周期：10ms = 0.01s

微分 = 5 / 0.01 = 500

如果 Kd = 0.1：
  D_out = 0.1 × 500 = 50

意思是：你加速太快，我给你减力刹车
```

### 特点

| 优点 | 缺点 |
|------|------|
| 抑制超调 | 对噪声敏感 |
| 改善动态性能 | 可能引起抖动 |
| 预测趋势 | 速度环通常不用 |

### 噪声问题

```
问题：微分项对噪声非常敏感

例如：编码器读数有±1的抖动
  误差变化：±1
  微分 = ±1 / 0.01 = ±100

这个噪声被放大了100倍！

解决方法：
  1. 不用D项（速度环常用）
  2. 对微分项滤波
```

---

## 三项协同

### 各项作用

| 项 | 作用 | 类比 |
|----|------|------|
| P | 根据当前误差调整 | 看着差距踩油门 |
| I | 根据累积误差调整 | 差距一直存在就持续加力 |
| D | 根据误差变化调整 | 看到要超了就提前刹车 |

### 组合效果

```
P项：快速响应，但有静差
I项：消除静差，但响应慢
D项：抑制超调，但怕噪声

组合：
  PI控制：快速响应 + 无静差（最常用）
  PID控制：快速响应 + 无静差 + 抑制超调
```

### 典型应用

| 控制类型 | 常用组合 | 原因 |
|---------|---------|------|
| 速度控制 | PI | D项对速度噪声敏感 |
| 位置控制 | PID | 位置信号相对平滑 |
| 温度控制 | PI | 温度变化慢，不需要D |

---

## 代码实现

### 基本PID结构

```c
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    float target;       // 目标值
    float actual;       // 实际值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分累积
    
    float output;       // 输出
    
    float integral_limit;  // 积分限幅
    float output_limit;    // 输出限幅
} PID_t;
```

### PID计算函数

```c
float pid_calculate(PID_t *pid, float target, float actual) {
    // 更新目标值和实际值
    pid->target = target;
    pid->actual = actual;
    
    // 计算误差
    pid->error = target - actual;
    
    // 积分项
    pid->integral += pid->error;
    
    // 积分限幅
    if (pid->integral_limit != 0) {
        if (pid->integral > pid->integral_limit) {
            pid->integral = pid->integral_limit;
        } else if (pid->integral < -pid->integral_limit) {
            pid->integral = -pid->integral_limit;
        }
    }
    
    // PID计算
    float P_out = pid->Kp * pid->error;
    float I_out = pid->Ki * pid->integral;
    float D_out = pid->Kd * (pid->error - pid->last_error);
    
    pid->output = P_out + I_out + D_out;
    
    // 输出限幅
    if (pid->output_limit != 0) {
        if (pid->output > pid->output_limit) {
            pid->output = pid->output_limit;
        } else if (pid->output < -pid->output_limit) {
            pid->output = -pid->output_limit;
        }
    }
    
    // 保存误差
    pid->last_error = pid->error;
    
    return pid->output;
}
```

---

## 本节要点

1. **P项**：根据当前误差调整，响应快但有静差
2. **I项**：累积历史误差，消除静差
3. **D项**：预测误差变化，抑制超调
4. **速度控制**通常用PI，不用D
5. **积分限幅**防止积分饱和

---

下一节：[参数整定方法](parameters.md) - 学习如何调整PID参数

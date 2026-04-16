# 速度计算公式

> 从编码器脉冲数到电机转速的换算

---

## 核心问题

编码器输出的是脉冲计数，但我们需要的是电机转速（RPM）。

**如何从脉冲数计算出转速？**

---

## 公式推导

### 基本思路

```
转速 = 转过的角度 / 时间
     = (脉冲数 / 每转脉冲数) × 360° / 时间
     = 脉冲数 / (每转脉冲数 × 时间)
```

### RPM 计算公式

```
RPM = (Δcount × 60) / (PPR × Δt)
```

### 参数说明

| 符号 | 含义 | 单位 | 说明 |
|:----:|:-----|:-----|:-----|
| RPM | 转速 | 转/分钟 | Revolutions Per Minute |
| Δcount | 脉冲变化量 | 个 | 测量周期内的脉冲计数变化 |
| PPR | 每转脉冲数 | 个/转 | Pulses Per Revolution |
| Δt | 测量周期 | 秒 | 两次读数的时间间隔 |
| 60 | 时间换算系数 | 秒/分钟 | 60秒 = 1分钟 |

---

## 计算示例

### 示例1：基本计算

```
已知：
  PPR = 1000（四倍频后）
  Δt = 0.01秒（10ms测量周期）
  Δcount = 10（10ms内增加了10个脉冲）

求：RPM = ?

解：
  RPM = Δcount × 60 / (PPR × Δt)
      = 10 × 60 / (1000 × 0.01)
      = 600 / 10
      = 60 RPM

物理意义：如果电机一直以这个速度转，1分钟能转60圈
```

### 示例2：系数简化

在实际代码中，我们通常预先计算好系数：

```
系数 = 60 / (PPR × Δt)

如果 PPR = 1000, Δt = 0.01秒：
  系数 = 60 / (1000 × 0.01) = 60 / 10 = 6

那么：
  RPM = Δcount × 6
```

这就是为什么很多代码里直接写 `RPM = count * 6`。

### 示例3：不同PPR的系数

| PPR | Δt | 系数 | 代码 |
|-----|-----|------|------|
| 500 | 10ms | 12 | `RPM = count * 12` |
| 1000 | 10ms | 6 | `RPM = count * 6` |
| 2000 | 10ms | 3 | `RPM = count * 3` |
| 1000 | 5ms | 12 | `RPM = count * 12` |

---

## 代码实现

### 基本实现

```c
// 编码器参数
#define PPR         1000    // 每转脉冲数（四倍频后）
#define SAMPLE_TIME 0.01f   // 采样周期（秒）

// 速度计算函数
float calculate_rpm(int16_t delta_count) {
    float rpm = (float)delta_count * 60.0f / (PPR * SAMPLE_TIME);
    return rpm;
}
```

### 优化实现（预计算系数）

```c
// 预计算系数
#define PPR         1000
#define SAMPLE_TIME 0.01f
#define RPM_FACTOR  (60.0f / (PPR * SAMPLE_TIME))  // = 6.0f

// 速度计算函数
float calculate_rpm(int16_t delta_count) {
    return (float)delta_count * RPM_FACTOR;
}
```

### 完整示例（STM32）

```c
// 全局变量
volatile int16_t encoder_count = 0;    // 当前计数值
volatile int16_t last_count = 0;       // 上次计数值
volatile float current_rpm = 0;        // 当前转速

// 10ms定时器中断
void TIM_IRQHandler(void) {
    // 读取当前计数值
    encoder_count = TIM2->CNT;
    
    // 计算脉冲变化量
    int16_t delta = encoder_count - last_count;
    
    // 计算转速
    current_rpm = (float)delta * 6.0f;  // PPR=1000, Δt=10ms
    
    // 保存当前值
    last_count = encoder_count;
}
```

---

## 线速度换算

有时我们需要的是轮子的线速度（m/s），而不是转速（RPM）。

### 线速度公式

```
V = (RPM × 2πr) / 60
```

| 符号 | 含义 | 单位 |
|:----:|:-----|:-----|
| V | 线速度 | m/s |
| r | 轮子半径 | m |
| RPM | 转速 | 转/分钟 |

### 代码

```c
#define WHEEL_RADIUS 0.05f  // 轮子半径 5cm

float rpm_to_velocity(float rpm) {
    return rpm * 2.0f * 3.14159f * WHEEL_RADIUS / 60.0f;
}

float velocity_to_rpm(float velocity) {
    return velocity * 60.0f / (2.0f * 3.14159f * WHEEL_RADIUS);
}
```

---

## 处理计数溢出

### 问题

编码器计数器是16位的，范围是 0~65535。

当轮子持续正转，计数器会从65535跳到0（溢出）。
当轮子持续反转，计数器会从0跳到65535（下溢）。

### 解决方法

**方法一：使用有符号计数模式**

STM32定时器可以配置为有符号计数模式，计数范围变为 -32768~+32767。

```c
// 配置定时器为有符号计数
TIM2->CR1 |= TIM_CR1_DIR;  // 向下计数
```

**方法二：软件处理溢出**

```c
int16_t delta = encoder_count - last_count;

// 处理溢出
if (delta > 32767) {
    delta -= 65536;  // 实际是反转
} else if (delta < -32768) {
    delta += 65536;  // 实际是正转
}
```

**方法三：读取差值寄存器**

STM32的编码器模式下，可以配置自动重装载值，让计数器在达到最大值时自动清零。

---

## 测量周期选择

### 周期与精度

测量周期 Δt 影响速度测量的精度和响应速度：

| Δt | 优点 | 缺点 |
|----|------|------|
| 小（1~5ms） | 响应快 | 低速时精度差（脉冲数少） |
| 中（10~20ms） | 平衡 | - |
| 大（50~100ms） | 低速精度高 | 响应慢 |

### 选择建议

```
低速应用（< 100 RPM）：Δt = 20~50ms
中速应用（100~1000 RPM）：Δt = 10~20ms
高速应用（> 1000 RPM）：Δt = 5~10ms
```

对于麦氏轮机器人，通常选择 **10ms** 作为测量周期。

---

## 本节要点

1. **基本公式**：`RPM = (Δcount × 60) / (PPR × Δt)`
2. **系数简化**：预先计算 `60/(PPR×Δt)` 可以简化代码
3. **线速度换算**：`V = (RPM × 2πr) / 60`
4. **溢出处理**：注意16位计数器的溢出问题
5. **周期选择**：10ms是常用的平衡选择

---

下一节：[STM32配置方法](stm32-config.md) - 学习如何在STM32上配置编码器
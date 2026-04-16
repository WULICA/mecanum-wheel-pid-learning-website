# STM32配置方法

> 使用定时器编码器模式读取编码器

---

## 为什么用定时器编码器模式？

### 方法对比

| 方法 | 原理 | 优点 | 缺点 |
|------|------|------|------|
| 外部中断 | A相触发中断，中断里读B相 | 简单 | 高速时丢脉冲，占用CPU |
| 定时器编码器模式 | 硬件自动计数 | 不占CPU，精度高 | 配置稍复杂 |

**推荐使用定时器编码器模式**。

---

## 硬件连接

### 接线图

```
编码器              STM32
───────            ──────
  A相  ─────────→  TIMx_CH1 (如PA0)
  B相  ─────────→  TIMx_CH2 (如PA1)
  VCC  ─────────→  3.3V 或 5V
  GND  ─────────→  GND
```

### 引脚选择

不同STM32型号的定时器引脚不同，以下是STM32F103的示例：

| 定时器 | 通道 | 引脚 | 备注 |
|--------|------|------|------|
| TIM2 | CH1 | PA0 | 常用 |
| TIM2 | CH2 | PA1 | 常用 |
| TIM3 | CH1 | PA6 | 备用 |
| TIM3 | CH2 | PA7 | 备用 |
| TIM4 | CH1 | PB6 | 备用 |
| TIM4 | CH2 | PB7 | 备用 |

---

## 配置步骤

### 步骤1：使能时钟

```c
// 使能GPIO和定时器时钟
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // 使能GPIOA时钟
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // 使能TIM2时钟
```

### 步骤2：配置GPIO

```c
// 配置PA0、PA1为复用输入
GPIOA->CRL &= ~(0xFF << 0);          // 清除PA0、PA1配置
GPIOA->CRL |= (0x88 << 0);           // 复用推挽输出，实际上编码器模式会自动处理
```

### 步骤3：配置定时器

```c
// 基本配置
TIM2->PSC = 0;                       // 不分频
TIM2->ARR = 0xFFFF;                  // 自动重装载值（16位最大值）

// 编码器模式配置
TIM2->SMCR &= ~TIM_SMCR_SMS;         // 清除从模式选择
TIM2->SMCR |= 0x03;                  // 编码器模式3（四倍频）

// 输入捕获配置
TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
TIM2->CCMR1 |= (0x01 << 0);          // IC1映射到TI1
TIM2->CCMR1 |= (0x01 << 8);          // IC2映射到TI2

// 极性配置
TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);  // 不反相

// 使能计数器
TIM2->CR1 |= TIM_CR1_CEN;
```

---

## 完整配置代码

### 使用HAL库

```c
// 编码器初始化
void encoder_init(void) {
    TIM_Encoder_InitTypeDef encoder_config = {0};
    TIM_HandleTypeDef htim = {0};
    
    // 定时器基本配置
    htim.Instance = TIM2;
    htim.Init.Prescaler = 0;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = 0xFFFF;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    // 编码器模式配置
    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12;  // 四倍频
    encoder_config.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = 0;
    encoder_config.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC2Filter = 0;
    
    // 初始化
    if (HAL_TIM_Encoder_Init(&htim, &encoder_config) != HAL_OK) {
        Error_Handler();
    }
    
    // 启动编码器
    HAL_TIM_Encoder_Start(&htim, TIM_CHANNEL_ALL);
}
```

### 使用寄存器（推荐）

```c
// 编码器初始化（寄存器版本）
void encoder_init(void) {
    // 1. 使能时钟
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    // 2. 配置GPIO（输入模式）
    GPIOA->CRL &= ~(0xFF << 0);
    GPIOA->CRL |= (0x44 << 0);  // 浮空输入
    
    // 3. 配置定时器
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFF;
    
    // 4. 编码器模式
    TIM2->SMCR &= ~TIM_SMCR_SMS;
    TIM2->SMCR |= 0x03;  // 编码器模式3
    
    // 5. 输入捕获配置
    TIM2->CCMR1 = 0;
    TIM2->CCMR1 |= (0x01 << 0);  // IC1 -> TI1
    TIM2->CCMR1 |= (0x01 << 8);  // IC2 -> TI2
    
    // 6. 使能
    TIM2->CR1 |= TIM_CR1_CEN;
    
    // 7. 清零计数器
    TIM2->CNT = 0;
}
```

---

## 读取编码器值

### 读取当前计数

```c
// 读取编码器计数值
int16_t encoder_read(void) {
    return (int16_t)TIM2->CNT;
}
```

### 计算速度

```c
// 全局变量
int16_t last_count = 0;
float current_rpm = 0;

// 10ms定时器中断
void TIM3_IRQHandler(void) {
    // 读取当前计数
    int16_t current_count = (int16_t)TIM2->CNT;
    
    // 计算差值
    int16_t delta = current_count - last_count;
    
    // 计算转速（PPR=1000, Δt=10ms）
    current_rpm = (float)delta * 6.0f;
    
    // 保存
    last_count = current_count;
    
    // 清除中断标志
    TIM3->SR &= ~TIM_SR_UIF;
}
```

---

## 多编码器配置

麦氏轮机器人需要4个编码器，需要配置4个定时器。

### 定时器分配

| 轮子 | 定时器 | CH1引脚 | CH2引脚 |
|------|--------|---------|---------|
| 左前 | TIM2 | PA0 | PA1 |
| 右前 | TIM3 | PA6 | PA7 |
| 左后 | TIM4 | PB6 | PB7 |
| 右后 | TIM5 | PA0 | PA1（重映射） |

### 代码框架

```c
// 编码器结构体
typedef struct {
    TIM_TypeDef *TIMx;
    int16_t last_count;
    float rpm;
} Encoder_t;

// 四个编码器
Encoder_t encoders[4] = {
    {TIM2, 0, 0},  // 左前
    {TIM3, 0, 0},  // 右前
    {TIM4, 0, 0},  // 左后
    {TIM5, 0, 0},  // 右后
};

// 初始化所有编码器
void encoders_init(void) {
    for (int i = 0; i < 4; i++) {
        // 配置每个定时器...
    }
}

// 读取所有编码器
void encoders_update(void) {
    for (int i = 0; i < 4; i++) {
        int16_t count = (int16_t)encoders[i].TIMx->CNT;
        int16_t delta = count - encoders[i].last_count;
        encoders[i].rpm = (float)delta * 6.0f;
        encoders[i].last_count = count;
    }
}
```

---

## 常见问题

### 问题1：计数方向反了

**原因**：A、B相接反了

**解决**：
```c
// 方法1：交换接线
// 方法2：软件取反
int16_t delta = last_count - current_count;  // 注意顺序反了
```

### 问题2：低速时不稳定

**原因**：测量周期太短，脉冲数太少

**解决**：
- 增加测量周期（如从10ms改为20ms）
- 使用滑动平均滤波

```c
// 滑动平均滤波
#define FILTER_SIZE 5
float rpm_buffer[FILTER_SIZE] = {0};
int buffer_index = 0;

float filter_rpm(float new_rpm) {
    rpm_buffer[buffer_index] = new_rpm;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    
    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += rpm_buffer[i];
    }
    return sum / FILTER_SIZE;
}
```

### 问题3：高速时丢脉冲

**原因**：编码器输出频率超过定时器输入频率限制

**解决**：
- 检查编码器最高转速规格
- 降低编码器PPR
- 使用更高主频的MCU

---

## 调试技巧

### 串口打印

```c
// 在主循环中打印编码器值
while (1) {
    printf("CNT: %d, RPM: %.2f\n", TIM2->CNT, current_rpm);
    HAL_Delay(100);
}
```

### 验证方法

1. **手动转动电机**，观察计数器是否变化
2. **正转**，计数器应该增加
3. **反转**，计数器应该减少
4. **匀速转动**，RPM应该稳定

---

## 本节要点

1. **定时器编码器模式**是读取编码器的最佳方式
2. **四倍频模式**可提高4倍分辨率
3. **配置步骤**：使能时钟 → 配置GPIO → 配置定时器 → 启动
4. **多编码器**需要多个定时器
5. **调试时**先用串口打印验证

---

基础概念模块完成！下一模块：[PID控制原理](../02-pid/README.md)

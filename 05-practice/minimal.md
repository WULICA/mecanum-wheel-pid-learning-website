# 最小可行系统

> 开环控制方案

---

## 什么是最小可行系统？

**最小可行系统**：用最简单的方案让机器人动起来。

```
特点：
  - 只有内环，没有外环
  - 直接设定目标轮速
  - 系统能跑通，但精度有限

优点：
  - 代码简单，容易调试
  - 可以验证基础功能
  - 为后续开发打基础
```

---

## 系统架构

### 开环控制

```
用户输入：Vx, Vy, ω
    ↓
运动学解算 → target_rpm[4]
    ↓
内环PID → PWM输出
    ↓
电机转动
```

**没有外环反馈修正**，轮速会有误差，但系统能跑通。

---

## 代码实现

### 1. 编码器读取

```c
// 编码器结构体
typedef struct {
    TIM_TypeDef *TIMx;
    int16_t last_count;
    float rpm;
} Encoder_t;

Encoder_t encoders[4] = {
    {TIM2, 0, 0},
    {TIM3, 0, 0},
    {TIM4, 0, 0},
    {TIM5, 0, 0}
};

// 初始化编码器
void encoder_init(Encoder_t *enc) {
    // 使能时钟
    if (enc->TIMx == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // ... 其他定时器
    
    // 配置编码器模式
    enc->TIMx->SMCR |= 0x03;  // 编码器模式3
    enc->TIMx->CCMR1 |= 0x0101;
    enc->TIMx->CR1 |= TIM_CR1_CEN;
    enc->TIMx->CNT = 0;
}

// 读取编码器转速
float encoder_read_rpm(Encoder_t *enc) {
    int16_t count = (int16_t)enc->TIMx->CNT;
    int16_t delta = count - enc->last_count;
    enc->last_count = count;
    
    // PPR=1000, dt=10ms, 系数=6
    enc->rpm = (float)delta * 6.0f;
    
    return enc->rpm;
}
```

### 2. 内环PID

```c
// PID结构体
typedef struct {
    float Kp, Ki, Kd;
    float integral, last_error;
    float output_limit;
} PID_t;

PID_t inner_pid[4];

// PID计算
float pid_update(PID_t *pid, float error, float dt) {
    pid->integral += error * dt;
    
    float P = pid->Kp * error;
    float I = pid->Ki * pid->integral;
    float D = pid->Kd * (error - pid->last_error) / dt;
    
    float output = P + I + D;
    output = CLAMP(output, -pid->output_limit, pid->output_limit);
    
    pid->last_error = error;
    return output;
}
```

### 3. 运动学解算

```c
// 机器人参数
#define WHEEL_RADIUS 0.05f
#define L 0.3f

// 正向运动学：机器人速度 → 轮速
void forward_kinematics(float Vx, float Vy, float omega, float target_rpm[4]) {
    float v_fl = Vx - Vy - omega * L;
    float v_fr = Vx + Vy + omega * L;
    float v_rl = Vx + Vy - omega * L;
    float v_rr = Vx - Vy + omega * L;
    
    // 线速度 → RPM
    target_rpm[0] = v_fl * 60.0f / (2.0f * 3.14159f * WHEEL_RADIUS);
    target_rpm[1] = v_fr * 60.0f / (2.0f * 3.14159f * WHEEL_RADIUS);
    target_rpm[2] = v_rl * 60.0f / (2.0f * 3.14159f * WHEEL_RADIUS);
    target_rpm[3] = v_rr * 60.0f / (2.0f * 3.14159f * WHEEL_RADIUS);
}
```

### 4. 电机控制

```c
// 设置电机PWM
void set_motor_pwm(int motor_id, int pwm) {
    pwm = CLAMP(pwm, 0, 100);
    
    switch (motor_id) {
        case 0:  // 左前
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            break;
        case 1:  // 右前
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
            break;
        // ... 其他电机
    }
}
```

### 5. 主控制循环

```c
// 全局变量
float target_Vx = 0;
float target_Vy = 0;
float target_omega = 0;
float target_rpm[4] = {0};
float actual_rpm[4] = {0};

// 10ms定时器中断
void TIM_Control_IRQHandler(void) {
    // 1. 读取编码器
    for (int i = 0; i < 4; i++) {
        actual_rpm[i] = encoder_read_rpm(&encoders[i]);
    }
    
    // 2. 运动学解算
    forward_kinematics(target_Vx, target_Vy, target_omega, target_rpm);
    
    // 3. 内环PID
    for (int i = 0; i < 4; i++) {
        float error = target_rpm[i] - actual_rpm[i];
        float output = pid_update(&inner_pid[i], error, 0.01f);
        
        int pwm = 50 + (int)output;  // 基础PWM + PID输出
        set_motor_pwm(i, pwm);
    }
    
    // 清除中断标志
    TIM->SR &= ~TIM_SR_UIF;
}
```

### 6. 主函数

```c
int main(void) {
    // 硬件初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();  // PWM
    MX_TIM2_Init();  // 编码器
    // ...
    
    // 编码器初始化
    for (int i = 0; i < 4; i++) {
        encoder_init(&encoders[i]);
    }
    
    // PID初始化
    for (int i = 0; i < 4; i++) {
        inner_pid[i].Kp = 1.0f;
        inner_pid[i].Ki = 0.05f;
        inner_pid[i].Kd = 0.0f;
        inner_pid[i].output_limit = 50;
    }
    
    // 启动定时器中断
    HAL_TIM_Base_Start_IT(&htim6);
    
    // 主循环
    while (1) {
        // 可以在这里接收串口命令，设置目标速度
        // 例如：target_Vx = 0.5;
    }
}
```

---

## 测试步骤

### 步骤1：测试编码器

```c
// 在主循环打印编码器值
while (1) {
    printf("RPM: %.2f, %.2f, %.2f, %.2f\n",
           actual_rpm[0], actual_rpm[1], actual_rpm[2], actual_rpm[3]);
    HAL_Delay(100);
}
```

**手动转动电机**，观察编码器读数是否变化。

### 步骤2：测试单电机

```c
// 设置一个电机的目标转速
target_rpm[0] = 100;
target_rpm[1] = 0;
target_rpm[2] = 0;
target_rpm[3] = 0;
```

观察左前轮是否能稳定在100 RPM。

### 步骤3：测试前进

```c
// 设置前进速度
target_Vx = 0.2;  // 0.2 m/s
target_Vy = 0;
target_omega = 0;
```

观察机器人是否能前进。

### 步骤4：测试旋转

```c
// 设置旋转速度
target_Vx = 0;
target_Vy = 0;
target_omega = 0.5;  // 0.5 rad/s
```

观察机器人是否能原地旋转。

---

## 调参建议

### 内环PID调参

```
1. 单轮测试
   - 固定其他轮，单独调一个电机
   - 给定 target_rpm = 100
   - 调 Kp 到响应快但无超调
   - 调 Ki 消除静差

2. 四轮同步
   - 设置前进运动
   - 观察四轮转速是否一致
   - 单独调整每个轮子的参数

3. 旋转测试
   - 设置旋转运动
   - 观察旋转是否平稳
```

---

## 局限性

### 开环控制的问题

```
问题1：轮速有误差
  - 负载变化
  - 摩擦变化
  - 电压波动

问题2：机器人速度不准确
  - 轮子打滑
  - 实际速度可能偏离目标

问题3：轨迹不准确
  - 前进可能跑偏
  - 平移可能弯曲
```

### 解决方法

**添加外环控制**，形成双层闭环。

---

## 本节要点

1. **最小系统**：只有内环，没有外环
2. **代码结构**：编码器 → 运动学 → 内环PID → PWM
3. **测试步骤**：编码器 → 单电机 → 前进 → 旋转
4. **调参方法**：先单轮，再四轮同步
5. **局限性**：轮速有误差，需要外环修正

---

下一节：[完整代码实现](full-code.md) - 学习双层闭环的完整实现

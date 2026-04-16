# 完整代码实现

> 双层闭环控制完整代码

---

## 代码结构

```
├── main.c              // 主函数
├── config.h            // 配置参数
├── encoder.c/h         // 编码器驱动
├── motor.c/h           // 电机驱动
├── pid.c/h             // PID算法
├── kinematics.c/h      // 运动学解算
├── mpu6050.c/h         // 陀螺仪驱动（可选）
└── control.c/h         // 控制逻辑
```

---

## 配置参数

```c
// config.h

// 机器人参数
#define WHEEL_RADIUS      0.05f    // 轮子半径 (m)
#define LX                0.15f    // X方向轮距 (m)
#define LY                0.15f    // Y方向轮距 (m)
#define L                 (LX + LY)

// 编码器参数
#define PPR               1000     // 每转脉冲数
#define SAMPLE_TIME       0.01f    // 采样周期 (s)
#define RPM_FACTOR        (60.0f / (PPR * SAMPLE_TIME))

// 控制周期
#define INNER_LOOP_PERIOD 10       // 内环周期 (ms)
#define OUTER_LOOP_PERIOD 50       // 外环周期 (ms)

// 内环PID参数
#define INNER_KP          1.0f
#define INNER_KI          0.05f
#define INNER_KD          0.0f

// 外环PID参数
#define OUTER_KP          0.1f
#define OUTER_KI          0.01f
#define OUTER_KD          0.0f

// PWM参数
#define PWM_BASE          50       // 基础PWM
#define PWM_MAX           100      // 最大PWM
```

---

## PID模块

```c
// pid.h
typedef struct {
    float Kp, Ki, Kd;
    float integral, last_error;
    float integral_limit, output_limit;
} PID_t;

void pid_init(PID_t *pid, float Kp, float Ki, float Kd);
float pid_update(PID_t *pid, float target, float actual, float dt);

// pid.c
void pid_init(PID_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->last_error = 0;
    pid->integral_limit = 1000;
    pid->output_limit = 1000;
}

float pid_update(PID_t *pid, float target, float actual, float dt) {
    float error = target - actual;
    
    pid->integral += error * dt;
    pid->integral = CLAMP(pid->integral, -pid->integral_limit, pid->integral_limit);
    
    float P = pid->Kp * error;
    float I = pid->Ki * pid->integral;
    float D = pid->Kd * (error - pid->last_error) / dt;
    
    float output = P + I + D;
    output = CLAMP(output, -pid->output_limit, pid->output_limit);
    
    pid->last_error = error;
    return output;
}
```

---

## 运动学模块

```c
// kinematics.h
typedef struct {
    float Vx, Vy, omega;
} RobotVelocity_t;

typedef struct {
    float fl, fr, rl, rr;
} WheelVelocity_t;

WheelVelocity_t forward_kinematics(RobotVelocity_t robot_vel);
RobotVelocity_t inverse_kinematics(WheelVelocity_t wheel_vel);
float velocity_to_rpm(float velocity);
float rpm_to_velocity(float rpm);

// kinematics.c
WheelVelocity_t forward_kinematics(RobotVelocity_t robot_vel) {
    WheelVelocity_t wheel;
    float L_omega = L * robot_vel.omega;
    
    wheel.fl = robot_vel.Vx - robot_vel.Vy - L_omega;
    wheel.fr = robot_vel.Vx + robot_vel.Vy + L_omega;
    wheel.rl = robot_vel.Vx + robot_vel.Vy - L_omega;
    wheel.rr = robot_vel.Vx - robot_vel.Vy + L_omega;
    
    return wheel;
}

RobotVelocity_t inverse_kinematics(WheelVelocity_t wheel) {
    RobotVelocity_t robot;
    
    robot.Vx = (wheel.fl + wheel.fr + wheel.rl + wheel.rr) / 4.0f;
    robot.Vy = (-wheel.fl + wheel.fr + wheel.rl - wheel.rr) / 4.0f;
    robot.omega = (-wheel.fl + wheel.fr - wheel.rl + wheel.rr) / (4.0f * L);
    
    return robot;
}

float velocity_to_rpm(float velocity) {
    return velocity * 60.0f / (2.0f * 3.14159f * WHEEL_RADIUS);
}

float rpm_to_velocity(float rpm) {
    return rpm * 2.0f * 3.14159f * WHEEL_RADIUS / 60.0f;
}
```

---

## 控制模块

```c
// control.h
void control_init(void);
void inner_loop_execute(void);
void outer_loop_execute(void);

// control.c

// 全局变量
volatile float target_rpm[4] = {0};
volatile float actual_rpm[4] = {0};
volatile float target_Vx = 0, target_Vy = 0, target_omega = 0;

// 内环PID
PID_t inner_pid[4];

// 外环PID
PID_t outer_pid_Vx, outer_pid_Vy, outer_pid_omega;

// 初始化
void control_init(void) {
    // 内环PID初始化
    for (int i = 0; i < 4; i++) {
        pid_init(&inner_pid[i], INNER_KP, INNER_KI, INNER_KD);
        inner_pid[i].output_limit = 50;
    }
    
    // 外环PID初始化
    pid_init(&outer_pid_Vx, OUTER_KP, OUTER_KI, OUTER_KD);
    pid_init(&outer_pid_Vy, 0, 0, 0);  // Vy不控制
    pid_init(&outer_pid_omega, OUTER_KP, OUTER_KI, OUTER_KD);
}

// 内环执行（10ms）
void inner_loop_execute(void) {
    // 读取编码器
    for (int i = 0; i < 4; i++) {
        actual_rpm[i] = encoder_read_rpm(i);
    }
    
    // 内环PID
    for (int i = 0; i < 4; i++) {
        float output = pid_update(&inner_pid[i], target_rpm[i], actual_rpm[i], 0.01f);
        
        int pwm = PWM_BASE + (int)output;
        pwm = CLAMP(pwm, 0, PWM_MAX);
        motor_set_pwm(i, pwm);
    }
}

// 外环执行（50ms）
void outer_loop_execute(void) {
    // 1. 获取机器人当前速度
    WheelVelocity_t wheel_vel;
    wheel_vel.fl = rpm_to_velocity(actual_rpm[0]);
    wheel_vel.fr = rpm_to_velocity(actual_rpm[1]);
    wheel_vel.rl = rpm_to_velocity(actual_rpm[2]);
    wheel_vel.rr = rpm_to_velocity(actual_rpm[3]);
    
    RobotVelocity_t current_vel = inverse_kinematics(wheel_vel);
    
    // 用陀螺仪的ω
    current_vel.omega = mpu6050_get_gyro_z();
    
    // 2. 外环PID
    float dVx = pid_update(&outer_pid_Vx, target_Vx, current_vel.Vx, 0.05f);
    float dVy = 0;  // Vy不控制
    float d_omega = pid_update(&outer_pid_omega, target_omega, current_vel.omega, 0.05f);
    
    // 3. 修正后的速度
    RobotVelocity_t corrected_vel;
    corrected_vel.Vx = target_Vx + dVx;
    corrected_vel.Vy = target_Vy + dVy;
    corrected_vel.omega = target_omega + d_omega;
    
    // 4. 正运动学解算
    WheelVelocity_t target_wheel = forward_kinematics(corrected_vel);
    
    // 5. 更新内环目标
    target_rpm[0] = velocity_to_rpm(target_wheel.fl);
    target_rpm[1] = velocity_to_rpm(target_wheel.fr);
    target_rpm[2] = velocity_to_rpm(target_wheel.rl);
    target_rpm[3] = velocity_to_rpm(target_wheel.rr);
}
```

---

## 主函数

```c
// main.c
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "kinematics.h"
#include "mpu6050.h"
#include "control.h"

int main(void) {
    // HAL初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();  // PWM
    MX_TIM2_Init();  // 编码器
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();  // 内环定时器
    MX_TIM7_Init();  // 外环定时器
    MX_I2C1_Init();  // MPU6050
    
    // 外设初始化
    encoder_init_all();
    motor_init_all();
    mpu6050_init();
    control_init();
    
    // 启动定时器中断
    HAL_TIM_Base_Start_IT(&htim6);  // 内环 10ms
    HAL_TIM_Base_Start_IT(&htim7);  // 外环 50ms
    
    // 主循环
    while (1) {
        // 可以在这里：
        // 1. 接收串口命令
        // 2. 设置目标速度
        // 3. 打印调试信息
    }
}

// 内环定时器中断
void TIM6_DAC_IRQHandler(void) {
    inner_loop_execute();
    TIM6->SR &= ~TIM_SR_UIF;
}

// 外环定时器中断
void TIM7_IRQHandler(void) {
    outer_loop_execute();
    TIM7->SR &= ~TIM_SR_UIF;
}
```

---

## 串口命令

```c
// 接收串口命令设置目标速度
void process_uart_command(void) {
    if (uart_rx_ready()) {
        char cmd = uart_get_char();
        
        switch (cmd) {
            case 'w':  // 前进
                target_Vx = 0.3f;
                target_Vy = 0;
                target_omega = 0;
                break;
            case 's':  // 后退
                target_Vx = -0.3f;
                target_Vy = 0;
                target_omega = 0;
                break;
            case 'a':  // 左平移
                target_Vx = 0;
                target_Vy = -0.2f;
                target_omega = 0;
                break;
            case 'd':  // 右平移
                target_Vx = 0;
                target_Vy = 0.2f;
                target_omega = 0;
                break;
            case 'q':  // 左旋转
                target_Vx = 0;
                target_Vy = 0;
                target_omega = -0.5f;
                break;
            case 'e':  // 右旋转
                target_Vx = 0;
                target_Vy = 0;
                target_omega = 0.5f;
                break;
            case 'x':  // 停止
                target_Vx = 0;
                target_Vy = 0;
                target_omega = 0;
                break;
        }
    }
}
```

---

## 本节要点

1. **代码结构**：模块化设计，便于维护
2. **配置参数**：集中管理，便于调整
3. **内环**：10ms执行，控制轮速
4. **外环**：50ms执行，控制机器人速度
5. **串口命令**：方便测试和调试

---

下一节：[调试方法](debugging.md) - 学习调试工具和方法

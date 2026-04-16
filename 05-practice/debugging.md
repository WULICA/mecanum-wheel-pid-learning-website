# 调试方法

> 串口调试、波形观察

---

## 调试工具

| 工具 | 用途 | 推荐软件 |
|------|------|---------|
| 串口调试助手 | 发送命令、查看数据 | SSCOM、XCOM |
| 波形绘制 | 实时观察曲线 | VOFA+、SerialPlot |
| 逻辑分析仪 | 分析时序 | Saleae |
| 示波器 | 分析PWM波形 | - |

---

## 串口调试

### 基本配置

```c
// 重定向printf
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
```

### 打印调试信息

```c
// 打印轮速信息
void print_rpm_info(void) {
    printf("Target: %.1f, %.1f, %.1f, %.1f\n",
           target_rpm[0], target_rpm[1], target_rpm[2], target_rpm[3]);
    printf("Actual: %.1f, %.1f, %.1f, %.1f\n",
           actual_rpm[0], actual_rpm[1], actual_rpm[2], actual_rpm[3]);
}

// 打印机器人速度
void print_robot_velocity(void) {
    printf("Vx: %.3f m/s, Vy: %.3f m/s, omega: %.3f rad/s\n",
           current_Vx, current_Vy, current_omega);
}

// 打印PID信息
void print_pid_info(int motor_id) {
    printf("Motor %d: error=%.2f, P=%.2f, I=%.2f, output=%.2f\n",
           motor_id,
           inner_pid[motor_id].error,
           inner_pid[motor_id].Kp * inner_pid[motor_id].error,
           inner_pid[motor_id].Ki * inner_pid[motor_id].integral,
           inner_pid[motor_id].output);
}
```

### 串口命令

```c
// 串口接收处理
void uart_rx_handler(void) {
    static char buffer[32];
    static int index = 0;
    
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        char ch = huart1.Instance->DR;
        
        if (ch == '\n' || ch == '\r') {
            buffer[index] = '\0';
            process_command(buffer);
            index = 0;
        } else {
            buffer[index++] = ch;
            if (index >= 32) index = 0;
        }
    }
}

// 命令处理
void process_command(char *cmd) {
    if (strcmp(cmd, "start") == 0) {
        target_Vx = 0.3f;
    } else if (strcmp(cmd, "stop") == 0) {
        target_Vx = 0;
        target_Vy = 0;
        target_omega = 0;
    } else if (strncmp(cmd, "vx ", 3) == 0) {
        target_Vx = atof(cmd + 3);
    } else if (strncmp(cmd, "kp ", 3) == 0) {
        inner_pid[0].Kp = atof(cmd + 3);
    }
    // ... 更多命令
}
```

---

## 波形观察

### VOFA+配置

VOFA+ 是一款强大的波形绘制工具，支持多种协议。

**JustFloat协议**：直接发送float数据

```c
// 发送波形数据
void send_wave_data(void) {
    float data[8];
    
    data[0] = target_rpm[0];  // 目标轮速
    data[1] = actual_rpm[0];  // 实际轮速
    data[2] = target_Vx;      // 目标Vx
    data[3] = current_Vx;     // 实际Vx
    data[4] = target_omega;   // 目标ω
    data[5] = current_omega;  // 实际ω
    data[6] = inner_pid[0].output;  // PID输出
    data[7] = 0;  // 填充
    
    // 发送
    HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(data), 100);
    
    // 发送尾帧
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart1, tail, 4, 100);
}
```

### SerialPlot配置

SerialPlot支持简单的CSV格式：

```c
// 发送CSV格式数据
void send_csv_data(void) {
    printf("%.2f,%.2f,%.2f,%.2f\n",
           target_rpm[0], actual_rpm[0],
           target_Vx, current_Vx);
}
```

---

## 调试流程

### 步骤1：验证编码器

```
测试方法：
1. 手动转动电机
2. 观察编码器读数

预期结果：
- 正转时读数增加
- 反转时读数减少
- 读数稳定，无跳变
```

### 步骤2：验证单电机PID

```
测试方法：
1. 设置 target_rpm[0] = 100
2. 观察 actual_rpm[0] 的响应

预期结果：
- 快速接近目标
- 无大幅超调
- 稳定在目标附近

调参：
- 响应慢 → 增大Kp
- 有静差 → 增大Ki
- 振荡 → 减小Kp和Ki
```

### 步骤3：验证四轮同步

```
测试方法：
1. 设置 target_Vx = 0.2
2. 观察四个轮子的转速

预期结果：
- 四轮转速接近
- 机器人直线前进

调参：
- 如果某轮转速偏差大，单独调整该轮的PID参数
```

### 步骤4：验证外环

```
测试方法：
1. 设置 target_Vx = 0.3
2. 观察 current_Vx 的响应

预期结果：
- current_Vx 接近 target_Vx
- 无大幅超调
- 稳定在目标附近

调参：
- 响应慢 → 增大外环Kp
- 有静差 → 增大外环Ki
- 振荡 → 减小外环Kp和Ki
```

---

## 常用调试技巧

### 技巧1：分段调试

```
不要一次性测试所有功能，分段测试：

1. 先测试编码器读取
2. 再测试单电机PID
3. 再测试四轮同步
4. 最后测试外环
```

### 技巧2：参数记录

```c
// 记录调好的参数
void print_params(void) {
    printf("=== PID Parameters ===\n");
    printf("Inner: Kp=%.2f, Ki=%.3f, Kd=%.3f\n",
           INNER_KP, INNER_KI, INNER_KD);
    printf("Outer: Kp=%.2f, Ki=%.3f, Kd=%.3f\n",
           OUTER_KP, OUTER_KI, OUTER_KD);
}
```

### 技巧3：数据记录

```c
// 记录关键数据，用于离线分析
#define LOG_SIZE 1000
typedef struct {
    float target;
    float actual;
    float output;
    uint32_t time;
} LogEntry_t;

LogEntry_t log_buffer[LOG_SIZE];
int log_index = 0;

void log_data(float target, float actual, float output) {
    if (log_index < LOG_SIZE) {
        log_buffer[log_index].target = target;
        log_buffer[log_index].actual = actual;
        log_buffer[log_index].output = output;
        log_buffer[log_index].time = HAL_GetTick();
        log_index++;
    }
}

void print_log(void) {
    for (int i = 0; i < log_index; i++) {
        printf("%lu,%.2f,%.2f,%.2f\n",
               log_buffer[i].time,
               log_buffer[i].target,
               log_buffer[i].actual,
               log_buffer[i].output);
    }
}
```

### 技巧4：安全保护

```c
// 添加安全保护，防止调试时出问题
void safety_check(void) {
    // 检查轮速是否超限
    for (int i = 0; i < 4; i++) {
        if (fabs(actual_rpm[i]) > 500) {
            // 轮速过快，紧急停止
            emergency_stop();
        }
    }
    
    // 检查电流是否超限
    if (get_current() > CURRENT_LIMIT) {
        emergency_stop();
    }
}

void emergency_stop(void) {
    // 停止所有电机
    for (int i = 0; i < 4; i++) {
        motor_set_pwm(i, 0);
    }
    
    // 清除目标
    target_Vx = 0;
    target_Vy = 0;
    target_omega = 0;
}
```

---

## 本节要点

1. **串口调试**：打印信息、发送命令
2. **波形观察**：VOFA+、SerialPlot
3. **调试流程**：编码器 → 单电机 → 四轮 → 外环
4. **调试技巧**：分段调试、参数记录、数据记录、安全保护

---

下一节：[常见问题FAQ](faq.md) - 学习问题诊断与解决

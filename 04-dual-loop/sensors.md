# 传感器选择

> Vx、Vy、ω 从哪来？

---

## 问题

外环控制需要知道机器人的实际速度：
- Vx：前进速度
- Vy：平移速度
- ω：旋转角速度

**这些速度从哪里获取？**

---

## 速度来源

### 三种速度的不同来源

| 速度 | 推荐来源 | 原因 |
|------|---------|------|
| Vx | 四轮速度平均 | 相对可靠，误差部分抵消 |
| Vy | 通常不控制 | 噪声大，难以准确测量 |
| ω | 陀螺仪 | 精度高，不受打滑影响 |

---

## Vx：前进速度

### 方法：四轮速度平均

$$
V_x = \frac{V_{fl} + V_{fr} + V_{rl} + V_{rr}}{4}
$$

### 为什么用平均？

```
优点：
  - 四个轮子都有贡献
  - 误差部分抵消
  - 相对可靠

缺点：
  - 如果所有轮子都打滑，会误判
  - 但这种情况较少
```

### 代码实现

```c
float get_Vx_from_wheels(float actual_rpm[4]) {
    // RPM → 线速度
    float v_fl = rpm_to_velocity(actual_rpm[0]);
    float v_fr = rpm_to_velocity(actual_rpm[1]);
    float v_rl = rpm_to_velocity(actual_rpm[2]);
    float v_rr = rpm_to_velocity(actual_rpm[3]);
    
    // 平均
    float Vx = (v_fl + v_fr + v_rl + v_rr) / 4.0f;
    
    return Vx;
}
```

---

## Vy：平移速度

### 为什么不控制？

```
问题：
  - 从轮速推算的Vy噪声大
  - 地面摩擦不均影响大
  - 轮子安装角度误差影响大
  - 难以准确测量

解决：
  - 通常设 target_Vy = 0
  - 不进行闭环控制
```

### 如果需要控制

可以使用光流传感器或外部定位系统，但成本较高。

```c
float get_Vy(void) {
    // 通常返回0，不控制
    return 0;
}
```

---

## ω：旋转角速度

### 方法1：从轮速推算（不推荐）

$$
\omega = \frac{-V_{fl} + V_{fr} - V_{rl} + V_{rr}}{4L}
$$

**问题**：
- 轮子打滑时误差大
- 误差会被放大（除以L）

### 方法2：用陀螺仪（推荐）

**优点**：
- 直接测量，精度高
- 不受轮子打滑影响
- 响应快

**常用陀螺仪**：MPU6050

### MPU6050配置

```c
// MPU6050初始化
void mpu6050_init(void) {
    // I2C初始化
    // 配置陀螺仪量程：±500°/s
    // 配置采样率：100Hz
}

// 读取角速度
float mpu6050_get_gyro_z(void) {
    int16_t gyro_z;
    // 读取Z轴陀螺仪数据
    gyro_z = i2c_read(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H);
    
    // 转换为rad/s
    // 量程±500°/s，灵敏度65.5 LSB/(°/s)
    float omega = (float)gyro_z / 65.5f * 3.14159f / 180.0f;
    
    return omega;
}
```

### 陀螺仪校准

```c
// 零偏校准
float gyro_offset = 0;

void gyro_calibrate(void) {
    // 静止状态，读取100次取平均
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += mpu6050_get_gyro_z_raw();
        delay(10);
    }
    gyro_offset = sum / 100.0f;
}

float get_omega_gyro(void) {
    float omega = mpu6050_get_gyro_z_raw() - gyro_offset;
    return omega;
}
```

---

## 传感器融合

### 融合策略

```c
RobotVelocity_t get_robot_velocity_fused(float actual_rpm[4]) {
    RobotVelocity_t vel;
    
    // Vx：用轮速平均
    vel.Vx = get_Vx_from_wheels(actual_rpm);
    
    // Vy：不控制
    vel.Vy = 0;
    
    // ω：用陀螺仪
    vel.omega = get_omega_gyro();
    
    return vel;
}
```

### 为什么这样融合？

```
Vx用轮速：
  - 前进运动四个轮子都参与
  - 误差部分抵消
  - 相对可靠

ω用陀螺仪：
  - 直接测量，精度高
  - 不受打滑影响
  - 旋转控制更准确

Vy不控制：
  - 噪声大
  - 通常不需要精确的平移控制
```

---

## 其他传感器

### 编码器

**用途**：测量轮子转速

**已在前面的章节讲解**

### 陀螺仪（MPU6050）

**用途**：测量旋转角速度

**特点**：
- 3轴陀螺仪 + 3轴加速度计
- I2C接口
- 成本低（约10元）

### 磁力计

**用途**：测量航向角

**特点**：
- 可以获取绝对方向
- 受磁场干扰影响

### 光流传感器

**用途**：测量平移速度

**特点**：
- 可以准确测量Vy
- 成本较高

### 外部定位

**用途**：获取绝对位置

**特点**：
- 如UWB、视觉定位
- 精度高，成本高

---

## 传感器数据处理

### 滤波

传感器数据通常需要滤波：

```c
// 滑动平均滤波
#define FILTER_SIZE 5
float filter_buffer[FILTER_SIZE];
int filter_index = 0;

float filter(float value) {
    filter_buffer[filter_index] = value;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    
    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += filter_buffer[i];
    }
    return sum / FILTER_SIZE;
}
```

### 互补滤波

融合陀螺仪和加速度计：

```c
// 互补滤波
float angle = 0;  // 融合后的角度
float alpha = 0.98f;  // 权重

void complementary_filter(float dt) {
    float gyro = get_omega_gyro();
    float accel = get_angle_accel();  // 从加速度计计算角度
    
    // 融合
    angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel;
}
```

---

## 本节要点

1. **Vx**：用四轮速度平均，相对可靠
2. **Vy**：通常不控制，噪声大
3. **ω**：用陀螺仪，精度高
4. **陀螺仪**：需要零偏校准
5. **滤波**：传感器数据通常需要滤波

---

双层闭环控制模块完成！下一模块：[实践案例](../05-practice/README.md)

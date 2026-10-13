# MPU6050 姿态解算封装使用说明

基于 STM32 HAL 库封装的 MPU6050 六轴传感器驱动 + Mahony 互补滤波姿态解算。
底层 I2C1（SCL=PB6，SDA=PB7），解算输出四元数、欧拉角（roll/pitch/yaw）、角速度、角加速度等。

```
mpu6050/
├── mpu6050.c / mpu6050.h   # 底层驱动：初始化、读原始数据、零偏校准
└── ins.c / ins.h           # 姿态解算：四元数、欧拉角、角加速度
main.c                      # 使用示例
```

## 一、文件添加

将 `mpu6050` 文件夹加入工程，并确保编译时包含该头文件路径，然后在 `main.c` 中引入总入口头文件（将源文件加入cmakelists中才能有效包含，参考第五条）：

```c
#include "mpu6050/ins.h"   // main.c:33
```

## 二、main.c 中的关键代码

### 1. 初始化（`MX_xxx_Init()` 之后，进入 while 之前）

```c
// 陀螺仪初始化
ins_init();                 // main.c:104
```

`ins_init()` 内部会自动完成：

1. `bsp_mpu6050_init()`：复位芯片、唤醒、配置量程（±2g / ±250°/s）与 100Hz 采样率；
2. `bsp_mpu6050_calibrate(500)`：上电静止约 500ms 完成陀螺仪零偏校准。

> 注意：调用 `ins_init()` 期间必须保持传感器**静止**，否则零偏不准。

### 2. 获取数据指针（只需一次）

```c
// 获取陀螺仪数据的指针
ins_data_t *ins = ins_data();   // main.c:114
```

`ins_data()` 返回内部静态数据的指针，后续直接在循环里读取即可，避免重复拷贝。

### 3. 周期调用更新（在 while(1) 里）

```c
// 更新 MPU6050 数据（陀螺仪 + 加速度计 + 姿态解算）
ins_update();                   // main.c:118
```

`ins_update()` 完成一次：读原始数据 → 计算 dt → 角速度/角加速度 → Mahony 滤波姿态解算 → 四元数转欧拉角。
建议固定周期调用（示例中 `HAL_Delay(10)`，即约 100Hz），周期越稳定解算越准。

### 4. 读取解算结果并通过串口发送（VOFA+ FireWater 格式）

```c
char buff[256];
sprintf(buff, "%.3f, %.3f, %.3f, %.2f, %.2f, %.2f, %.3f, %.3f, %.3f\n",
        ins->gyro[0], ins->gyro[1], ins->gyro[2],              // 角速度 °/s
        ins->gyro_accel[0], ins->gyro_accel[1], ins->gyro_accel[2], // 角加速度 °/s²
        ins->roll, ins->pitch, ins->yaw);                      // 姿态角 °
HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff));      // main.c:121-125

HAL_Delay(10);                  // main.c:127
```

## 三、`ins_data_t` 可用字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `q[4]` | float | 姿态四元数 `[w, x, y, z]` |
| `roll` / `pitch` / `yaw` | float | 欧拉角，单位 °，范围约 ±180 |
| `yaw_total_angle` | float | 累计偏航角，单位 °，可超过 ±180 |
| `gyro[3]` | float | 角速度，单位 °/s（已去零偏） |
| `gyro_accel[3]` | float | 角加速度，单位 °/s² |
| `timestamp` | uint32_t | 更新时间戳，单位 ms |
| `raw` | bsp_mpu6050_data_t | 本次底层原始数据 |

其中 `raw` 内含：

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `accel[3]` | g | 加速度 |
| `gyro[3]` | °/s | 角速度 |
| `temp` | ℃ | 温度 |


## 四、常见问题

- **yaw 会缓慢漂移**：MPU6050 无磁力计，yaw 由陀螺积分得到，属正常现象。需要绝对航向请加磁力计（如 HMC5883L）做九轴融合。
- **数据抖动大**：检查传感器是否固定牢靠、供电是否稳定。


## 五、CMakeLists.txt 配置

如果使用 CMake 工程，需要把 `.c` 源文件加入编译，并让 `printf` 支持浮点（`sprintf` 打印 `float` 才不会丢失小数）。每次修改CmakeLists都需要重新点击cmake

### 1. 添加源文件

找到 `target_sources(...)` 并加入 MPU6050 的源文件：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        mpu6050/mpu6050.c
        mpu6050/ins.c
)
```
> 说明：添加的源文件路径根据自己文件夹而定，此处只作为演示

### 2. 开启浮点 printf 支持

在CmakeLists.txt最下面添加下面代码
```cmake
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
)
```

> 说明：该选项链接了支持浮点的 `printf/sprintf` 实现，否则 `sprintf("%.3f", ...)` 打印 `float` 会输出异常。

# OLED屏幕封装代码说明
此处代码是祖传的，源文件中写了大量的注释，自行学习使用，或在B站上学习其他教程来用屏幕打印字符
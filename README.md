# STM32 电控考核

基于 STM32F103C8T6、TB6612 和 MPU6050 的电机控制工程，使用编码器测速，通过 VOFA+ 设置目标并查看数据。

## 功能

- PWM 正反转与调速。
- 编码器测速，PID 速度闭环。
- IMU 数据采集与姿态解算，输出角速度、角加速度和欧拉角。
- 角度 PID 外环配合速度内环，控制电机轴上雪糕棒的角度。

## 硬件接线

| STM32 引脚 | 连接 |
| --- | --- |
| PA0 / TIM2_CH1 | TB6612 PWMA |
| PB0 / PB1 | TB6612 AIN1 / AIN2 |
| PA6 / PA7 / TIM3 | 编码器 A / B |
| PB6 / PB7 / I2C1 | IMU SCL / SDA，AD0 接地 |
| PA9 / PA10 / USART1 | USB 转串口模块 RX / TX |
| PA13 / PA14 | ST-LINK SWDIO / SWCLK |

所有模块共地，TB6612 的 VM、VCC 和 STBY 需正确连接。IMU 固定在棒上，pitch 转轴与电机输出轴平行。

## 构建与烧录

开发环境为 Windows PowerShell，需要 CMake 3.22 及以上、Ninja、GNU Arm 工具链和 OpenOCD。

在工程根目录执行：

```powershell
.\tools\build.ps1 -Preset Debug
.\tools\flash.ps1 -Preset Debug
```

烧录使用 ST-LINK。构建产物为 `build/Debug/STM32_Control.elf`，将 `Debug` 换成 `Release` 可构建发布版本。工具路径不同，可用 `-ToolchainBin` 和 `-OpenOcdRoot` 分别指定编译器目录与 OpenOCD 根目录。

## 使用

上电后保持电机和 IMU 静止约 2 秒，等待 `READY`。程序不会自动启动电机。

VOFA+ 选择 **FireWater**，打开设备对应串口，设置为 **115200、8N1、无流控**。发送框追加 LF 或 CRLF；`q` 无需换行。

| 指令 | 作用 |
| --- | --- |
| `pwm:300` / `pwm:-300` | 正转 / 反转，约 30% 占空比，范围 −500～500 |
| `speed:3` / `speed:-3` | 设置目标转速，单位 RPM，范围 −30～30 |
| `z` | 将当前位置设为零点，立即启动 0° 保持；发送前把棒摆水平并保持静止 |
| `angle:5` / `angle:-5` | 设置相对零点的目标角度，范围 −60°～60°；需先发送 `z` |
| `q` / `0` | 停止电机 |
| `r` | 停机并重新初始化、校准 IMU；成功后重新发送 `z` |
| `v` / `p` | 显示 / 暂停连续数据，不影响控制 |

发送一条指令后，等收到提示再发送下一条。默认只显示提示文字，发送 `v` 才会连续输出数据。

测速时先发送 `speed:3`，再发送 `v`，将曲线绑定到 I2、I3。角度控制先摆平并发送 `z`，随后发送 `angle:5`，查看 I0、I1。

## VOFA+ 数据通道

| 通道 | 数据 | 单位 |
| --- | --- | --- |
| I0 / I1 | 目标角度 / 实际角度 | ° |
| I2 / I3 | 目标速度 / 实际速度 | RPM |
| I4 / I5 | 累计编码器计数 / PWM 输出 | 计数 / −500～500 |
| I6～I8 | X、Y、Z 角速度 | °/s |
| I9～I11 | X、Y、Z 角加速度 | °/s² |
| I12～I14 | roll / pitch / yaw | ° |
| I15 | 控制模式：0 停机，1 PWM，2 速度，3 角度 | — |
| I16 | IMU 数据有效标志：1 有效，0 无效 | — |

数据约每 50 ms 发送一次。六轴 IMU 的 yaw 会漂移。

## 代码结构

```text
Core/
├── Src/
│   ├── main.c       # 初始化、串口命令与控制主流程
│   ├── motor.c      # PWM 与方向控制
│   ├── pid.c        # PID 计算
│   ├── mpu6050.c    # IMU 初始化、原始数据读取与零偏校准
│   └── ins.c        # 四元数与姿态解算
└── Inc/             # 配套头文件与 config.h
tools/               # 构建、烧录脚本
tests/               # 主机模型测试与 CubeMX 生成区校验
STM32_Control.ioc    # CubeMX 外设配置
```

`main.c` 的 `USER CODE BEGIN 3` 按收指令 → 读数据 → 角度环 → 速度环 → 发数据的顺序执行。CubeMX 生成的初始化代码保留在原位置。

## 参数与故障处理

- `Core/Inc/config.h` 中的 `ENCODER_CPR` 是输出轴一圈的四倍频计数，当前值 **2112 尚未核实**，需确认后才能保证 RPM 准确。`ENCODER_SIGN` 和 `ANGLE_SIGN` 决定计数与角度方向。
- PID 参数在 `main.c` 的 `USER CODE BEGIN PV`。速度环初值为 `25 / 80 / 0`，角度环为 `1.5 / 0 / 0.05`，按 P、I、D 排列。先整定速度环，再整定角度环。
- 控制名义周期为 20 ms，PID 使用实际时间间隔。PWM 最大占空比约 50%，电机换向等待 100 ms。
- 角度模式短暂读失败会暂停输出并重置 I2C，200 ms 内恢复就继续控制，保留零点。持续失联则停机；检查供电和接线后发送 `r`，成功后重新摆平、发送 `z`。

## 测试

```powershell
.\tests\run.ps1
```

需要 Python 和主机 GCC，可通过 `-Python`、`-Gcc` 指定路径。测试覆盖 PWM、速度环、角度环、IMU 故障恢复及串口指令。

Debug、Release 构建和六组主机测试已通过，最新固件尚未烧录，实机角度控制仍待验证。详见[验证记录](docs/VALIDATION.md)和[考核验收说明](docs/ASSESSMENT.md)。

## 参考与许可

MPU6050 与 INS 模块移植自 [jiangjiaxu222/mpu6050-code](https://gitee.com/jiangjiaxu222/mpu6050-code)，保留原封装接口，适配说明见 [PORTING.md](docs/mpu6050/PORTING.md)。该仓库未明示许可证；HAL 和 CMSIS 使用各目录附带的许可证。

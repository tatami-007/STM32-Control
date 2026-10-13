# STM32 电控考核：精简版

保留四项基础要求：TB6612正反转调速、IMU姿态数据、PID速度环、角度外环+速度内环。
按使用者要求不做OLED和按键。上电不自动转动，控制由串口手动启动。

## 代码怎么读

**精简范围仅限我新增的代码。main.c中USER CODE区域之外的CubeMX代码完整保留，Gitee的封装文件也保留。**
tests/check_cube_main.py对照本次修改前备份校验生成区；以后主动用CubeMX变更外设配置重新生成时，需核对并更新校验基线。

1. **先看main.c的USER CODE BEGIN 2和while(1)**：初始化后，按1～5的注释执行收指令→读数据→算角度PID→算速度PID→发送数据。下面几百行MX初始化函数是CubeMX生成的，初学时先略过。
2. MPU6050按原README调用：`ins_init();`初始化，`ins_data_t *ins=ins_data();`取指针，循环`ins_update();`后读`ins->pitch`、`ins->gyro`等数据。
3. `motor.c`：PWM与方向；`pid.c`：通用PID。
4. `serial.c`：串口只收命令/发送字符串，命令由main.c处理。
5. `mpu6050/`：**Gitee原仓库的四个驱动/姿态文件直接移植并参与编译**，保留README总入口`ins.h`。修改记录见[PORTING.md](mpu6050/PORTING.md)。

基础版本只有三种运行方式：直接PWM、速度PID、角度PID。
角度模式仅在IMU读取失败时停止；q手动急停、串口错误停机、PWM限幅和换向等待仍保留。

## 构建与烧录

```powershell
./tools/build.ps1 -Preset Release
./tools/flash.ps1 -Preset Release
```

脚本可指定工具链/OpenOCD路径。完整工程包括CubeMX .ioc、启动文件、链接脚本和HAL/CMSIS。
上电保持IMU静止约2秒，等IMU READY。使用失败时显示IMU INIT FAILED，不会假装正常。

## 接线

| 引脚 | 连接 |
|---|---|
| PA0 / TIM2_CH1 | TB6612 PWMA |
| PB0 / PB1 | AIN1 / AIN2 |
| PA6 / PA7 / TIM3 | 编码器A / B |
| PB6 / PB7 / I2C1 | IMU SCL / SDA，AD0接地 |
| PA9 / PA10 / USART1 | USB串口RX / TX，交叉连接 |
| PA13 / PA14 | ST-LINK SWDIO / SWCLK |

沿用已验证的驱动供电/STBY连接，所有模块共地。
PWM为1kHz；编码器TI12四倍频、16位计数；IMU和控制为100Hz。

## VOFA操作

FireWater、COM5（以设备实际端口为准）、115200、8N1、无流控。

| 指令 | 作用 |
|---|---|
| `pwm:300` / `pwm:-300` | 正/反转30%PWM，范围±500 |
| `speed:3` / `speed:-3` | 速度PID目标RPM，范围±30 |
| `z` | 把当前棒位置设零，立即启动0度保持；必须先摆水平并静止 |
| `angle:5` / `angle:-5` | 相对零点目标角度，范围±60° |
| `q` / `0` | 停机 |
| `p` / `v` | 暂停/恢复数值，暂停时每秒显示控制状态 |

**除急停q外，所有指令都追加LF或CRLF**。发送框例如填speed:3，并选择追加换行。
发p后暂停数值，v恢复；不再重复打印定时状态消息。

| 通道 | 含义 |
|---|---|
| I0 / I1 | 目标角度 / 实际角度，° |
| I2 / I3 | 目标速度 / 实际速度，RPM |
| I4 / I5 | 累计编码器计数 / PWM（-500～500） |
| I6～I8 | XYZ角速度，°/s |
| I9～I11 | XYZ角加速度，°/s² |
| I12～I14 | roll / pitch / yaw，°（六轴yaw会漂移） |
| I15 / I16 | 模式0停机/1PWM/2速度/3角度，IMU读取有效标志 |

VOFA添加曲线控件，角度选I0/I1，速度选I2/I3。旧版21/22通道布局已删除。

## 必须确认的两个配置

`Core/Inc/config.h`：

- ENCODER_CPR=2112是未核实值。正确RPM必须使用输出轴一圈的四倍频计数：资料线数×4×减速比（注意资料定义）。Gitee仓库没有电机参数。
- ANGLE_USE_ROLL=0、ANGLE_SIGN=-1：当前用负pitch。IMU相应轴需平行电机轴，且正PWM使实际角度增加；换安装方向需修改配置。精简版不自动处理任意斜装。

角度环输出目标RPM，速度环输出PWM；速度PID初值25/80/0，角度PID初值1.5/0/0.05，均在main.c的USER CODE BEGIN 2处。
先整定速度环，再整定角度环。角度模式持续按误差调整，只有发送q或姿态读取失败才停止。
主机模型通过不能保证真实电机稳定，尤其CPR、静摩擦和IMU噪声仍需实测。

## 验收与版本管理

`./tests/run.ps1`：验证实际Gitee移植代码参与的五组主机模型测试（需要Python和C编译器）。
[逐项验收](docs/ASSESSMENT.md)、[验证记录](docs/VALIDATION.md)。
GitHub私有仓库：[tatami-007/STM32-Control-Assessment](https://github.com/tatami-007/STM32-Control-Assessment)。
保留参考来源与ST HAL/CMSIS的许可证，不将Gitee原代码声明为自己原创。

## 先理解这三句话

- 编码器测速：这一小段时间的计数 ÷ 每圈计数 ÷ 时间 × 60，得到RPM。
- 速度PID：目标速度减实际速度，得到误差，再算出PWM。
- 角度PID：目标角度减实际角度，先算出需要的目标速度，交给速度PID。

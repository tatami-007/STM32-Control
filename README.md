# STM32 电控考核：基础版

保留四项基础要求：TB6612正反转调速、IMU姿态数据、PID速度环、角度外环+速度内环。
按使用者要求不做OLED和按键。上电不自动转动，控制由串口手动启动。

## 代码怎么读

当前按基础难度实现：普通PID、一个串口命令数组、HAL直接发送。去掉条件积分、测量值微分、异步发送状态和详细诊断。

PID只做这几步：

```c
error = target - actual;            // 误差
P = kp * error;                     // 当前差多少
I += ki * error * dt;               // 累加误差
D = kd * (error - last_error) / dt;  // 误差变化多快
output = P + I + D;
```

pid.c额外保留积分和输出限幅，参数仍需实机整定。

**第一次只读这两处：main.c的USER CODE BEGIN 2（启动准备）和USER CODE BEGIN 3（循环执行）。**
其中`Process_Command`负责把电脑发来的文字转换成目标值，串口回调负责收发字符，可以在理解控制流程后再学习。

| 变量 | 含义 |
|---|---|
| control_mode | 当前是停机、直接PWM、速度控制还是角度控制 |
| target_angle / actual_angle | 希望达到的角度 / 现在的角度 |
| target_speed / actual_speed | 希望达到的转速 / 编码器测到的转速 |
| target_pwm | 直接PWM模式的输出，正数正转、负数反转 |
| dt | 上次控制到本次控制经过的秒数 |
| ins->pitch | ins指向的传感器数据中的俯仰角 |
| show_data | 是否向VOFA连续发送数值 |

两层PID最核心的计算是：

```c
target_speed = PID_Update(&angle_pid, target_angle, actual_angle, dt);
float pwm_output = PID_Update(&speed_pid, target_speed, actual_speed, dt);
Motor_Set((int)pwm_output);
```

第一句根据角度误差决定需要多快，第二句根据速度误差决定PWM，第三句把PWM输出到电机。
`pid.c`按P、I、D分别计算；`motor.c`用if/else明确设置正反转引脚。

**精简范围仅限我新增的代码。main.c中USER CODE区域之外的CubeMX代码完整保留，Gitee的封装文件也保留。**
tests/check_cube_main.py对照本次修改前备份校验生成区；以后主动用CubeMX变更外设配置重新生成时，需核对并更新校验基线。

1. **先看main.c的USER CODE BEGIN 2和while(1)**：初始化后，按1～5的注释执行收指令→读数据→算角度PID→算速度PID→发送数据。下面几百行MX初始化函数是CubeMX生成的，初学时先略过。
2. MPU6050按原README调用：`ins_init();`初始化，`ins_data_t *ins=ins_data();`取指针，循环`ins_update();`后读`ins->pitch`、`ins->gyro`等数据。
3. `motor.c`：PWM与方向；`pid.c`：通用PID。
4. 串口收发已经合并到main.c的USER CODE BEGIN 4；自写.c文件只剩motor.c和pid.c。
5. Gitee封装：`mpu6050.c`、`ins.c`直接放在`Core/Src`，`mpu6050.h`、`ins.h`直接放在`Core/Inc`，通过`#include "ins.h"`调用。修改记录见[PORTING.md](docs/mpu6050/PORTING.md)。

主流程直接使用HAL串口函数。接收中断收齐一行后交给main，发送直接HAL_UART_Transmit，没有异步TX状态或发送完成回调。
数字指令用sscanf读取。收到提示后再发下一条指令，q仍可以随时请求停机。
Motor_Set直接执行PWM输出，换向仍保留100ms停顿；无需另外调用Motor_Task。

基础版本只有三种运行方式：直接PWM、速度PID、角度PID。
角度模式仅在IMU读取失败时停止；q手动急停、串口错误停机、PWM限幅和换向等待仍保留。

## 构建与烧录

```powershell
./tools/build.ps1 -Preset Release
./tools/flash.ps1 -Preset Release
```

脚本可指定工具链/OpenOCD路径。完整工程包括CubeMX .ioc、启动文件、链接脚本和HAL/CMSIS。
上电保持IMU静止约2秒，等READY。失败时显示IMU INVALID；发r重新初始化和校准，成功后仍停机，需重新发z设零。

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
PWM为1kHz；编码器TI12四倍频、16位计数；IMU配置100Hz、控制名义50Hz。
每20ms尝试计算一次PID，发送数据会占用时间，所以dt仍使用实际采样间隔。

## VOFA操作

FireWater、COM5（以设备实际端口为准）、115200、8N1、无流控。

| 指令 | 作用 |
|---|---|
| `pwm:300` / `pwm:-300` | 正/反转30%PWM，范围±500 |
| `speed:3` / `speed:-3` | 速度PID目标RPM，范围±30 |
| `z` | 把当前棒位置设零，立即启动0度保持；必须先摆水平并静止 |
| `r` | 停机，重新初始化IMU并静止校准约2秒；成功后再发z |
| `angle:5` / `angle:-5` | 相对零点目标角度，范围±60° |
| `q` / `0` | 停机 |
| `p` / `v` | 暂停/恢复数值，不影响电机控制 |

**除急停q外，所有指令都追加LF或CRLF**。发送框例如填speed:3，并选择追加换行。
默认只显示启动/命令/故障提示。发v显示曲线数值，发p暂停数值，避免刷走提示；不影响电机控制。

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

`./tests/run.ps1`：验证实际Gitee移植代码参与的六组主机模型测试（需要Python和C编译器）。
角度测试包含持续输出数据时115200串口发送耗时。当前基础版已编译、模型测试通过，尚未烧录验证真实角度效果。
[逐项验收](docs/ASSESSMENT.md)、[验证记录](docs/VALIDATION.md)。
GitHub私有仓库：[tatami-007/STM32-Control-Assessment](https://github.com/tatami-007/STM32-Control-Assessment)。
保留参考来源与ST HAL/CMSIS的许可证，不将Gitee原代码声明为自己原创。

## 先理解这三句话

- 编码器测速：这一小段时间的计数 ÷ 每圈计数 ÷ 时间 × 60，得到RPM。
- 速度PID：目标速度减实际速度，得到误差，再算出PWM。
- 角度PID：目标角度减实际角度，先算出需要的目标速度，交给速度PID。

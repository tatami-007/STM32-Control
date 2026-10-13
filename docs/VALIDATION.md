# 精简版验证记录

日期：2026-10-13。以使用者“最简洁易懂，包含Gitee代码”的最新要求为准。

## 当前基础版

- 普通位置式PID：P乘当前误差，I累加误差，D对误差差分，保留积分和输出限幅；去掉条件积分和测量值微分。
- UART一个接收数组、一个完成标志，发送直接HAL_UART_Transmit；去掉异步TX状态、完成回调和详细IMU诊断结构。
- 数字指令用sscanf读取，保留范围判断及数字后多余字符检查；r手动重试、z设零、p/v暂停显示保留。
- 控制周期20ms，dt仍使用实际间隔；主机模型按115200、8N1模拟发送耗时，角度测试在持续输出数据的条件下通过。
- 六组模型测试和CubeMX生成区校验通过，Debug/Release构建通过；Debug Flash57576B，Release47176B，静态RAM4000B。
- main用户代码、pid.c和mpu6050.c合计比上一版减少64个非空行（含注释）。
- 当前基础版尚未烧录，真实PID参数和角度跟踪仍待实机验证。下方实机记录是此前版本的验证。

## 此前修改记录

- Gitee四个原始文件复制后移植，两个.c直接放在Core/Src，两个.h直接放在Core/Inc，实际参与构建；差异见docs/mpu6050/PORTING.md。
- 主流程直接在main.c，不再有App_Init/App_Task；按原README直接调用ins_init/ins_data/ins_update。
- 初学者版进一步移除运行时CPR/辅助指令、定时状态消息、辅助角度函数，主程序净减少89行、串口净减少32行。
- 指令统一追加换行（急停q例外），PID与采样直接在main.c按1～5注释执行。
- main.c生成区与before_assessment_rewrite备份逐段一致，SystemClock_Config、MX_I2C1_Init、MX_TIM2_Init、MX_TIM3_Init、MX_USART1_UART_Init、MX_GPIO_Init和Error_Handler完整保留；校验纳入测试入口。
- Debug/Release ARM构建通过，无编译警告；RAM约4.6KB，Flash Debug约55KB、Release约45KB。
- 串口收发已合并到main.c，serial.c/.h删除；CubeMX生成文件全部保留，自写.c文件只剩motor.c/pid.c。
- 再次减少串口辅助函数，main直接调用HAL收发；motor去掉请求/任务两层，Motor_Set直接执行输出，仍保留限幅和100ms换向等待。
- 六组主机模型测试通过，直接提取main.c的USER CODE区测试，连同motor/pid及Gitee移植的mpu6050/ins：
  PWM正反转和换向等待；正负速度及超过30秒持续运行；正负角度及外壳倾斜补偿；
  I²C读取失败停止且不自动恢复；串口参数解析、NaN拒绝、未完成输入行中q停机。
- 新增启动读取失败、校准读取失败、r重试成功且仍停机、重新设零后角度控制恢复的回归测试。
- 后续可读性修改：主流程变量改用明确名称，展开命令判断、角度选择和电机正反转；PID分开写P/I/D及积分限幅。六组测试再次通过，生成区校验通过，Debug/Release编译通过；此次源码整理未重新烧录。
- 覆盖16位编码器溢出和I²C读取完成时间晚于主循环入口的时间戳检查。
- 静止零偏为Gitee的平均采样法，不再做统计状态机；上电必须实际静止。

## 实机

- 后续基础版故障诊断：COM5发送q、p得到STOP/PAUSED，发送r仍返回IMU INVALID。
- OpenOCD核对板上Debug固件与本地ELF匹配；I²C错误码0x20、SR2 BUSY=1、INS未初始化。
- GPIOB IDR显示PB6/PB7均低。停机后暂停CPU、关闭I²C并把这两个引脚设为浮空输入、内部上拉输入，两次读取仍均低；已恢复原GPIO/I²C设置并运行CPU。
- 已确认总线低电平导致通信失败，尚未确定具体是供电、接线、模块还是引脚硬件问题；待用户检查接线。本次未改变PID或添加复杂诊断到学习代码，只修改了误导性的KEEP STILL错误提示。
- 后续设备重新接入：COM5和ST-LINK可用，Debug版已烧录，OpenOCD Verified OK。
- 初次读取旧固件时，姿态冻结、IMU有效标志为0；此前具体I²C错误未记录，不能据此断言故障发生在初始化还是运行读取。
- 新固件启动READY，手动r重试后IMU READY；12秒串口记录243帧全部有效，PWM一直为0。结束时停机、数值输出暂停。
- 实机角度闭环、准确RPM和长时间稳定性仍需验证；本次未让电机运动。
- 前版能转但抖动/漂移的结果不能算本版通过。
- 下一步确认CPR、固定IMU安装方向、整定速度/角度环。

## 待提交

- 两个实物演示视频、专业/班级/姓名尚缺。
- OLED/按键按使用者要求跳过。
- 私有GitHub仓库已创建并推送：https://github.com/tatami-007/STM32-Control-Assessment 。

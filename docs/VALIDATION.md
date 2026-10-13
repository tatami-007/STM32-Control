# 精简版验证记录

日期：2026-10-13。以使用者“最简洁易懂，包含Gitee代码”的最新要求为准。

## 软件

- Gitee四个原始文件直接复制到mpu6050后移植，实际参与构建；差异见mpu6050/PORTING.md。
- 主流程直接在main.c，不再有App_Init/App_Task；按原README直接调用ins_init/ins_data/ins_update。
- 初学者版进一步移除运行时CPR/辅助指令、定时状态消息、辅助角度函数，主程序净减少89行、串口净减少32行。
- 指令统一追加换行（急停q例外），PID与采样直接在main.c按1～5注释执行。
- main.c生成区与before_assessment_rewrite备份逐段一致，SystemClock_Config、MX_I2C1_Init、MX_TIM2_Init、MX_TIM3_Init、MX_USART1_UART_Init、MX_GPIO_Init和Error_Handler完整保留；校验纳入测试入口。
- Debug/Release ARM构建通过，无编译警告；RAM约4.6KB，Flash Debug约55KB、Release约45KB。
- 五组主机模型测试通过，直接提取main.c的USER CODE区测试，连同serial/motor/pid及Gitee移植的mpu6050/ins：
  PWM正反转和换向等待；正负速度及超过30秒持续运行；正负角度及外壳倾斜补偿；
  I²C读取失败停止且不自动恢复；串口参数解析、NaN拒绝、未完成输入行中q停机。
- 覆盖16位编码器溢出和I²C读取完成时间晚于主循环入口的时间戳检查。
- 静止零偏为Gitee的平均采样法，不再做统计状态机；上电必须实际静止。

## 实机

- 本轮设备状态：COM5已消失，只枚举COM3/COM4；ST-LINK OpenOCD open failed。
- 新版未烧录，实机稳定性、准确RPM、最终角度跟踪尚未验证。
- 前版能转但抖动/漂移的结果不能算本版通过。
- 等设备接回后再烧录校验，确认CPR、固定IMU安装方向、整定速度/角度环。

## 待提交

- 两个实物演示视频、专业/班级/姓名尚缺。
- OLED/按键按使用者要求跳过。
- 私有GitHub仓库已创建并推送：https://github.com/tatami-007/STM32-Control-Assessment 。

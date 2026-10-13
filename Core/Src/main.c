/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "motor.h"
#include "pid.h"
#include "ins.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
enum { STOP, PWM, SPEED, ANGLE };   // 停机、直接PWM、速度PID、角度PID
static int control_mode = STOP;   // 当前选择哪一种控制方式
static int target_pwm;            // pwm指令设置的输出
static int show_data = 0;         // v显示数值，p暂停显示
static int zero_valid;            // 是否已经用z设零
static PID speed_pid, angle_pid;
static uint16_t last_count;
static int32_t total_count;        // 编码器累计计数
static uint32_t last_control, last_send;
static float actual_speed, target_speed;  // 实际/目标转速，RPM
static float actual_angle, target_angle, angle_zero; // 角度，单位度
static const char *message;       // 等待发给电脑的提示文字

/* 收到一行再交给main处理；中断中不做PID或浮点运算。 */
static uint8_t rx_byte;
static char rx_input[32], rx_completed[32];
static unsigned int rx_length;
static volatile int command_ready, tx_busy, serial_stop;
static int rx_discard;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void Process_Command(const char *command);
static void IMU_Error(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1); // 中断接收串口命令
    Motor_Start(&htim2); // 开启PWM，但初始输出为0
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
    ins_init();                      // 按Gitee README：初始化并静止校准
    ins_data_t *ins = ins_data();     // 取数据指针，之后通过ins读取结果
    PID_Init(&speed_pid, 25, 80, 0, PWM_LIMIT);             // 速度内环
    PID_Init(&angle_pid, 1.5f, 0, 0.05f, ANGLE_SPEED_MAX);   // 角度外环
    last_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    last_control = HAL_GetTick();
    message = "READY; z=ZERO, q=STOP\r\n";
    if (!ins->raw.valid) IMU_Error();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
    // 1. 收指令：中断收字符，这里处理收到的整行文字。
    if (serial_stop) {
        serial_stop = 0;
        command_ready = 0;
        Process_Command("q");
    }
    if (command_ready) {
        Process_Command(rx_completed); // 处理结束前，中断不会覆盖这条命令
        command_ready = 0;
    }
    uint32_t now = HAL_GetTick();
    if (now - last_control >= CONTROL_MS) {
        float dt = (now - last_control) / 1000.0f; // 实际采样间隔，秒
        last_control = now;

        // 2. 读取姿态和编码器，计算实际角度、实际转速。
        ins_update(); // Gitee封装自动更新ins中的姿态、角速度、角加速度
        float measured_angle = ins->pitch;
        if (ANGLE_USE_ROLL) measured_angle = ins->roll;
        actual_angle = ANGLE_SIGN * measured_angle - angle_zero;
        uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t delta = (int16_t)(count - last_count); // 16位差值，兼顾计数溢出
        last_count = count;
        total_count += delta * ENCODER_SIGN;
        float turns = delta * ENCODER_SIGN / ENCODER_CPR; // 这段时间转了几圈
        float rpm = turns / dt * 60.0f; // 每秒圈数乘60，得到每分钟圈数
        float filter_weight = dt / (0.04f + dt);
        actual_speed += filter_weight * (rpm - actual_speed); // 平滑测速，减小抖动

        // 3. 角度外环：把角度误差转换成目标转速。
        if (control_mode == ANGLE) {
            if (!ins->raw.valid) { // 读取失败不能继续用错误角度驱动电机
                control_mode = STOP;
                zero_valid = 0;
                IMU_Error();
            } else {
                target_speed = PID_Update(&angle_pid, target_angle, actual_angle, dt);
            }
        }

        // 4. 速度内环：把速度误差转换成PWM。
        if (control_mode == STOP) {
            Motor_Stop();
        } else if (control_mode == PWM) {
            Motor_Set(target_pwm);
        } else {
            float pwm_output = PID_Update(&speed_pid, target_speed, actual_speed, dt);
            Motor_Set((int)pwm_output);
        }
    }

    // 5. 发给VOFA：I0/I1角度，I2/I3速度，其余为编码器及IMU数据。
    if (!tx_busy) {
        const char *send_text = message; // 提示优先，没有提示再发数值
        if (!send_text && show_data && now - last_send >= 50U) {
            static char text[320]; // 发送完成前，文字必须一直保存在这个数组里
            last_send = now;
            snprintf(text, sizeof(text),
                "imu:%.2f,%.2f,%.2f,%.2f,%ld,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%u\r\n",
                target_angle, actual_angle, target_speed, actual_speed, (long)total_count, Motor_Output(),
                ins->gyro[0], ins->gyro[1], ins->gyro[2],
                ins->gyro_accel[0], ins->gyro_accel[1], ins->gyro_accel[2],
                ins->roll, ins->pitch, ins->yaw, control_mode, (unsigned int)ins->raw.valid);
            send_text = text;
        }
        if (send_text) {
            tx_busy = 1; // 提示和数据共用这一个发送位置
            if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)send_text, strlen(send_text)) == HAL_OK) {
                message = NULL;
            } else {
                tx_busy = 0;
            }
        }
    }
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void IMU_Error(void)
{
    static char text[128]; // 异步发送期间保持缓冲区不变
    message = "IMU INVALID; STOP; SEND r TO RETRY\r\n";
    if (tx_busy) return;
    bsp_mpu6050_status_t s = bsp_mpu6050_status();
    snprintf(text, sizeof(text), "IMU INVALID; reg=0x%02X status=%u error=0x%08lX id=0x%02X; SEND r\r\n",
             (unsigned)s.reg, (unsigned)s.status, (unsigned long)s.error, (unsigned)s.id);
    message = text;
}

/* 只保留考核会用到的命令；所有目标命令添加换行。 */
static void Process_Command(const char *command)
{
    // strcmp返回0表示两段文字相同，例如收到的文字是p。
    if (strcmp(command, "p") == 0) {
        show_data = 0;
        message = "PAUSED\r\n";
        return;
    }
    if (strcmp(command, "v") == 0) {
        show_data = 1;
        message = "RESUMED\r\n";
        return;
    }
    if (strcmp(command, "q") == 0 || strcmp(command, "0") == 0) {
        control_mode = STOP;
        target_speed = 0;
        message = "STOP\r\n";
    } else if (strcmp(command, "r") == 0) {
        // 重新初始化和静止校准；恢复成功后仍停机，须重新发送z设零。
        control_mode = STOP;
        target_speed = 0;
        zero_valid = 0;
        Motor_Stop();
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        ins_init();
        last_control = HAL_GetTick();
        actual_speed = 0;
        message = "IMU READY; SET LEVEL THEN SEND z\r\n";
        if (!ins_data()->raw.valid) IMU_Error();
    } else if (strcmp(command, "z") == 0) {
        const ins_data_t *ins = ins_data();
        if (!ins->raw.valid) {
            IMU_Error();
            return;
        }
        float measured_angle = ins->pitch;
        if (ANGLE_USE_ROLL) measured_angle = ins->roll;
        angle_zero = ANGLE_SIGN * measured_angle;
        actual_angle = 0;
        target_angle = 0;
        zero_valid = 1;
        control_mode = ANGLE;
        message = "ZERO SET; HOLD\r\n";
    } else {
        // 带数值的命令：先找到冒号，再把冒号后的文字转换成数字。
        const char *colon = strchr(command, ':');
        if (!colon) {
            message = "USE pwm:300 / speed:3 / angle:5\r\n";
            return;
        }
        char *end;
        float value = strtof(colon + 1, &end);
        // 没有数字、数字后还有乱码、或数字为NaN/无穷大，都不执行。
        if (end == colon + 1 || *end != '\0' || !isfinite(value)) {
            message = "INVALID NUMBER\r\n";
            return;
        }
        if (strncmp(command, "angle:", 6) == 0 && !zero_valid) {
            message = "ZERO NOT SET; SEND z (IF IMU INVALID, SEND r)\r\n";
            return;
        }
        // strncmp(...,4)==0表示前4个字符是pwm:，其他命令同理。
        if (strncmp(command, "pwm:", 4) == 0 && fabsf(value) <= PWM_LIMIT) {
            target_pwm = (int)value;
            control_mode = PWM;
        } else if (strncmp(command, "speed:", 6) == 0 && fabsf(value) <= SPEED_LIMIT) {
            target_speed = value;
            control_mode = SPEED;
        } else if (strncmp(command, "angle:", 6) == 0 && fabsf(value) <= ANGLE_LIMIT) {
            target_angle = value;
            control_mode = ANGLE;
        } else {
            message = "INVALID COMMAND / OUT OF RANGE\r\n";
            return;
        }
        message = "TARGET SET\r\n";
    }
    // 切换目标时清除上一次PID积分，避免旧输出影响新目标。
    Motor_Stop();
    PID_Reset(&speed_pid);
    PID_Reset(&angle_pid);
}

/* 串口回调：收满一行置标志；q立即请求停机。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart1) return;
    if (rx_byte == 'q' || rx_byte == 'Q') {
        serial_stop = 1;
        rx_length = 0;
        rx_discard = 0;
    }
    else if (rx_byte == '\n' || rx_byte == '\r') {
        if (rx_length && !rx_discard) {
            rx_input[rx_length] = 0;
            if (command_ready) serial_stop = 1; // 上一条未处理，停机而不是执行残缺命令
            else {
                strcpy(rx_completed, rx_input);
                __DMB(); // 先完成复制，再通知main可以读这行命令
                command_ready = 1;
            }
        }
        rx_length = 0;
        rx_discard = 0;
    } else if (!rx_discard) {
        if (rx_length < sizeof(rx_input) - 1) rx_input[rx_length++] = (char)rx_byte;
        else {
            rx_discard = 1;
            serial_stop = 1;
        }
    }
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK) serial_stop = 1;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) tx_busy = 0;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart1) return;
    serial_stop = 1;
    HAL_UART_AbortTransmit(&huart1);
    tx_busy = 0;
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

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
#include "serial.h"
#include "mpu6050/ins.h"
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
/* 仅四种模式，没有自动试转、辨轴、启动脉冲或计时停机流程。 */
enum { STOP, PWM, SPEED, ANGLE };
static int mode, pwm_target, stream = 1, zero_valid; // 运行模式、PWM目标、是否输出波形、是否已设零
static TIM_HandleTypeDef *encoder;
static PID speed_pid, angle_pid; // 速度内环、角度外环
static uint16_t last_count;
static int32_t total_count;
static uint32_t last_control, last_send, last_status;
static float cpr = ENCODER_CPR; // 输出轴每圈编码器计数
static float speed, target_speed; // 实际速度、目标速度，RPM
static float target_angle, angle_zero; // 目标角度、水平参考，度
static const char *message = "BOOT: MOTOR OFF; KEEP IMU STILL\r\n";
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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float sensor_angle(void) {
    const ins_data_t *data = ins_data();
    return ANGLE_SIGN * (ANGLE_USE_ROLL ? data->roll : data->pitch);
}
static float angle(void) {
    return sensor_angle() - angle_zero;
}
static void stop(void) {
    Motor_Stop();
    mode = STOP;
    pwm_target = 0;
    target_speed = 0;
    PID_Reset(&speed_pid);
    PID_Reset(&angle_pid);
}
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
    // 1. 开启串口、PWM和编码器；电机初始保持停止。

    encoder = &htim3;
    Serial_Init(&huart1);
    Motor_Start(&htim2);
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
    // 2. Gitee封装：初始化量程/采样率，静止采集零偏，再解算姿态。
    ins_init();                       // README第一步：初始化并静止校准
    ins_data_t *ins = ins_data();      // README第二步：获取数据指针，只取一次
    message = ins->raw.valid ? "IMU READY; z=ZERO; speed:N; angle:N; pwm:N; q=STOP\r\n"
                            : "IMU INIT FAILED; CHECK POWER/I2C; MOTOR OFF\r\n";
    // 3. 配置速度内环和角度外环。
    PID_Init(&speed_pid, 25, 80, 0, PWM_LIMIT);
    PID_Init(&angle_pid, 1.5f, 0, 0.05f, ANGLE_SPEED_MAX);
    last_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    last_control = HAL_GetTick();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 第一步：收串口指令，更新目标速度或目标角度。
    const char *command;
    while ((command = Serial_ReadCommand()) != NULL) Process_Command(command);
    Motor_Task();
    uint32_t now = HAL_GetTick();
    if (now - last_control >= CONTROL_MS) {
        float dt = (now - last_control) * 0.001f;
        last_control = now;
        // 第二步：读取MPU6050姿态和编码器，计算实际速度。
        ins_update();        // README第三步：周期更新，ins指向的数据自动更新
        now = HAL_GetTick(); // I2C结束后的时间，避免比INS时间戳早导致无符号下溢
        uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(encoder);
        int32_t delta = ((uint32_t)count - last_count) & 0xffffU;
        if (delta >= 32768) delta -= 65536; // 16位计数溢出也能正确计算正反转
        last_count = count;
        total_count += delta * ENCODER_SIGN;
        float measured = delta * ENCODER_SIGN * 60.0f / (cpr * dt);
        speed += dt / (0.04f + dt) * (measured - speed); // 编码器测速低通

        if (mode == ANGLE && (!ins->raw.valid || now - ins->timestamp > 100U)) {
            stop();
            zero_valid = 0;
            message = "STOP: IMU READ FAILED\r\n";
        }
        if (mode == PWM) Motor_Set(pwm_target);
        // 第三步：角度外环计算目标速度。
        if (mode == ANGLE) {
            // 角度外环输出目标RPM；速度内环输出PWM。
            target_speed = PID_Update(&angle_pid, target_angle, angle(), dt);
            if (fabsf(target_angle - angle()) < 0.5f && fabsf(speed) < 0.5f) {
                target_speed = 0;
                PID_Reset(&speed_pid);
                PID_Reset(&angle_pid);
                Motor_Stop();
            }
        }
        // 第四步：速度内环计算PWM，让电机追上目标速度。
        if (mode == SPEED || (mode == ANGLE && target_speed != 0))
            Motor_Set((int)lroundf(PID_Update(&speed_pid, target_speed, speed, dt)));
        else if (mode == ANGLE)
            Motor_Stop();
    }
    // 第五步：把目标/实际值发给VOFA。
    // 非阻塞发送：状态优先；波形20Hz，控制100Hz，不互相等待。
    if (!Serial_Busy()) {
        if (message) {
            if (Serial_Send(message)) message = 0;
        } else if (stream && now - last_send >= 50U) {
            static char frame[320];
            last_send = now;
            snprintf(frame, sizeof(frame),
                     "imu:%.2f,%.2f,%.2f,%.2f,%ld,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%"
                     "d,%u\r\n",
                     target_angle, angle(), target_speed, speed, (long)total_count, Motor_Output(),
                     ins->gyro[0], ins->gyro[1], ins->gyro[2], ins->gyro_accel[0],
                     ins->gyro_accel[1], ins->gyro_accel[2], ins->roll, ins->pitch, ins->yaw,
                     mode, (unsigned int)ins->raw.valid);
            Serial_Send(frame);
        } else if (!stream && now - last_status >= 1000U) {
            static char status[100];
            last_status = now;
            snprintf(status, sizeof(status),
                     "CTRL mode=%d target=%.2f angle=%.2f pwm=%d imu=%u\r\n", mode, target_angle,
                     angle(), Motor_Output(), (unsigned int)ins->raw.valid);
            Serial_Send(status);
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
/* 串口命令处理：主流程在main的while(1)，这里只修改目标和模式。 */
static void Process_Command(const char *command) {
    if (!strcmp(command, "q") || !strcmp(command, "0")) {
        stop();
        message = "STOP: USER\r\n";
        return;
    }
    if (!strcmp(command, "p")) {
        stream = 0;
        message = "VALUES PAUSED\r\n";
        return;
    }
    if (!strcmp(command, "v")) {
        stream = 1;
        message = "VALUES RESUMED\r\n";
        return;
    }
    if (!strcmp(command, "h")) {
        stop();
        message = "PLACE LEVEL, THEN z\r\n";
        return;
    }
    if (!strcmp(command, "z")) {
        stop();
        if (!ins_data()->raw.valid || HAL_GetTick() - ins_data()->timestamp > 100U) {
            message = "ZERO FAILED: IMU INVALID\r\n";
            return;
        }
        angle_zero = sensor_angle();
        zero_valid = 1;
        target_angle = 0;
        mode = ANGLE;
        message = "ZERO SET; HOLDING THIS POSITION\r\n";
        return;
    }
    const char *colon = strchr(command, ':');
    if (!colon) {
        message = "USE pwm:300 / speed:3 / angle:5 + NEWLINE\r\n";
        return;
    }
    char *end;
    float value = strtof(colon + 1, &end);
    while (*end == ' ')
        ++end;
    if (end == colon + 1 || *end || !isfinite(value)) {
        message = "INVALID NUMBER\r\n";
        return;
    }
    if (!strncmp(command, "pwm:", 4) && fabsf(value) <= PWM_LIMIT) {
        stop();
        pwm_target = (int)lroundf(value);
        mode = PWM;
    } else if (!strncmp(command, "speed:", 6) && fabsf(value) <= SPEED_LIMIT) {
        stop();
        target_speed = value;
        mode = SPEED;
    } else if (!strncmp(command, "angle:", 6) && fabsf(value) <= ANGLE_LIMIT && zero_valid &&
               ins_data()->raw.valid) {
        stop();
        target_angle = value;
        mode = ANGLE;
    } else if (!strncmp(command, "cpr:", 4) && value >= 4 && value <= 1000000) {
        stop();
        cpr = value;
        speed = 0;
    } else {
        message = "REJECTED: RANGE / NEED z FIRST\r\n";
        return;
    }
    message = "TARGET UPDATED\r\n";
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

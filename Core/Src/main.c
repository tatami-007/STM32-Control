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
enum { STOP, PWM, SPEED, ANGLE };   /* 控制模式：停机、PWM、速度闭环、角度闭环 */
static int control_mode = STOP;   /* 当前控制模式 */
static int target_pwm;            /* PWM 模式的有符号输出 */
static int show_data = 0;         /* 遥测开关：v 开启，p 关闭 */
static int zero_valid;            /* 角度零点有效标志 */
static PID speed_pid = {.kp = 25, .ki = 80, .kd = 0, .limit = PWM_LIMIT}; /* 速度环 */
static PID angle_pid = {.kp = 1.5f, .ki = 0, .kd = 0.05f, .limit = ANGLE_SPEED_MAX}; /* 角度环 */
static uint16_t last_count;
static int32_t total_count;        /* 编码器累计计数 */
static uint32_t last_control, last_send;
static uint32_t last_imu_ok;       /* 上次有效姿态数据的时间，单位 ms */
static float actual_speed, target_speed;  /* 实际转速、目标转速，单位 RPM */
static float actual_angle, target_angle, angle_zero; /* 实际角度、目标角度、零点偏置，单位 ° */
static const char *message;       /* 待发送的状态消息 */

/* 中断负责接收字符，主循环负责解析完整命令。 */
static uint8_t rx_byte;
static char rx_input[32];          /* 串口命令缓冲区 */
static unsigned int rx_length;
static volatile int command_ready, serial_stop;
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
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1); /* 启动单字节中断接收 */
    Motor_Start(&htim2); /* 启动 PWM，初始占空比为 0 */
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
    ins_init();                      /* 初始化 IMU 并校准陀螺仪零偏 */
    ins_data_t *ins = ins_data();     /* 姿态解算结果 */
    last_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    last_control = HAL_GetTick();
    last_imu_ok = last_control;
    message = "READY; z=ZERO, q=STOP\r\n";
    if (!ins->raw.valid) message = "IMU INVALID; CHECK POWER/WIRING; SEND r\r\n";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
    /* 处理停机请求和完整串口命令。 */
    if (serial_stop) {
        serial_stop = 0;
        command_ready = 0;
        Process_Command("q");
    }
    if (command_ready) {
        Process_Command(rx_input); /* 处理期间，接收中断不改写命令缓冲区 */
        command_ready = 0;
    }
    uint32_t now = HAL_GetTick();
    if (now - last_control >= CONTROL_MS) {
        /* 采集姿态和编码器数据，计算角度与转速。 */
        ins_update();
        uint32_t imu_now = HAL_GetTick();
        float dt = (imu_now - last_control) / 1000.0f; /* 实际采样周期，单位 s，包含 I2C 读取耗时 */
        last_control = imu_now;
        int imu_expired = imu_now - last_imu_ok > 200U; /* 姿态数据连续失效超过 200 ms */
        if (ins->raw.valid) last_imu_ok = imu_now;
        actual_angle = ANGLE_SIGN * ins->pitch - angle_zero; /* 以俯仰角计算相对零点的角度 */
        uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t delta = (int16_t)(count - last_count); /* 有符号差值处理 16 位计数器回绕 */
        last_count = count;
        total_count += delta * ENCODER_SIGN;
        float turns = delta * ENCODER_SIGN / ENCODER_CPR; /* 采样周期内的输出轴转数 */
        float rpm = turns / dt * 60.0f; /* 输出轴转速，单位 RPM */
        float filter_weight = dt / (0.04f + dt);
        actual_speed += filter_weight * (rpm - actual_speed); /* 一阶低通滤波，时间常数 40 ms */

        /* 角度外环根据角度误差生成目标转速。 */
        if (control_mode == ANGLE) {
            if (imu_expired) { /* 数据持续失效时停机并清除零点 */
                control_mode = STOP;
                zero_valid = 0;
                message = "IMU LOST >200ms; STOP; CHECK WIRING; SEND r\r\n";
            } else if (!ins->raw.valid) {
                /* 短时读取失败时暂停输出并重置 I2C，恢复后沿用原零点。 */
                Motor_Stop();
                PID_Reset(&speed_pid);
                PID_Reset(&angle_pid);
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
            } else {
                target_speed = PID_Update(&angle_pid, target_angle, actual_angle, dt);
            }
        }

        /* 速度内环计算 PWM；停机或角度模式下姿态无效时输出为 0。 */
        int pwm_output = 0;
        if (control_mode == PWM) pwm_output = target_pwm;
        if (control_mode == SPEED || (control_mode == ANGLE && ins->raw.valid))
            pwm_output = (int)PID_Update(&speed_pid, target_speed, actual_speed, dt);
        Motor_Set(pwm_output);
    }

    /* 状态消息优先发送；遥测使用 FireWater 格式，每 50 ms 发送一帧。 */
    if (message) {
        HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 50);
        message = NULL;
    } else if (show_data && now - last_send >= 50U) {
        char text[320];
        last_send = now;
        snprintf(text, sizeof(text),
            "imu:%.2f,%.2f,%.2f,%.2f,%ld,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%u\r\n",
            target_angle, actual_angle, target_speed, actual_speed, (long)total_count, Motor_Output(),
            ins->gyro[0], ins->gyro[1], ins->gyro[2],
            ins->gyro_accel[0], ins->gyro_accel[1], ins->gyro_accel[2],
            ins->roll, ins->pitch, ins->yaw, control_mode, (unsigned int)ins->raw.valid);
        HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 50);
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
/**
 * @brief 解析串口命令并更新控制目标。
 * @param command 以空字符结尾、不含行结束符的命令字符串。
 */
static void Process_Command(const char *command)
{
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
        /* 重新初始化并校准 IMU，清除零点后保持停机。 */
        control_mode = STOP;
        target_speed = 0;
        zero_valid = 0;
        Motor_Stop();
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        ins_init();
        last_control = HAL_GetTick();
        last_imu_ok = last_control;
        actual_speed = 0;
        message = "IMU READY; SET LEVEL THEN SEND z\r\n";
        if (!ins_data()->raw.valid) message = "IMU INVALID; CHECK POWER/WIRING; SEND r\r\n";
    } else if (strcmp(command, "z") == 0) {
        const ins_data_t *ins = ins_data();
        if (!ins->raw.valid) {
            message = "IMU INVALID; CHECK POWER/WIRING; SEND r\r\n";
            return;
        }
        angle_zero = ANGLE_SIGN * ins->pitch;
        actual_angle = 0;
        target_angle = 0;
        zero_valid = 1;
        control_mode = ANGLE;
        message = "ZERO SET; HOLD\r\n";
    } else {
        /* 仅接受一个数值参数，拒绝数值后的额外字符。 */
        float value;
        char extra;
        if (sscanf(command, "pwm:%f%c", &value, &extra) == 1 && fabsf(value) <= PWM_LIMIT) {
            target_pwm = (int)value;
            control_mode = PWM;
        } else if (sscanf(command, "speed:%f%c", &value, &extra) == 1 && fabsf(value) <= SPEED_LIMIT) {
            target_speed = value;
            control_mode = SPEED;
        } else if (sscanf(command, "angle:%f%c", &value, &extra) == 1 && fabsf(value) <= ANGLE_LIMIT) {
            if (!zero_valid) {
                message = "ZERO NOT SET; SEND z\r\n";
                return;
            }
            target_angle = value;
            control_mode = ANGLE;
        } else {
            message = "INVALID COMMAND / OUT OF RANGE\r\n";
            return;
        }
        message = "TARGET SET\r\n";
    }
    /* 更新控制目标后，清除两个 PID 的积分和历史误差。 */
    Motor_Stop();
    PID_Reset(&speed_pid);
    PID_Reset(&angle_pid);
}

/**
 * @brief 缓存串口命令，收到 q 或 Q 时立即置停机请求标志。
 * @param uart 产生接收完成中断的串口句柄。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart1) return;
    if (rx_byte == 'q' || rx_byte == 'Q') {
        serial_stop = 1;
        rx_length = 0;
        rx_discard = 0;
    }
    else if (command_ready) {
        /* 保留待处理命令，仅接受 q 或 Q 停机请求。 */
    } else if (rx_byte == '\n' || rx_byte == '\r') {
        if (rx_length && !rx_discard) {
            rx_input[rx_length] = 0;
            command_ready = 1;
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
/**
 * @brief 串口接收出错时请求停机，并重新启动中断接收。
 * @param uart 产生错误中断的串口句柄。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart1) return;
    serial_stop = 1;
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

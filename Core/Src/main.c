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
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STEPPER_CMD_NONE = 0,
  STEPPER_CMD_FWD_LOW,
  STEPPER_CMD_FWD_HIGH,
  STEPPER_CMD_REV_LOW,
  STEPPER_CMD_REV_HIGH
} StepperCommand_t;

typedef enum
{
  PWM_MOTOR_CMD_NONE = 0,
  PWM_MOTOR_CMD_FWD_LOW,
  PWM_MOTOR_CMD_FWD_HIGH,
  PWM_MOTOR_CMD_REV_LOW,
  PWM_MOTOR_CMD_REV_HIGH
} PwmMotorCommand_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* FG 黄线每转输出 6 个脉冲。 */
#define MOTOR_FG_PULSES_PER_REV      6U
/* TIM2 当前配置为 1 kHz PWM，Period=999，所以 60% 占空比对应 Compare=600。 */
#define MOTOR_PWM_TIMER_PERIOD       999U
/* 测试时轮流切换两档速度，占空比相差一倍。 */
#define MOTOR_PWM_DUTY_LOW_PERCENT   30U
#define MOTOR_PWM_DUTY_HIGH_PERCENT  60U
/* 当前默认方向：SET/RESET 对应实际正反转以电机驱动实物为准。 */
#define MOTOR_DIR_DEFAULT            GPIO_PIN_SET
/* 每 1000 ms 统计一次转速并通过 USART1 发给上位机。 */
#define MOTOR_RPM_REPORT_PERIOD_MS   1000U
/* 主循环里每 3 秒切换一次 PWM 占空比。 */
#define MOTOR_PWM_SWITCH_PERIOD_MS   3000U
/* 以下参数只给“未启用”的上位机控制占位函数使用。 */
#define STEPPER_UART_LOW_RPS         3U
#define STEPPER_UART_HIGH_RPS        6U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint32_t motor_fg_pulse_count = 0;
static uint32_t motor_last_report_tick = 0;
static uint32_t motor_last_report_pulse_count = 0;
static uint32_t motor_last_switch_tick = 0;
static uint32_t motor_rpm = 0;
static uint32_t motor_pwm_duty_percent = MOTOR_PWM_DUTY_LOW_PERCENT;
static uint8_t motor_uart_tx_buf[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t Motor_GetPwmCompareFromDuty(uint32_t duty_percent);
static void Motor_UpdateTestSpeedIfReady(void);
static void Motor_ReportRpmIfReady(void);
static void Stepper_UartControlPlaceholder(uint8_t rx_byte);
static void PwmMotor_UartControlPlaceholder(uint8_t rx_byte);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 把占空比百分比转换成 TIM2_CH1 的比较值。 */
static uint32_t Motor_GetPwmCompareFromDuty(uint32_t duty_percent)
{
  if (duty_percent >= 100U)
  {
    return MOTOR_PWM_TIMER_PERIOD;
  }

  return ((MOTOR_PWM_TIMER_PERIOD + 1U) * duty_percent) / 100U;
}

/* 为了测速，主循环里每 3 秒在两档速度之间来回切换。 */
static void Motor_UpdateTestSpeedIfReady(void)
{
  uint32_t now_tick = HAL_GetTick();

  if ((now_tick - motor_last_switch_tick) < MOTOR_PWM_SWITCH_PERIOD_MS)
  {
    return;
  }

  motor_last_switch_tick = now_tick;
  if (motor_pwm_duty_percent == MOTOR_PWM_DUTY_LOW_PERCENT)
  {
    motor_pwm_duty_percent = MOTOR_PWM_DUTY_HIGH_PERCENT;
  }
  else
  {
    motor_pwm_duty_percent = MOTOR_PWM_DUTY_LOW_PERCENT;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Motor_GetPwmCompareFromDuty(motor_pwm_duty_percent));
}

/* 每秒统计一次 FG 脉冲，换算成 rpm 后通过 USART1 发送给上位机。 */
static void Motor_ReportRpmIfReady(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint32_t elapsed_ms;
  uint32_t pulse_count_snapshot;
  uint32_t pulse_delta;
  int msg_len;

  elapsed_ms = now_tick - motor_last_report_tick;
  if (elapsed_ms < MOTOR_RPM_REPORT_PERIOD_MS)
  {
    return;
  }

  __disable_irq();
  pulse_count_snapshot = motor_fg_pulse_count;
  __enable_irq();

  pulse_delta = pulse_count_snapshot - motor_last_report_pulse_count;
  motor_last_report_pulse_count = pulse_count_snapshot;
  motor_last_report_tick = now_tick;

  if (elapsed_ms > 0U)
  {
    motor_rpm = (pulse_delta * 60000U) / (MOTOR_FG_PULSES_PER_REV * elapsed_ms);
  }
  else
  {
    motor_rpm = 0U;
  }

  msg_len = snprintf((char *)motor_uart_tx_buf, sizeof(motor_uart_tx_buf), "rpm=%lu\r\n", motor_rpm);
  if (msg_len > 0)
  {
    HAL_UART_Transmit(&huart1, motor_uart_tx_buf, (uint16_t)msg_len, 50);
  }
}

/* 未启用：预留给步进电机的上位机串口控制入口，后续收到命令字节后再调用。 */
static void Stepper_UartControlPlaceholder(uint8_t rx_byte)
{
  StepperCommand_t command = STEPPER_CMD_NONE;
  GPIO_PinState dir = GPIO_PIN_RESET;
  uint32_t target_rps = 0U;

  switch (rx_byte)
  {
    case '1':
      command = STEPPER_CMD_FWD_LOW;
      dir = GPIO_PIN_SET;
      target_rps = STEPPER_UART_LOW_RPS;
      break;

    case '2':
      command = STEPPER_CMD_FWD_HIGH;
      dir = GPIO_PIN_SET;
      target_rps = STEPPER_UART_HIGH_RPS;
      break;

    case '3':
      command = STEPPER_CMD_REV_LOW;
      dir = GPIO_PIN_RESET;
      target_rps = STEPPER_UART_LOW_RPS;
      break;

    case '4':
      command = STEPPER_CMD_REV_HIGH;
      dir = GPIO_PIN_RESET;
      target_rps = STEPPER_UART_HIGH_RPS;
      break;

    default:
      break;
  }

  if (command == STEPPER_CMD_NONE)
  {
    return;
  }

  /*
   * TODO: 这里后续接步进电机控制逻辑。
   * dir 表示方向，target_rps 表示目标转速档位。
   * 例如：Stepper_SetDirectionAndSpeed(dir, target_rps);
   */
  (void)dir;
  (void)target_rps;
}

/* 未启用：预留给 PA0/TIM2_CH1 PWM 电机的上位机串口控制入口。 */
static void PwmMotor_UartControlPlaceholder(uint8_t rx_byte)
{
  PwmMotorCommand_t command = PWM_MOTOR_CMD_NONE;
  GPIO_PinState dir = MOTOR_DIR_DEFAULT;
  uint32_t duty_percent = motor_pwm_duty_percent;

  switch (rx_byte)
  {
    case 'A':
      command = PWM_MOTOR_CMD_FWD_LOW;
      dir = GPIO_PIN_SET;
      duty_percent = MOTOR_PWM_DUTY_LOW_PERCENT;
      break;

    case 'B':
      command = PWM_MOTOR_CMD_FWD_HIGH;
      dir = GPIO_PIN_SET;
      duty_percent = MOTOR_PWM_DUTY_HIGH_PERCENT;
      break;

    case 'C':
      command = PWM_MOTOR_CMD_REV_LOW;
      dir = GPIO_PIN_RESET;
      duty_percent = MOTOR_PWM_DUTY_LOW_PERCENT;
      break;

    case 'D':
      command = PWM_MOTOR_CMD_REV_HIGH;
      dir = GPIO_PIN_RESET;
      duty_percent = MOTOR_PWM_DUTY_HIGH_PERCENT;
      break;

    default:
      break;
  }

  if (command == PWM_MOTOR_CMD_NONE)
  {
    return;
  }

  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, dir);
  motor_pwm_duty_percent = duty_percent;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Motor_GetPwmCompareFromDuty(motor_pwm_duty_percent));
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  motor_last_report_tick = HAL_GetTick();
  motor_last_switch_tick = HAL_GetTick();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Motor_GetPwmCompareFromDuty(motor_pwm_duty_percent));
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_DIR_DEFAULT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Motor_UpdateTestSpeedIfReady();
    Motor_ReportRpmIfReady();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MOTOR_FG_Pin)
  {
    motor_fg_pulse_count++;
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

#ifdef  USE_FULL_ASSERT
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

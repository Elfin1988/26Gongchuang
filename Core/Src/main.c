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
  FRICTION_3650_A_CMD_NONE = 0,
  FRICTION_3650_A_CMD_FWD_LOW,
  FRICTION_3650_A_CMD_FWD_HIGH,
  FRICTION_3650_A_CMD_REV_LOW,
  FRICTION_3650_A_CMD_REV_HIGH
} Friction3650ACommand_t;

typedef enum
{
  FRICTION_3650_B_MOTOR_CMD_NONE = 0,
  FRICTION_3650_B_MOTOR_CMD_FWD_LOW,
  FRICTION_3650_B_MOTOR_CMD_FWD_HIGH,
  FRICTION_3650_B_MOTOR_CMD_REV_LOW,
  FRICTION_3650_B_MOTOR_CMD_REV_HIGH
} Friction3650BMotorCommand_t;

typedef enum
{
  BELT_DRIVE_CMD_NONE = 0,
  BELT_DRIVE_CMD_STOP,
  BELT_DRIVE_CMD_REV_LOW,
  BELT_DRIVE_CMD_REV_HIGH
} BeltDriveCommand_t;

typedef enum
{
  MECANUM_WHEEL_A = 0,
  MECANUM_WHEEL_B,
  MECANUM_WHEEL_C,
  MECANUM_WHEEL_D
} MecanumWheel_t;

typedef enum
{
  MECANUM_STOP = 0,
  MECANUM_FORWARD,
  MECANUM_REVERSE
} MecanumDirection_t;

typedef enum
{
  MECANUM_DEMO_STAGE_FORWARD = 0,
  MECANUM_DEMO_STAGE_STOP_AFTER_FORWARD,
  MECANUM_DEMO_STAGE_REVERSE,
  MECANUM_DEMO_STAGE_STOP_AFTER_REVERSE
} MecanumDemoStage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 3650-A motor control constants */
#define FRICTION_3650_A_FG_PULSES_PER_REV      6U
#define FRICTION_3650_A_PWM_TIMER_PERIOD       999U
#define FRICTION_3650_A_PWM_DUTY_LOW_PERCENT   16U
#define FRICTION_3650_A_PWM_DUTY_HIGH_PERCENT  30U
#define FRICTION_3650_A_DIR_DEFAULT            GPIO_PIN_SET

/* 3650-B motor control constants */
#define FRICTION_3650_B_FG_PULSES_PER_REV       6U
#define FRICTION_3650_B_PWM_TIMER_PERIOD        999U
#define FRICTION_3650_B_DUTY_LOW_PERCENT        16U
#define FRICTION_3650_B_DUTY_HIGH_PERCENT       30U
#define FRICTION_3650_B_DIR_DEFAULT             GPIO_PIN_SET

/* RPM report period for conveyor control */
#define BELT_DRIVE_RPM_REPORT_PERIOD_MS      1000U

/* Placeholder speed constants for stepper extension */
#define STEPPER_UART_LOW_RPS                 3U
#define STEPPER_UART_HIGH_RPS                6U

/* 麦轮四路 PWM 定时器参数（TIM3，1 MHz 计数，周期 1000） */
#define MECANUM_PWM_TIMER_PERIOD             999U
#define MECANUM_DEMO_DUTY_PERCENT            99U
#define MECANUM_DEMO_RUN_TIME_MS             3000U
#define MECANUM_DEMO_STOP_TIME_MS            1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint32_t friction_3650_a_fg_pulse_count = 0;
static uint32_t friction_3650_a_last_report_pulse_count = 0;
static uint32_t friction_3650_a_rpm = 0;
static uint32_t friction_3650_a_pwm_duty_percent = 0U;
static GPIO_PinState friction_3650_a_dir_state = FRICTION_3650_A_DIR_DEFAULT;
static uint8_t friction_3650_a_pwm_running = 0U;

static volatile uint32_t friction_3650_b_fg_pulse_count = 0;
static uint32_t friction_3650_b_last_report_pulse_count = 0;
static uint32_t friction_3650_b_rpm = 0;
static uint32_t friction_3650_b_pwm_duty_percent = 0U;
static GPIO_PinState friction_3650_b_dir_state = FRICTION_3650_B_DIR_DEFAULT;
static uint8_t friction_3650_b_pwm_running = 0U;

static uint32_t belt_drive_last_report_tick = 0;
static uint8_t uart1_rx_byte = 0U;
static uint8_t belt_drive_uart_tx_buf[64];
static uint32_t mecanum_demo_last_tick = 0U;
static MecanumDemoStage_t mecanum_demo_stage = MECANUM_DEMO_STAGE_FORWARD;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t Friction3650A_GetPwmCompareFromDuty(uint32_t duty_percent);
static uint32_t Friction3650B_GetPwmCompareFromDuty(uint32_t duty_percent);
static void Friction3650A_StopOutput(void);
static void Friction3650B_StopOutput(void);
static void Friction3650A_SetOutput(GPIO_PinState dir, uint32_t duty_percent);
static void Friction3650B_SetOutput(GPIO_PinState dir, uint32_t duty_percent);
static void BeltDrive_StartUartReceive(void);
static void BeltDrive_ApplyCommand(uint8_t rx_byte);
static void BeltDrive_ReportRpmIfReady(void);
static uint32_t Mecanum_GetPwmCompareFromDuty(uint32_t duty_percent);
static void Mecanum_SetWheelOutput(MecanumWheel_t wheel, MecanumDirection_t direction, uint32_t duty_percent);
static void Mecanum_StopAllWheels(void);
static void Mecanum_RunDemoInMainLoop(void);
static void Stepper_UartControlPlaceholder(uint8_t rx_byte);
static void Friction3650A_UartControlPlaceholder(uint8_t rx_byte);
static void Friction3650BMotor_UartControlPlaceholder(uint8_t rx_byte);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Convert duty percent to TIM2 CH1 compare value for motor A */
static uint32_t Friction3650A_GetPwmCompareFromDuty(uint32_t duty_percent)
{
  uint32_t inverted_duty_percent;

  if (duty_percent >= 100U)
  {
    return 0U;
  }

  inverted_duty_percent = 100U - duty_percent;
  return ((FRICTION_3650_A_PWM_TIMER_PERIOD + 1U) * inverted_duty_percent) / 100U;
}

/* Convert duty percent to TIM4 CH1 compare value for motor B */
static uint32_t Friction3650B_GetPwmCompareFromDuty(uint32_t duty_percent)
{
  uint32_t inverted_duty_percent;

  if (duty_percent >= 100U)
  {
    return 0U;
  }

  inverted_duty_percent = 100U - duty_percent;
  return ((FRICTION_3650_B_PWM_TIMER_PERIOD + 1U) * inverted_duty_percent) / 100U;
}

/* Apply direction and PWM output for 3650-A motor */
static void Friction3650A_StopOutput(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  friction_3650_a_pwm_duty_percent = 0U;
  friction_3650_a_pwm_running = 0U;
}

static void Friction3650A_SetOutput(GPIO_PinState dir, uint32_t duty_percent)
{
  friction_3650_a_dir_state = dir;
  friction_3650_a_pwm_duty_percent = duty_percent;
  HAL_GPIO_WritePin(FRICTION_3650_A_DIR_GPIO_Port, FRICTION_3650_A_DIR_Pin, friction_3650_a_dir_state);

  if (friction_3650_a_pwm_running == 0U)
  {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    friction_3650_a_pwm_running = 1U;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Friction3650A_GetPwmCompareFromDuty(friction_3650_a_pwm_duty_percent));
}

/* Apply direction and PWM output for 3650-B motor */
static void Friction3650B_StopOutput(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  friction_3650_b_pwm_duty_percent = 0U;
  friction_3650_b_pwm_running = 0U;
}

static void Friction3650B_SetOutput(GPIO_PinState dir, uint32_t duty_percent)
{
  friction_3650_b_dir_state = dir;
  friction_3650_b_pwm_duty_percent = duty_percent;
  HAL_GPIO_WritePin(FRICTION_3650_B_DIR_GPIO_Port, FRICTION_3650_B_DIR_Pin, friction_3650_b_dir_state);

  if (friction_3650_b_pwm_running == 0U)
  {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    friction_3650_b_pwm_running = 1U;
  }

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Friction3650B_GetPwmCompareFromDuty(friction_3650_b_pwm_duty_percent));
}

/* Start non-blocking USART1 RX (1 byte) and keep receiver alive */
static void BeltDrive_StartUartReceive(void)
{
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);

  if ((status != HAL_OK) && (status != HAL_BUSY))
  {
    Error_Handler();
  }
}

/*
 * Conveyor combined command mapping:
 *   '0' = stop
 *   '1' = reverse low speed
 *   '2' = reverse high speed
 * Direction of motor A and B is coordinated to match conveyor motion.
 */
static void BeltDrive_ApplyCommand(uint8_t rx_byte)
{
  BeltDriveCommand_t command = BELT_DRIVE_CMD_NONE;
  uint32_t friction_duty = friction_3650_a_pwm_duty_percent;
  uint32_t storage_duty = friction_3650_b_pwm_duty_percent;
  GPIO_PinState friction_dir = friction_3650_a_dir_state;
  GPIO_PinState storage_dir = friction_3650_b_dir_state;

  switch (rx_byte)
  {
    case '0':
      command = BELT_DRIVE_CMD_STOP;
      break;

    case '1':
      command = BELT_DRIVE_CMD_REV_LOW;
      friction_dir = (FRICTION_3650_A_DIR_DEFAULT == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
      storage_dir = FRICTION_3650_B_DIR_DEFAULT;
      friction_duty = FRICTION_3650_A_PWM_DUTY_LOW_PERCENT;
      storage_duty = FRICTION_3650_B_DUTY_LOW_PERCENT;
      break;

    case '2':
      command = BELT_DRIVE_CMD_REV_HIGH;
      friction_dir = (FRICTION_3650_A_DIR_DEFAULT == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
      storage_dir = FRICTION_3650_B_DIR_DEFAULT;
      friction_duty = FRICTION_3650_A_PWM_DUTY_HIGH_PERCENT;
      storage_duty = FRICTION_3650_B_DUTY_HIGH_PERCENT;
      break;

    default:
      break;
  }

  if (command == BELT_DRIVE_CMD_NONE)
  {
    return;
  }

  if (command == BELT_DRIVE_CMD_STOP)
  {
    Friction3650A_StopOutput();
    Friction3650B_StopOutput();
    return;
  }

  Friction3650A_SetOutput(friction_dir, friction_duty);
  Friction3650B_SetOutput(storage_dir, storage_duty);
}

/* Periodically compute RPM from FG pulses and report via USART1 */
static void BeltDrive_ReportRpmIfReady(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint32_t elapsed_ms = now_tick - belt_drive_last_report_tick;
  uint32_t friction_pulse_snapshot;
  uint32_t storage_pulse_snapshot;
  uint32_t friction_pulse_delta;
  uint32_t storage_pulse_delta;
  int msg_len;

  if (elapsed_ms < BELT_DRIVE_RPM_REPORT_PERIOD_MS)
  {
    return;
  }

  __disable_irq();
  friction_pulse_snapshot = friction_3650_a_fg_pulse_count;
  storage_pulse_snapshot = friction_3650_b_fg_pulse_count;
  __enable_irq();

  friction_pulse_delta = friction_pulse_snapshot - friction_3650_a_last_report_pulse_count;
  storage_pulse_delta = storage_pulse_snapshot - friction_3650_b_last_report_pulse_count;
  friction_3650_a_last_report_pulse_count = friction_pulse_snapshot;
  friction_3650_b_last_report_pulse_count = storage_pulse_snapshot;
  belt_drive_last_report_tick = now_tick;

  if (elapsed_ms > 0U)
  {
    friction_3650_a_rpm = (friction_pulse_delta * 60000U) / (FRICTION_3650_A_FG_PULSES_PER_REV * elapsed_ms);
    friction_3650_b_rpm = (storage_pulse_delta * 60000U) / (FRICTION_3650_B_FG_PULSES_PER_REV * elapsed_ms);
  }
  else
  {
    friction_3650_a_rpm = 0U;
    friction_3650_b_rpm = 0U;
  }

  msg_len = snprintf((char *)belt_drive_uart_tx_buf,
                     sizeof(belt_drive_uart_tx_buf),
                     "3650_a_rpm=%lu,3650_b_rpm=%lu\r\n",
                     friction_3650_a_rpm,
                     friction_3650_b_rpm);
  if (msg_len > 0)
  {
    HAL_UART_Transmit(&huart1, belt_drive_uart_tx_buf, (uint16_t)msg_len, 50);
  }
}

/* 将占空比百分比转换成 TIM3 的比较值 */
static uint32_t Mecanum_GetPwmCompareFromDuty(uint32_t duty_percent)
{
  uint32_t compare_value;

  if (duty_percent >= 100U)
  {
    return MECANUM_PWM_TIMER_PERIOD;
  }

  compare_value = ((MECANUM_PWM_TIMER_PERIOD + 1U) * duty_percent) / 100U;
  if (compare_value > MECANUM_PWM_TIMER_PERIOD)
  {
    compare_value = MECANUM_PWM_TIMER_PERIOD;
  }

  return compare_value;
}

/*
 * 麦轮单路控制封装：
 *   正转  -> IN1=1, IN2=0
 *   反转  -> IN1=0, IN2=1
 *   停转  -> IN1=0, IN2=0，同时关闭该路 PWM
 *
 * 当前主循环未调用这个函数，先保留给后续底盘控制逻辑使用。
 */
static void Mecanum_SetWheelOutput(MecanumWheel_t wheel, MecanumDirection_t direction, uint32_t duty_percent)
{
  uint32_t tim_channel = 0U;
  GPIO_TypeDef *in1_port = GPIOE;
  GPIO_TypeDef *in2_port = GPIOE;
  uint16_t in1_pin = 0U;
  uint16_t in2_pin = 0U;

  switch (wheel)
  {
    case MECANUM_WHEEL_A:
      tim_channel = TIM_CHANNEL_1;
      in1_pin = AIN1_Pin;
      in2_pin = AIN2_Pin;
      break;

    case MECANUM_WHEEL_B:
      tim_channel = TIM_CHANNEL_2;
      in1_pin = BIN1_Pin;
      in2_pin = BIN2_Pin;
      break;

    case MECANUM_WHEEL_C:
      tim_channel = TIM_CHANNEL_3;
      in1_pin = CIN1_Pin;
      in2_pin = CIN2_Pin;
      break;

    case MECANUM_WHEEL_D:
      tim_channel = TIM_CHANNEL_4;
      in1_pin = DIN1_Pin;
      in2_pin = DIN2_Pin;
      break;

    default:
      return;
  }

  if ((direction == MECANUM_STOP) || (duty_percent == 0U))
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, tim_channel, 0U);
    HAL_TIM_PWM_Stop(&htim3, tim_channel);
    return;
  }

  if (direction == MECANUM_FORWARD)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
  }

  HAL_TIM_PWM_Start(&htim3, tim_channel);
  __HAL_TIM_SET_COMPARE(&htim3, tim_channel, Mecanum_GetPwmCompareFromDuty(duty_percent));
}

/* 一键关闭四个麦轮通道，作为底盘待机状态 */
static void Mecanum_StopAllWheels(void)
{
  Mecanum_SetWheelOutput(MECANUM_WHEEL_A, MECANUM_STOP, 0U);
  Mecanum_SetWheelOutput(MECANUM_WHEEL_B, MECANUM_STOP, 0U);
  Mecanum_SetWheelOutput(MECANUM_WHEEL_C, MECANUM_STOP, 0U);
  Mecanum_SetWheelOutput(MECANUM_WHEEL_D, MECANUM_STOP, 0U);
}

/*
 * 主循环里的麦轮测试逻辑：
 * 1. 四个轮子同速正转 3 秒
 * 2. 全部停转 1 秒
 * 3. 四个轮子同速反转 3 秒
 * 4. 全部停转 1 秒
 * 然后重复，便于直接观察方向和 PWM 是否正常。
 */
static void Mecanum_RunDemoInMainLoop(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint32_t elapsed_ms = now_tick - mecanum_demo_last_tick;

  switch (mecanum_demo_stage)
  {
    case MECANUM_DEMO_STAGE_FORWARD:
      Mecanum_SetWheelOutput(MECANUM_WHEEL_A, MECANUM_FORWARD, MECANUM_DEMO_DUTY_PERCENT);
      Mecanum_SetWheelOutput(MECANUM_WHEEL_B, MECANUM_FORWARD, MECANUM_DEMO_DUTY_PERCENT);
      Mecanum_SetWheelOutput(MECANUM_WHEEL_C, MECANUM_FORWARD, MECANUM_DEMO_DUTY_PERCENT);
      Mecanum_SetWheelOutput(MECANUM_WHEEL_D, MECANUM_FORWARD, MECANUM_DEMO_DUTY_PERCENT);
      if (elapsed_ms >= MECANUM_DEMO_RUN_TIME_MS)
      {
        mecanum_demo_stage = MECANUM_DEMO_STAGE_STOP_AFTER_FORWARD;
        mecanum_demo_last_tick = now_tick;
        Mecanum_StopAllWheels();
      }
      break;

    case MECANUM_DEMO_STAGE_STOP_AFTER_FORWARD:
      if (elapsed_ms >= MECANUM_DEMO_STOP_TIME_MS)
      {
        mecanum_demo_stage = MECANUM_DEMO_STAGE_REVERSE;
        mecanum_demo_last_tick = now_tick;
      }
      break;

    case MECANUM_DEMO_STAGE_REVERSE:
      Mecanum_SetWheelOutput(MECANUM_WHEEL_A, MECANUM_REVERSE, MECANUM_DEMO_DUTY_PERCENT);
      Mecanum_SetWheelOutput(MECANUM_WHEEL_B, MECANUM_REVERSE, MECANUM_DEMO_DUTY_PERCENT);
      Mecanum_SetWheelOutput(MECANUM_WHEEL_C, MECANUM_REVERSE, MECANUM_DEMO_DUTY_PERCENT);
      Mecanum_SetWheelOutput(MECANUM_WHEEL_D, MECANUM_REVERSE, MECANUM_DEMO_DUTY_PERCENT);
      if (elapsed_ms >= MECANUM_DEMO_RUN_TIME_MS)
      {
        mecanum_demo_stage = MECANUM_DEMO_STAGE_STOP_AFTER_REVERSE;
        mecanum_demo_last_tick = now_tick;
        Mecanum_StopAllWheels();
      }
      break;

    case MECANUM_DEMO_STAGE_STOP_AFTER_REVERSE:
    default:
      if (elapsed_ms >= MECANUM_DEMO_STOP_TIME_MS)
      {
        mecanum_demo_stage = MECANUM_DEMO_STAGE_FORWARD;
        mecanum_demo_last_tick = now_tick;
      }
      break;
  }
}

/* Stepper command placeholder for future implementation */
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
   * TODO: Replace this placeholder with real stepper driver calls.
   * Suggested interface:
   *   Stepper_SetDirectionAndSpeed(dir, target_rps);
   */
  (void)dir;
  (void)target_rps;
}

/* 3650-A single motor UART command placeholder */
static void Friction3650A_UartControlPlaceholder(uint8_t rx_byte)
{
  Friction3650ACommand_t command = FRICTION_3650_A_CMD_NONE;
  GPIO_PinState dir = FRICTION_3650_A_DIR_DEFAULT;
  uint32_t duty_percent = friction_3650_a_pwm_duty_percent;

  switch (rx_byte)
  {
    case 'A':
      command = FRICTION_3650_A_CMD_FWD_LOW;
      dir = GPIO_PIN_SET;
      duty_percent = FRICTION_3650_A_PWM_DUTY_LOW_PERCENT;
      break;

    case 'B':
      command = FRICTION_3650_A_CMD_FWD_HIGH;
      dir = GPIO_PIN_SET;
      duty_percent = FRICTION_3650_A_PWM_DUTY_HIGH_PERCENT;
      break;

    case 'C':
      command = FRICTION_3650_A_CMD_REV_LOW;
      dir = GPIO_PIN_RESET;
      duty_percent = FRICTION_3650_A_PWM_DUTY_LOW_PERCENT;
      break;

    case 'D':
      command = FRICTION_3650_A_CMD_REV_HIGH;
      dir = GPIO_PIN_RESET;
      duty_percent = FRICTION_3650_A_PWM_DUTY_HIGH_PERCENT;
      break;

    default:
      break;
  }

  if (command == FRICTION_3650_A_CMD_NONE)
  {
    return;
  }

  Friction3650A_SetOutput(dir, duty_percent);
}

/* 3650-B single motor UART command placeholder */
static void Friction3650BMotor_UartControlPlaceholder(uint8_t rx_byte)
{
  Friction3650BMotorCommand_t command = FRICTION_3650_B_MOTOR_CMD_NONE;
  GPIO_PinState dir = friction_3650_b_dir_state;
  uint32_t duty_percent = friction_3650_b_pwm_duty_percent;

  switch (rx_byte)
  {
    case 'E':
      command = FRICTION_3650_B_MOTOR_CMD_FWD_LOW;
      dir = GPIO_PIN_SET;
      duty_percent = FRICTION_3650_B_DUTY_LOW_PERCENT;
      break;

    case 'F':
      command = FRICTION_3650_B_MOTOR_CMD_FWD_HIGH;
      dir = GPIO_PIN_SET;
      duty_percent = FRICTION_3650_B_DUTY_HIGH_PERCENT;
      break;

    case 'G':
      command = FRICTION_3650_B_MOTOR_CMD_REV_LOW;
      dir = GPIO_PIN_RESET;
      duty_percent = FRICTION_3650_B_DUTY_LOW_PERCENT;
      break;

    case 'H':
      command = FRICTION_3650_B_MOTOR_CMD_REV_HIGH;
      dir = GPIO_PIN_RESET;
      duty_percent = FRICTION_3650_B_DUTY_HIGH_PERCENT;
      break;

    default:
      break;
  }

  if (command == FRICTION_3650_B_MOTOR_CMD_NONE)
  {
    return;
  }

  Friction3650B_SetOutput(dir, duty_percent);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  belt_drive_last_report_tick = HAL_GetTick();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  friction_3650_a_pwm_running = 1U;
  friction_3650_b_pwm_running = 1U;

  /* 初始化两路 3650 电机为默认方向、0% 占空比 */
  HAL_GPIO_WritePin(FRICTION_3650_A_DIR_GPIO_Port, FRICTION_3650_A_DIR_Pin, FRICTION_3650_A_DIR_DEFAULT);
  HAL_GPIO_WritePin(FRICTION_3650_B_DIR_GPIO_Port, FRICTION_3650_B_DIR_Pin, FRICTION_3650_B_DIR_DEFAULT);
  Friction3650A_StopOutput();
  Friction3650B_StopOutput();

  /* 麦轮测试逻辑已接入主循环，先从正转阶段开始。 */
  mecanum_demo_stage = MECANUM_DEMO_STAGE_FORWARD;
  mecanum_demo_last_tick = HAL_GetTick();
  Mecanum_StopAllWheels();

  BeltDrive_StartUartReceive();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Mecanum_RunDemoInMainLoop();
    BeltDrive_ReportRpmIfReady();
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
  if (GPIO_Pin == FRICTION_3650_A_FG_Pin)
  {
    friction_3650_a_fg_pulse_count++;
  }
  else if (GPIO_Pin == FRICTION_3650_B_FG_Pin)
  {
    friction_3650_b_fg_pulse_count++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    BeltDrive_ApplyCommand(uart1_rx_byte);
    BeltDrive_StartUartReceive();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    BeltDrive_StartUartReceive();
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




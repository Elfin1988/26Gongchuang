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
  FRICTION_MOTOR_CMD_NONE = 0,
  FRICTION_MOTOR_CMD_FWD_LOW,
  FRICTION_MOTOR_CMD_FWD_HIGH,
  FRICTION_MOTOR_CMD_REV_LOW,
  FRICTION_MOTOR_CMD_REV_HIGH
} FrictionMotorCommand_t;

typedef enum
{
  BELT_DRIVE_CMD_NONE = 0,
  BELT_DRIVE_CMD_STOP,
  BELT_DRIVE_CMD_REV_LOW,
  BELT_DRIVE_CMD_REV_HIGH
} BeltDriveCommand_t;

typedef enum
{
  CHASSIS_WHEEL_FRONT_LEFT = 0,
  CHASSIS_WHEEL_FRONT_RIGHT,
  CHASSIS_WHEEL_REAR_LEFT,
  CHASSIS_WHEEL_REAR_RIGHT,
  CHASSIS_WHEEL_COUNT
} ChassisWheel_t;

typedef enum
{
  CHASSIS_STOP = 0,
  CHASSIS_FORWARD,
  CHASSIS_REVERSE
} ChassisDirection_t;

typedef enum
{
  CHASSIS_DEMO_STAGE_FORWARD = 0,
  CHASSIS_DEMO_STAGE_STOP_AFTER_FORWARD,
  CHASSIS_DEMO_STAGE_REVERSE,
  CHASSIS_DEMO_STAGE_STOP_AFTER_REVERSE
} ChassisDemoStage_t;

typedef struct
{
  TIM_HandleTypeDef *tim;
  uint32_t tim_channel;
  uint32_t pwm_timer_period;
  uint32_t fg_pulses_per_rev;
  GPIO_TypeDef *dir_gpio_port;
  uint16_t dir_gpio_pin;
  GPIO_PinState dir_default;
  volatile uint32_t fg_pulse_count;
  uint32_t last_report_pulse_count;
  uint32_t rpm;
  uint32_t pwm_duty_percent;
  GPIO_PinState dir_state;
  uint8_t pwm_running;
} FrictionMotor_t;

typedef FrictionMotor_t ChassisMotor_t;

typedef enum
{
  STEPPER_OUTPUT_SOFTWARE_GPIO = 0,
  STEPPER_OUTPUT_TIM1_CH1,
  STEPPER_OUTPUT_TIM1_CH2
} StepperOutputMode_t;

typedef enum
{
  STEPPER_DEBUG_PHASE_FORWARD_RUN = 0,
  STEPPER_DEBUG_PHASE_FORWARD_DECEL,
  STEPPER_DEBUG_PHASE_REVERSE_RUN,
  STEPPER_DEBUG_PHASE_REVERSE_DECEL
} StepperDebugPhase_t;

typedef enum
{
  STEPPER_CONTROL_MODE_AUTO_DEBUG = 0,
  STEPPER_CONTROL_MODE_MANUAL
} StepperControlMode_t;

typedef struct
{
  StepperOutputMode_t output_mode;
  GPIO_TypeDef *dir_gpio_port;
  uint16_t dir_gpio_pin;
  GPIO_PinState forward_dir_state;
  float max_speed_sps;
  float accel_sps2;
  volatile float current_speed_sps;
  volatile float target_speed_sps;
  GPIO_TypeDef *step_gpio_port;
  uint16_t step_gpio_pin;
  uint32_t tim_channel;
  volatile uint32_t tim_toggle_interval_ticks;
  volatile uint8_t tim_output_running;
  uint32_t last_cycle_counter;
  float software_step_accumulator;
  uint8_t software_pulse_high;
} StepperAxis_t;

typedef struct
{
  TIM_HandleTypeDef *tim;
  uint32_t tim_channel;
  volatile uint32_t target_pulse_width_us;
  volatile int32_t target_angle_deg;
  uint8_t pwm_running;
} HardwareServo_t;
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

/* Chassis 3650 motors use the same active-low PWM driver as the belt motors. */
#define CHASSIS_3650_FG_PULSES_PER_REV       6U
#define CHASSIS_3650_PWM_TIMER_PERIOD        999U
#define CHASSIS_3650_DEMO_DUTY_PERCENT       30U
#define CHASSIS_3650_DEMO_RUN_TIME_MS        3000U
#define CHASSIS_3650_DEMO_STOP_TIME_MS       1000U
#define CHASSIS_3650_DIR_DEFAULT             GPIO_PIN_SET
#define STEPPER_DEBUG_FORWARD_RUN_MS         2000U
#define STEPPER_DEBUG_CONTROL_PERIOD_MS      5U
#define STEPPER_AXIS_COUNT                   2U
#define STEPPER_Y_MAX_SPEED_SPS              680.0f
#define STEPPER_Z_MAX_SPEED_SPS              2200.0f
#define STEPPER_Y_ACCEL_SPS2                 260.0f
#define STEPPER_Z_ACCEL_SPS2                 3000.0f
#define STEPPER_TIM_STOP_THRESHOLD_SPS       1.0f
#define STEPPER_DEBUG_TIM1_SLOT_HZ           10000U
#define STEPPER_TIM_PWM_PULSE_WIDTH_US       4U
#define STEPPER_UART_SINGLE_AXIS_SCALE       0.45f
#define STEPPER_UART_ALL_AXES_SCALE          0.30f
#define UART1_RX_CMD_QUEUE_SIZE              16U
#define SERVO_PWM_FRAME_US                   20000U
#define SERVO_PWM_MIN_PULSE_US               500U
#define SERVO_PWM_CENTER_PULSE_US            1500U
#define SERVO_PWM_MAX_PULSE_US               2500U
#define SERVO_MAX_ANGLE_DEG                  135
#define SERVO_DEFAULT_ANGLE_DEG              0
#define SERVO_UART_POSITIVE_ANGLE_DEG        135
#define SERVO_UART_NEGATIVE_ANGLE_DEG        (-135)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static FrictionMotor_t friction_3650_a =
{
  &htim24,
  TIM_CHANNEL_1,
  FRICTION_3650_A_PWM_TIMER_PERIOD,
  FRICTION_3650_A_FG_PULSES_PER_REV,
  FRICTION_3650_A_DIR_GPIO_Port,
  FRICTION_3650_A_DIR_Pin,
  FRICTION_3650_A_DIR_DEFAULT,
  0U,
  0U,
  0U,
  0U,
  FRICTION_3650_A_DIR_DEFAULT,
  0U
};

static FrictionMotor_t friction_3650_b =
{
  &htim24,
  TIM_CHANNEL_2,
  FRICTION_3650_B_PWM_TIMER_PERIOD,
  FRICTION_3650_B_FG_PULSES_PER_REV,
  FRICTION_3650_B_DIR_GPIO_Port,
  FRICTION_3650_B_DIR_Pin,
  FRICTION_3650_B_DIR_DEFAULT,
  0U,
  0U,
  0U,
  0U,
  FRICTION_3650_B_DIR_DEFAULT,
  0U
};

static uint32_t belt_drive_last_report_tick = 0;
static uint8_t uart1_rx_byte = 0U;
static uint8_t belt_drive_uart_tx_buf[64];
static uint8_t uart1_ack_tx_buf[32];
static volatile uint8_t uart1_ack_pending = 0U;
static volatile uint8_t uart1_ack_byte = 0U;
static volatile uint8_t uart1_rx_cmd_queue[UART1_RX_CMD_QUEUE_SIZE];
static volatile uint8_t uart1_rx_cmd_read_index = 0U;
static volatile uint8_t uart1_rx_cmd_write_index = 0U;
static volatile uint8_t uart1_rx_cmd_count = 0U;
static uint32_t chassis_demo_last_tick = 0U;
static ChassisDemoStage_t chassis_demo_stage = CHASSIS_DEMO_STAGE_FORWARD;
static ChassisMotor_t chassis_3650_motors[CHASSIS_WHEEL_COUNT] =
{
  {&htim3, TIM_CHANNEL_1, CHASSIS_3650_PWM_TIMER_PERIOD, CHASSIS_3650_FG_PULSES_PER_REV,
   CHASSIS_3650_FL_DIR_GPIO_Port, CHASSIS_3650_FL_DIR_Pin, CHASSIS_3650_DIR_DEFAULT,
   0U, 0U, 0U, 0U, CHASSIS_3650_DIR_DEFAULT, 0U},
  {&htim2, TIM_CHANNEL_4, CHASSIS_3650_PWM_TIMER_PERIOD, CHASSIS_3650_FG_PULSES_PER_REV,
   CHASSIS_3650_FR_DIR_GPIO_Port, CHASSIS_3650_FR_DIR_Pin, CHASSIS_3650_DIR_DEFAULT,
   0U, 0U, 0U, 0U, CHASSIS_3650_DIR_DEFAULT, 0U},
  {&htim4, TIM_CHANNEL_3, CHASSIS_3650_PWM_TIMER_PERIOD, CHASSIS_3650_FG_PULSES_PER_REV,
   CHASSIS_3650_RL_DIR_GPIO_Port, CHASSIS_3650_RL_DIR_Pin, CHASSIS_3650_DIR_DEFAULT,
   0U, 0U, 0U, 0U, CHASSIS_3650_DIR_DEFAULT, 0U},
  {&htim4, TIM_CHANNEL_4, CHASSIS_3650_PWM_TIMER_PERIOD, CHASSIS_3650_FG_PULSES_PER_REV,
   CHASSIS_3650_RR_DIR_GPIO_Port, CHASSIS_3650_RR_DIR_Pin, CHASSIS_3650_DIR_DEFAULT,
   0U, 0U, 0U, 0U, CHASSIS_3650_DIR_DEFAULT, 0U}
};
static StepperAxis_t stepper_axis_y =
{
  STEPPER_OUTPUT_SOFTWARE_GPIO,
  Y_DIR_GPIO_Port,
  Y_DIR_Pin,
  GPIO_PIN_SET,
  STEPPER_Y_MAX_SPEED_SPS,
  STEPPER_Y_ACCEL_SPS2,
  0.0f,
  0.0f,
  Y_STEP_GPIO_Port,
  Y_STEP_Pin,
  TIM_CHANNEL_2,
  0U,
  0U,
  0U,
  0.0f,
  0U
};
static StepperAxis_t stepper_axis_z =
{
  STEPPER_OUTPUT_SOFTWARE_GPIO,
  Z_DIR_GPIO_Port,
  Z_DIR_Pin,
  GPIO_PIN_SET,
  STEPPER_Z_MAX_SPEED_SPS,
  STEPPER_Z_ACCEL_SPS2,
  0.0f,
  0.0f,
  Z_STEP_GPIO_Port,
  Z_STEP_Pin,
  TIM_CHANNEL_1,
  0U,
  0U,
  0U,
  0.0f,
  0U
};
static StepperAxis_t * const stepper_axes[STEPPER_AXIS_COUNT] =
{
  &stepper_axis_y,
  &stepper_axis_z
};
static volatile StepperControlMode_t stepper_control_mode = STEPPER_CONTROL_MODE_AUTO_DEBUG;
static volatile StepperDebugPhase_t stepper_debug_phase = STEPPER_DEBUG_PHASE_FORWARD_RUN;
static volatile uint32_t stepper_debug_phase_tick = 0U;
static volatile uint32_t stepper_debug_last_control_tick = 0U;
static uint32_t stepper_tim_pwm_pulse_ticks = 1U;
static HardwareServo_t servo_elbow =
{
  &htim15,
  TIM_CHANNEL_1,
  SERVO_PWM_CENTER_PULSE_US,
  SERVO_DEFAULT_ANGLE_DEG,
  0U
};
static HardwareServo_t servo_block =
{
  &htim13,
  TIM_CHANNEL_1,
  SERVO_PWM_CENTER_PULSE_US,
  SERVO_DEFAULT_ANGLE_DEG,
  0U
};
static HardwareServo_t servo_gripper =
{
  &htim8,
  TIM_CHANNEL_2,
  SERVO_PWM_CENTER_PULSE_US,
  SERVO_DEFAULT_ANGLE_DEG,
  0U
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t FrictionMotor_GetPwmCompareFromDuty(const FrictionMotor_t *motor, uint32_t duty_percent);
static void FrictionMotor_StopOutput(FrictionMotor_t *motor);
static void FrictionMotor_SetOutput(FrictionMotor_t *motor, GPIO_PinState dir, uint32_t duty_percent);
static void FrictionMotor_InitStopped(FrictionMotor_t *motor);
static GPIO_PinState FrictionMotor_GetReverseDirection(const FrictionMotor_t *motor);
static void BeltDrive_StartUartReceive(void);
static void BeltDrive_ApplyCommand(uint8_t rx_byte);
static void BeltDrive_ReportRpmIfReady(void);
static void Chassis3650_ApplyToAllWheels(ChassisDirection_t direction, uint32_t duty_percent);
static void Chassis3650_SetWheelOutput(ChassisWheel_t wheel, ChassisDirection_t direction, uint32_t duty_percent);
static void Chassis3650_StopAllWheels(void);
static void Chassis3650_RunDemoInMainLoop(void);
static void StepperDebug_Init(void);
static void StepperDebug_Run(void);
static uint32_t StepperDebug_GetTim1CounterClockHz(void);
static void StepperDebug_ForceTim1ChannelLow(uint32_t tim_channel);
static void StepperDebug_SetAxisStepLow(StepperAxis_t *axis);
static void StepperDebug_StartTim1Axis(StepperAxis_t *axis);
static void StepperDebug_SetAxisDirection(StepperAxis_t *axis, uint8_t forward);
static float StepperDebug_LimitAxisSpeed(const StepperAxis_t *axis, float speed_sps);
static void StepperDebug_EnterManualMode(void);
static void StepperDebug_EnterAutoMode(void);
static void StepperDebug_CommandAxis(StepperAxis_t *axis, uint8_t forward, float speed_sps);
static void StepperDebug_CommandAllAxes(uint8_t forward, float speed_scale);
static void StepperDebug_RequestStopAllAxes(void);
static void StepperDebug_SetTargetsToCurrentPhase(void);
static void StepperDebug_UpdatePhase(void);
static void StepperDebug_UpdateAxisSpeeds(float delta_s);
static void StepperDebug_ApplyAxisSpeed(StepperAxis_t *axis);
static uint8_t StepperDebug_AllAxesStopped(void);
static void StepperDebug_ServiceTim1Axes(float delta_s);
static void Stepper_RequestStopYAndZAxes(void);
static void Stepper_ProcessUartCommand(uint8_t rx_byte);
static void Friction3650A_UartControlPlaceholder(uint8_t rx_byte);
static void Friction3650BMotor_UartControlPlaceholder(uint8_t rx_byte);
static uint32_t Servo_ClampPulseWidthUs(uint32_t pulse_width_us);
static int32_t Servo_ClampAngleDeg(int32_t angle_deg);
static uint32_t Servo_AngleToPulseWidthUs(int32_t angle_deg);
static void Servo_Init(HardwareServo_t *servo);
static void Servo_SetAngleDegrees(HardwareServo_t *servo, int32_t angle_deg);
static void Servo_UartControlPlaceholder(uint8_t rx_byte);
static void Uart1_EnqueueRxByte(uint8_t rx_byte);
static uint8_t Uart1_TryDequeueRxByte(uint8_t *rx_byte);
static void Uart1_ProcessPendingRxCommands(void);
static void Uart1_QueueAck(uint8_t rx_byte);
static void Uart1_SendPendingAck(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Convert duty percent to compare value for active-low PWM outputs */
static uint32_t FrictionMotor_GetPwmCompareFromDuty(const FrictionMotor_t *motor, uint32_t duty_percent)
{
  uint32_t inverted_duty_percent;

  if (duty_percent >= 100U)
  {
    return 0U;
  }

  inverted_duty_percent = 100U - duty_percent;
  return ((motor->pwm_timer_period + 1U) * inverted_duty_percent) / 100U;
}

static void FrictionMotor_StopOutput(FrictionMotor_t *motor)
{
  __HAL_TIM_SET_COMPARE(motor->tim, motor->tim_channel, 0U);
  if (motor->pwm_running != 0U)
  {
    if (HAL_TIM_PWM_Stop(motor->tim, motor->tim_channel) != HAL_OK)
    {
      Error_Handler();
    }
  }

  motor->pwm_duty_percent = 0U;
  motor->pwm_running = 0U;
}

static void FrictionMotor_SetOutput(FrictionMotor_t *motor, GPIO_PinState dir, uint32_t duty_percent)
{
  motor->dir_state = dir;
  motor->pwm_duty_percent = duty_percent;
  HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_gpio_pin, motor->dir_state);

  if (motor->pwm_running == 0U)
  {
    if (HAL_TIM_PWM_Start(motor->tim, motor->tim_channel) != HAL_OK)
    {
      Error_Handler();
    }
    motor->pwm_running = 1U;
  }

  __HAL_TIM_SET_COMPARE(motor->tim,
                        motor->tim_channel,
                        FrictionMotor_GetPwmCompareFromDuty(motor, motor->pwm_duty_percent));
}

static void FrictionMotor_InitStopped(FrictionMotor_t *motor)
{
  HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_gpio_pin, motor->dir_default);
  motor->dir_state = motor->dir_default;
  FrictionMotor_StopOutput(motor);
}

static GPIO_PinState FrictionMotor_GetReverseDirection(const FrictionMotor_t *motor)
{
  return (motor->dir_default == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
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
  uint32_t friction_duty = friction_3650_a.pwm_duty_percent;
  uint32_t storage_duty = friction_3650_b.pwm_duty_percent;
  GPIO_PinState friction_dir = friction_3650_a.dir_state;
  GPIO_PinState storage_dir = friction_3650_b.dir_state;

  switch (rx_byte)
  {
    case '0':
      command = BELT_DRIVE_CMD_STOP;
      break;

    case '1':
      command = BELT_DRIVE_CMD_REV_LOW;
      friction_dir = FrictionMotor_GetReverseDirection(&friction_3650_a);
      storage_dir = friction_3650_b.dir_default;
      friction_duty = FRICTION_3650_A_PWM_DUTY_LOW_PERCENT;
      storage_duty = FRICTION_3650_B_DUTY_LOW_PERCENT;
      break;

    case '2':
      command = BELT_DRIVE_CMD_REV_HIGH;
      friction_dir = FrictionMotor_GetReverseDirection(&friction_3650_a);
      storage_dir = friction_3650_b.dir_default;
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
    FrictionMotor_StopOutput(&friction_3650_a);
    FrictionMotor_StopOutput(&friction_3650_b);
    return;
  }

  FrictionMotor_SetOutput(&friction_3650_a, friction_dir, friction_duty);
  FrictionMotor_SetOutput(&friction_3650_b, storage_dir, storage_duty);
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
  friction_pulse_snapshot = friction_3650_a.fg_pulse_count;
  storage_pulse_snapshot = friction_3650_b.fg_pulse_count;
  __enable_irq();

  friction_pulse_delta = friction_pulse_snapshot - friction_3650_a.last_report_pulse_count;
  storage_pulse_delta = storage_pulse_snapshot - friction_3650_b.last_report_pulse_count;
  friction_3650_a.last_report_pulse_count = friction_pulse_snapshot;
  friction_3650_b.last_report_pulse_count = storage_pulse_snapshot;
  belt_drive_last_report_tick = now_tick;

  if (elapsed_ms > 0U)
  {
    friction_3650_a.rpm = (friction_pulse_delta * 60000U) / (friction_3650_a.fg_pulses_per_rev * elapsed_ms);
    friction_3650_b.rpm = (storage_pulse_delta * 60000U) / (friction_3650_b.fg_pulses_per_rev * elapsed_ms);
  }
  else
  {
    friction_3650_a.rpm = 0U;
    friction_3650_b.rpm = 0U;
  }

  msg_len = snprintf((char *)belt_drive_uart_tx_buf,
                     sizeof(belt_drive_uart_tx_buf),
                     "3650_a_rpm=%lu,3650_b_rpm=%lu\r\n",
                     friction_3650_a.rpm,
                     friction_3650_b.rpm);
  if (msg_len > 0)
  {
    HAL_UART_Transmit(&huart1, belt_drive_uart_tx_buf, (uint16_t)msg_len, 50);
  }
}

static void Chassis3650_ApplyToAllWheels(ChassisDirection_t direction, uint32_t duty_percent)
{
  uint32_t wheel_index;

  for (wheel_index = 0U; wheel_index < (uint32_t)CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    Chassis3650_SetWheelOutput((ChassisWheel_t)wheel_index, direction, duty_percent);
  }
}

static void Chassis3650_SetWheelOutput(ChassisWheel_t wheel, ChassisDirection_t direction, uint32_t duty_percent)
{
  ChassisMotor_t *motor;
  GPIO_PinState dir;

  if ((uint32_t)wheel >= (uint32_t)CHASSIS_WHEEL_COUNT)
  {
    return;
  }

  motor = &chassis_3650_motors[(uint32_t)wheel];

  if ((direction == CHASSIS_STOP) || (duty_percent == 0U))
  {
    FrictionMotor_StopOutput(motor);
    return;
  }

  dir = (direction == CHASSIS_FORWARD) ? motor->dir_default : FrictionMotor_GetReverseDirection(motor);
  FrictionMotor_SetOutput(motor, dir, duty_percent);
}

static void Chassis3650_StopAllWheels(void)
{
  Chassis3650_ApplyToAllWheels(CHASSIS_STOP, 0U);
}

static void Chassis3650_RunDemoInMainLoop(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint32_t elapsed_ms = now_tick - chassis_demo_last_tick;

  switch (chassis_demo_stage)
  {
    case CHASSIS_DEMO_STAGE_FORWARD:
      Chassis3650_ApplyToAllWheels(CHASSIS_FORWARD, CHASSIS_3650_DEMO_DUTY_PERCENT);
      if (elapsed_ms >= CHASSIS_3650_DEMO_RUN_TIME_MS)
      {
        chassis_demo_stage = CHASSIS_DEMO_STAGE_STOP_AFTER_FORWARD;
        chassis_demo_last_tick = now_tick;
        Chassis3650_StopAllWheels();
      }
      break;

    case CHASSIS_DEMO_STAGE_STOP_AFTER_FORWARD:
      if (elapsed_ms >= CHASSIS_3650_DEMO_STOP_TIME_MS)
      {
        chassis_demo_stage = CHASSIS_DEMO_STAGE_REVERSE;
        chassis_demo_last_tick = now_tick;
      }
      break;

    case CHASSIS_DEMO_STAGE_REVERSE:
      Chassis3650_ApplyToAllWheels(CHASSIS_REVERSE, CHASSIS_3650_DEMO_DUTY_PERCENT);
      if (elapsed_ms >= CHASSIS_3650_DEMO_RUN_TIME_MS)
      {
        chassis_demo_stage = CHASSIS_DEMO_STAGE_STOP_AFTER_REVERSE;
        chassis_demo_last_tick = now_tick;
        Chassis3650_StopAllWheels();
      }
      break;

    case CHASSIS_DEMO_STAGE_STOP_AFTER_REVERSE:
    default:
      if (elapsed_ms >= CHASSIS_3650_DEMO_STOP_TIME_MS)
      {
        chassis_demo_stage = CHASSIS_DEMO_STAGE_FORWARD;
        chassis_demo_last_tick = now_tick;
      }
      break;
  }
}

static uint32_t StepperDebug_GetTim1CounterClockHz(void)
{
  RCC_ClkInitTypeDef clk_config;
  uint32_t flash_latency;
  uint32_t pclk_hz;

  HAL_RCC_GetClockConfig(&clk_config, &flash_latency);
  pclk_hz = HAL_RCC_GetPCLK2Freq();
  if (clk_config.APB2CLKDivider != RCC_HCLK_DIV1)
  {
    pclk_hz *= 2U;
  }

  return pclk_hz / (htim1.Init.Prescaler + 1U);
}

static void StepperDebug_ForceTim1ChannelLow(uint32_t tim_channel)
{
  if (tim_channel == TIM_CHANNEL_1)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  }
}

static void StepperDebug_SetAxisStepLow(StepperAxis_t *axis)
{
  if (axis->output_mode == STEPPER_OUTPUT_SOFTWARE_GPIO)
  {
    HAL_GPIO_WritePin(axis->step_gpio_port, axis->step_gpio_pin, GPIO_PIN_RESET);
  }
  else
  {
    StepperDebug_ForceTim1ChannelLow(axis->tim_channel);
  }
}

static void StepperDebug_StartTim1Axis(StepperAxis_t *axis)
{
  if (axis->output_mode == STEPPER_OUTPUT_SOFTWARE_GPIO)
  {
    axis->tim_output_running = 1U;
    StepperDebug_SetAxisStepLow(axis);
    return;
  }

  if (axis->tim_output_running != 0U)
  {
    return;
  }

  if (HAL_TIM_PWM_Start(&htim1, axis->tim_channel) != HAL_OK)
  {
    Error_Handler();
  }

  axis->tim_output_running = 1U;
  StepperDebug_ForceTim1ChannelLow(axis->tim_channel);
}

static void StepperDebug_SetAxisDirection(StepperAxis_t *axis, uint8_t forward)
{
  GPIO_PinState dir_state = axis->forward_dir_state;

  if (forward == 0U)
  {
    dir_state = (axis->forward_dir_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(axis->dir_gpio_port, axis->dir_gpio_pin, dir_state);
}

static float StepperDebug_LimitAxisSpeed(const StepperAxis_t *axis, float speed_sps)
{
  if (speed_sps < 0.0f)
  {
    return 0.0f;
  }

  if (speed_sps > axis->max_speed_sps)
  {
    return axis->max_speed_sps;
  }

  return speed_sps;
}

static void StepperDebug_EnterManualMode(void)
{
  stepper_control_mode = STEPPER_CONTROL_MODE_MANUAL;
  stepper_debug_phase_tick = HAL_GetTick();
  stepper_debug_last_control_tick = stepper_debug_phase_tick;
}

static void StepperDebug_EnterAutoMode(void)
{
  uint32_t axis_index;

  stepper_control_mode = STEPPER_CONTROL_MODE_AUTO_DEBUG;
  stepper_debug_phase = STEPPER_DEBUG_PHASE_FORWARD_RUN;
  stepper_debug_phase_tick = HAL_GetTick();
  stepper_debug_last_control_tick = stepper_debug_phase_tick;

  for (axis_index = 0U; axis_index < STEPPER_AXIS_COUNT; axis_index++)
  {
    stepper_axes[axis_index]->target_speed_sps = 0.0f;
  }

  StepperDebug_SetTargetsToCurrentPhase();
}

static void StepperDebug_CommandAxis(StepperAxis_t *axis, uint8_t forward, float speed_sps)
{
  StepperDebug_EnterManualMode();
  StepperDebug_SetAxisDirection(axis, forward);
  axis->target_speed_sps = StepperDebug_LimitAxisSpeed(axis, speed_sps);
}

static void StepperDebug_CommandAllAxes(uint8_t forward, float speed_scale)
{
  uint32_t axis_index;

  StepperDebug_EnterManualMode();
  for (axis_index = 0U; axis_index < STEPPER_AXIS_COUNT; axis_index++)
  {
    StepperAxis_t *axis = stepper_axes[axis_index];
    StepperDebug_SetAxisDirection(axis, forward);
    axis->target_speed_sps = StepperDebug_LimitAxisSpeed(axis, axis->max_speed_sps * speed_scale);
  }
}

static void StepperDebug_RequestStopAllAxes(void)
{
  uint32_t axis_index;

  StepperDebug_EnterManualMode();
  for (axis_index = 0U; axis_index < STEPPER_AXIS_COUNT; axis_index++)
  {
    stepper_axes[axis_index]->target_speed_sps = 0.0f;
  }
}

static void StepperDebug_SetTargetsToCurrentPhase(void)
{
  uint32_t axis_index;
  uint8_t forward = ((stepper_debug_phase == STEPPER_DEBUG_PHASE_FORWARD_RUN) ||
                     (stepper_debug_phase == STEPPER_DEBUG_PHASE_FORWARD_DECEL)) ? 1U : 0U;

  for (axis_index = 0U; axis_index < STEPPER_AXIS_COUNT; axis_index++)
  {
    StepperAxis_t *axis = stepper_axes[axis_index];
    StepperDebug_SetAxisDirection(axis, forward);

    if ((stepper_debug_phase == STEPPER_DEBUG_PHASE_FORWARD_RUN) ||
        (stepper_debug_phase == STEPPER_DEBUG_PHASE_REVERSE_RUN))
    {
      axis->target_speed_sps = axis->max_speed_sps;
    }
    else
    {
      axis->target_speed_sps = 0.0f;
    }
  }
}

static void StepperDebug_UpdatePhase(void)
{
  uint32_t now_tick = HAL_GetTick();

  switch (stepper_debug_phase)
  {
    case STEPPER_DEBUG_PHASE_FORWARD_RUN:
      if ((now_tick - stepper_debug_phase_tick) >= STEPPER_DEBUG_FORWARD_RUN_MS)
      {
        stepper_debug_phase = STEPPER_DEBUG_PHASE_FORWARD_DECEL;
        stepper_debug_phase_tick = now_tick;
        StepperDebug_SetTargetsToCurrentPhase();
      }
      break;

    case STEPPER_DEBUG_PHASE_FORWARD_DECEL:
      if (StepperDebug_AllAxesStopped() != 0U)
      {
        stepper_debug_phase = STEPPER_DEBUG_PHASE_REVERSE_RUN;
        stepper_debug_phase_tick = now_tick;
        StepperDebug_SetTargetsToCurrentPhase();
      }
      break;

    case STEPPER_DEBUG_PHASE_REVERSE_RUN:
      if ((now_tick - stepper_debug_phase_tick) >= STEPPER_DEBUG_FORWARD_RUN_MS)
      {
        stepper_debug_phase = STEPPER_DEBUG_PHASE_REVERSE_DECEL;
        stepper_debug_phase_tick = now_tick;
        StepperDebug_SetTargetsToCurrentPhase();
      }
      break;

    case STEPPER_DEBUG_PHASE_REVERSE_DECEL:
    default:
      if (StepperDebug_AllAxesStopped() != 0U)
      {
        stepper_debug_phase = STEPPER_DEBUG_PHASE_FORWARD_RUN;
        stepper_debug_phase_tick = now_tick;
        StepperDebug_SetTargetsToCurrentPhase();
      }
      break;
  }
}

static void StepperDebug_UpdateAxisSpeeds(float delta_s)
{
  uint32_t axis_index;

  for (axis_index = 0U; axis_index < STEPPER_AXIS_COUNT; axis_index++)
  {
    StepperAxis_t *axis = stepper_axes[axis_index];
    float speed_step = axis->accel_sps2 * delta_s;

    if (axis->current_speed_sps < axis->target_speed_sps)
    {
      axis->current_speed_sps += speed_step;
      if (axis->current_speed_sps > axis->target_speed_sps)
      {
        axis->current_speed_sps = axis->target_speed_sps;
      }
    }
    else if (axis->current_speed_sps > axis->target_speed_sps)
    {
      axis->current_speed_sps -= speed_step;
      if (axis->current_speed_sps < axis->target_speed_sps)
      {
        axis->current_speed_sps = axis->target_speed_sps;
      }
    }

    if (axis->current_speed_sps < 0.0f)
    {
      axis->current_speed_sps = 0.0f;
    }

    StepperDebug_ApplyAxisSpeed(axis);
  }
}

static void StepperDebug_ApplyAxisSpeed(StepperAxis_t *axis)
{
  if (axis->current_speed_sps < STEPPER_TIM_STOP_THRESHOLD_SPS)
  {
    axis->tim_toggle_interval_ticks = 0U;
    axis->tim_output_running = 0U;
    StepperDebug_SetAxisStepLow(axis);
    return;
  }

  StepperDebug_StartTim1Axis(axis);
}

static uint8_t StepperDebug_AllAxesStopped(void)
{
  uint32_t axis_index;

  for (axis_index = 0U; axis_index < STEPPER_AXIS_COUNT; axis_index++)
  {
    if (stepper_axes[axis_index]->current_speed_sps > STEPPER_TIM_STOP_THRESHOLD_SPS)
    {
      return 0U;
    }
  }

  return 1U;
}

static void StepperDebug_ServiceTim1Axes(float delta_s)
{
  uint32_t axis_index;

  for (axis_index = 0U; axis_index < STEPPER_AXIS_COUNT; axis_index++)
  {
    StepperAxis_t *axis = stepper_axes[axis_index];

    if (axis->tim_output_running == 0U)
    {
      StepperDebug_SetAxisStepLow(axis);
      continue;
    }

    if (axis->current_speed_sps < STEPPER_TIM_STOP_THRESHOLD_SPS)
    {
      StepperDebug_SetAxisStepLow(axis);
      continue;
    }

    axis->software_step_accumulator += axis->current_speed_sps * delta_s;
    if (axis->software_step_accumulator >= 1.0f)
    {
      axis->software_step_accumulator -= 1.0f;
      if (axis->output_mode == STEPPER_OUTPUT_SOFTWARE_GPIO)
      {
        HAL_GPIO_WritePin(axis->step_gpio_port, axis->step_gpio_pin, GPIO_PIN_SET);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim1, axis->tim_channel, stepper_tim_pwm_pulse_ticks);
      }
    }
    else
    {
      StepperDebug_SetAxisStepLow(axis);
    }
  }
}

static void StepperDebug_Init(void)
{
  uint32_t axis_index;
  uint32_t tim_clock_hz;
  uint32_t tim_period_ticks;

  tim_clock_hz = StepperDebug_GetTim1CounterClockHz();
  tim_period_ticks = (tim_clock_hz / STEPPER_DEBUG_TIM1_SLOT_HZ);
  stepper_tim_pwm_pulse_ticks = (tim_clock_hz / 1000000U) * STEPPER_TIM_PWM_PULSE_WIDTH_US;
  if (stepper_tim_pwm_pulse_ticks == 0U)
  {
    stepper_tim_pwm_pulse_ticks = 1U;
  }
  if (tim_period_ticks < (stepper_tim_pwm_pulse_ticks + 2U))
  {
    tim_period_ticks = stepper_tim_pwm_pulse_ticks + 2U;
  }
  if (stepper_tim_pwm_pulse_ticks >= tim_period_ticks)
  {
    stepper_tim_pwm_pulse_ticks = tim_period_ticks - 1U;
  }

  __HAL_TIM_SET_AUTORELOAD(&htim1, tim_period_ticks - 1U);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  if (HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

  for (axis_index = 0U; axis_index < STEPPER_AXIS_COUNT; axis_index++)
  {
    StepperAxis_t *axis = stepper_axes[axis_index];
    axis->current_speed_sps = 0.0f;
    axis->target_speed_sps = 0.0f;
    axis->tim_toggle_interval_ticks = 0U;
    axis->tim_output_running = 0U;
    axis->software_step_accumulator = 0.0f;
    StepperDebug_SetAxisStepLow(axis);
  }

  StepperDebug_StartTim1Axis(&stepper_axis_y);
  StepperDebug_StartTim1Axis(&stepper_axis_z);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim1);
  stepper_debug_phase = STEPPER_DEBUG_PHASE_FORWARD_RUN;
  stepper_debug_phase_tick = HAL_GetTick();
  stepper_debug_last_control_tick = stepper_debug_phase_tick;
  StepperDebug_SetTargetsToCurrentPhase();
}

static void StepperDebug_Run(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint32_t elapsed_ms = now_tick - stepper_debug_last_control_tick;
  float elapsed_s;

  if (elapsed_ms >= STEPPER_DEBUG_CONTROL_PERIOD_MS)
  {
    stepper_debug_last_control_tick = now_tick;
    elapsed_s = (float)elapsed_ms / 1000.0f;
    if (stepper_control_mode == STEPPER_CONTROL_MODE_AUTO_DEBUG)
    {
      StepperDebug_UpdatePhase();
    }
    StepperDebug_UpdateAxisSpeeds(elapsed_s);
  }
}

/*
 * Stepper UART placeholder:
 *   '3'/'4' Y axis forward/reverse
 *   '5'/'6' Z axis forward/reverse
 *
 * The command handler is intentionally byte-oriented so it can be fed directly
 * from HAL_UART_RxCpltCallback() during bring-up.
 */
static void Stepper_RequestStopYAndZAxes(void)
{
  StepperDebug_EnterManualMode();
  stepper_axis_y.target_speed_sps = 0.0f;
  stepper_axis_z.target_speed_sps = 0.0f;
}

/*
 * Stepper UART command mapping handled in the main loop:
 *   '0' stop Y/Z axes with deceleration
 *   '3'/'4' Y axis forward/reverse
 *   '5'/'6' Z axis forward/reverse
 */
static void Stepper_ProcessUartCommand(uint8_t rx_byte)
{
  switch (rx_byte)
  {
    case '0':
      Stepper_RequestStopYAndZAxes();
      break;

    case '3':
      StepperDebug_CommandAxis(&stepper_axis_y, 1U, stepper_axis_y.max_speed_sps * STEPPER_UART_SINGLE_AXIS_SCALE);
      break;

    case '4':
      StepperDebug_CommandAxis(&stepper_axis_y, 0U, stepper_axis_y.max_speed_sps * STEPPER_UART_SINGLE_AXIS_SCALE);
      break;

    case '5':
      StepperDebug_CommandAxis(&stepper_axis_z, 1U, stepper_axis_z.max_speed_sps * STEPPER_UART_SINGLE_AXIS_SCALE);
      break;

    case '6':
      StepperDebug_CommandAxis(&stepper_axis_z, 0U, stepper_axis_z.max_speed_sps * STEPPER_UART_SINGLE_AXIS_SCALE);
      break;

    default:
      break;
  }
}

/* 3650-A single motor UART command placeholder */
static void Friction3650A_UartControlPlaceholder(uint8_t rx_byte)
{
  FrictionMotorCommand_t command = FRICTION_MOTOR_CMD_NONE;
  GPIO_PinState dir = friction_3650_a.dir_default;
  uint32_t duty_percent = friction_3650_a.pwm_duty_percent;

  switch (rx_byte)
  {
    case 'A':
      command = FRICTION_MOTOR_CMD_FWD_LOW;
      dir = friction_3650_a.dir_default;
      duty_percent = FRICTION_3650_A_PWM_DUTY_LOW_PERCENT;
      break;

    case 'B':
      command = FRICTION_MOTOR_CMD_FWD_HIGH;
      dir = friction_3650_a.dir_default;
      duty_percent = FRICTION_3650_A_PWM_DUTY_HIGH_PERCENT;
      break;

    case 'C':
      command = FRICTION_MOTOR_CMD_REV_LOW;
      dir = FrictionMotor_GetReverseDirection(&friction_3650_a);
      duty_percent = FRICTION_3650_A_PWM_DUTY_LOW_PERCENT;
      break;

    case 'D':
      command = FRICTION_MOTOR_CMD_REV_HIGH;
      dir = FrictionMotor_GetReverseDirection(&friction_3650_a);
      duty_percent = FRICTION_3650_A_PWM_DUTY_HIGH_PERCENT;
      break;

    default:
      break;
  }

  if (command == FRICTION_MOTOR_CMD_NONE)
  {
    return;
  }

  FrictionMotor_SetOutput(&friction_3650_a, dir, duty_percent);
}

/* 3650-B single motor UART command placeholder */
static void Friction3650BMotor_UartControlPlaceholder(uint8_t rx_byte)
{
  FrictionMotorCommand_t command = FRICTION_MOTOR_CMD_NONE;
  GPIO_PinState dir = friction_3650_b.dir_default;
  uint32_t duty_percent = friction_3650_b.pwm_duty_percent;

  switch (rx_byte)
  {
    case 'E':
      command = FRICTION_MOTOR_CMD_FWD_LOW;
      dir = friction_3650_b.dir_default;
      duty_percent = FRICTION_3650_B_DUTY_LOW_PERCENT;
      break;

    case 'F':
      command = FRICTION_MOTOR_CMD_FWD_HIGH;
      dir = friction_3650_b.dir_default;
      duty_percent = FRICTION_3650_B_DUTY_HIGH_PERCENT;
      break;

    case 'G':
      command = FRICTION_MOTOR_CMD_REV_LOW;
      dir = FrictionMotor_GetReverseDirection(&friction_3650_b);
      duty_percent = FRICTION_3650_B_DUTY_LOW_PERCENT;
      break;

    case 'H':
      command = FRICTION_MOTOR_CMD_REV_HIGH;
      dir = FrictionMotor_GetReverseDirection(&friction_3650_b);
      duty_percent = FRICTION_3650_B_DUTY_HIGH_PERCENT;
      break;

    default:
      break;
  }

  if (command == FRICTION_MOTOR_CMD_NONE)
  {
    return;
  }

  FrictionMotor_SetOutput(&friction_3650_b, dir, duty_percent);
}

static uint32_t Servo_ClampPulseWidthUs(uint32_t pulse_width_us)
{
  if (pulse_width_us < SERVO_PWM_MIN_PULSE_US)
  {
    return SERVO_PWM_MIN_PULSE_US;
  }

  if (pulse_width_us > SERVO_PWM_MAX_PULSE_US)
  {
    return SERVO_PWM_MAX_PULSE_US;
  }

  return pulse_width_us;
}

static int32_t Servo_ClampAngleDeg(int32_t angle_deg)
{
  if (angle_deg < (-SERVO_MAX_ANGLE_DEG))
  {
    return -SERVO_MAX_ANGLE_DEG;
  }

  if (angle_deg > SERVO_MAX_ANGLE_DEG)
  {
    return SERVO_MAX_ANGLE_DEG;
  }

  return angle_deg;
}

static uint32_t Servo_AngleToPulseWidthUs(int32_t angle_deg)
{
  int32_t clamped_angle = Servo_ClampAngleDeg(angle_deg);
  int32_t numerator = clamped_angle * (int32_t)(SERVO_PWM_MAX_PULSE_US - SERVO_PWM_CENTER_PULSE_US);
  int32_t offset_us;

  if (numerator >= 0)
  {
    numerator += SERVO_MAX_ANGLE_DEG / 2;
  }
  else
  {
    numerator -= SERVO_MAX_ANGLE_DEG / 2;
  }

  offset_us = numerator / SERVO_MAX_ANGLE_DEG;
  return Servo_ClampPulseWidthUs((uint32_t)((int32_t)SERVO_PWM_CENTER_PULSE_US + offset_us));
}

static void Servo_SetAngleDegrees(HardwareServo_t *servo, int32_t angle_deg)
{
  int32_t clamped_angle = Servo_ClampAngleDeg(angle_deg);

  servo->target_angle_deg = clamped_angle;
  servo->target_pulse_width_us = Servo_AngleToPulseWidthUs(clamped_angle);
  __HAL_TIM_SET_COMPARE(servo->tim, servo->tim_channel, servo->target_pulse_width_us);
}

static void Servo_Init(HardwareServo_t *servo)
{
  Servo_SetAngleDegrees(servo, servo->target_angle_deg);

  if (servo->pwm_running == 0U)
  {
    if (HAL_TIM_PWM_Start(servo->tim, servo->tim_channel) != HAL_OK)
    {
      Error_Handler();
    }
    servo->pwm_running = 1U;
  }
}

/*
 * Servo UART commands:
 *   'I' -> center (0 deg)
 *   'J' -> +135 deg
 *   'K' -> -135 deg
 */
static void Servo_UartControlPlaceholder(uint8_t rx_byte)
{
  switch (rx_byte)
  {
    case 'I':
      Servo_SetAngleDegrees(&servo_elbow, SERVO_DEFAULT_ANGLE_DEG);
      break;

    case 'J':
      Servo_SetAngleDegrees(&servo_elbow, SERVO_UART_POSITIVE_ANGLE_DEG);
      break;

    case 'K':
      Servo_SetAngleDegrees(&servo_elbow, SERVO_UART_NEGATIVE_ANGLE_DEG);
      break;

    default:
      break;
  }
}

static void Uart1_EnqueueRxByte(uint8_t rx_byte)
{
  __disable_irq();

  if (uart1_rx_cmd_count >= UART1_RX_CMD_QUEUE_SIZE)
  {
    uart1_rx_cmd_read_index++;
    if (uart1_rx_cmd_read_index >= UART1_RX_CMD_QUEUE_SIZE)
    {
      uart1_rx_cmd_read_index = 0U;
    }
    uart1_rx_cmd_count--;
  }

  uart1_rx_cmd_queue[uart1_rx_cmd_write_index] = rx_byte;
  uart1_rx_cmd_write_index++;
  if (uart1_rx_cmd_write_index >= UART1_RX_CMD_QUEUE_SIZE)
  {
    uart1_rx_cmd_write_index = 0U;
  }
  uart1_rx_cmd_count++;

  __enable_irq();
}

static uint8_t Uart1_TryDequeueRxByte(uint8_t *rx_byte)
{
  uint8_t has_data = 0U;

  __disable_irq();

  if (uart1_rx_cmd_count > 0U)
  {
    *rx_byte = uart1_rx_cmd_queue[uart1_rx_cmd_read_index];
    uart1_rx_cmd_read_index++;
    if (uart1_rx_cmd_read_index >= UART1_RX_CMD_QUEUE_SIZE)
    {
      uart1_rx_cmd_read_index = 0U;
    }
    uart1_rx_cmd_count--;
    has_data = 1U;
  }

  __enable_irq();
  return has_data;
}

static void Uart1_ProcessPendingRxCommands(void)
{
  uint8_t rx_byte;

  while (Uart1_TryDequeueRxByte(&rx_byte) != 0U)
  {
    Stepper_ProcessUartCommand(rx_byte);
    Servo_UartControlPlaceholder(rx_byte);
    Uart1_QueueAck(rx_byte);
  }
}

static void Uart1_QueueAck(uint8_t rx_byte)
{
  uart1_ack_byte = rx_byte;
  uart1_ack_pending = 1U;
}

static void Uart1_SendPendingAck(void)
{
  uint8_t rx_byte;
  int msg_len;

  if (uart1_ack_pending == 0U)
  {
    return;
  }

  __disable_irq();
  rx_byte = uart1_ack_byte;
  uart1_ack_pending = 0U;
  __enable_irq();

  if ((rx_byte >= 32U) && (rx_byte <= 126U))
  {
    msg_len = snprintf((char *)uart1_ack_tx_buf,
                       sizeof(uart1_ack_tx_buf),
                       "ACK:%c\r\n",
                       rx_byte);
  }
  else
  {
    msg_len = snprintf((char *)uart1_ack_tx_buf,
                       sizeof(uart1_ack_tx_buf),
                       "ACK:0x%02X\r\n",
                       rx_byte);
  }

  if (msg_len > 0)
  {
    HAL_UART_Transmit(&huart1, uart1_ack_tx_buf, (uint16_t)msg_len, 50);
  }
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
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  MX_TIM15_Init();
  MX_TIM24_Init();
  /* USER CODE BEGIN 2 */
  FrictionMotor_InitStopped(&friction_3650_a);
  FrictionMotor_InitStopped(&friction_3650_b);
  Chassis3650_StopAllWheels();
  StepperDebug_Init();
  StepperDebug_RequestStopAllAxes();
  Servo_Init(&servo_elbow);
  Servo_Init(&servo_block);
  Servo_Init(&servo_gripper);
  BeltDrive_StartUartReceive();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Uart1_ProcessPendingRxCommands();
    Uart1_SendPendingAck();
    StepperDebug_Run();
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
    friction_3650_a.fg_pulse_count++;
  }
  else if (GPIO_Pin == FRICTION_3650_B_FG_Pin)
  {
    friction_3650_b.fg_pulse_count++;
  }
  else if (GPIO_Pin == CHASSIS_3650_FL_FG_Pin)
  {
    chassis_3650_motors[CHASSIS_WHEEL_FRONT_LEFT].fg_pulse_count++;
  }
  else if (GPIO_Pin == CHASSIS_3650_FR_FG_Pin)
  {
    chassis_3650_motors[CHASSIS_WHEEL_FRONT_RIGHT].fg_pulse_count++;
  }
  else if (GPIO_Pin == CHASSIS_3650_RL_FG_Pin)
  {
    chassis_3650_motors[CHASSIS_WHEEL_REAR_LEFT].fg_pulse_count++;
  }
  else if (GPIO_Pin == CHASSIS_3650_RR_FG_Pin)
  {
    chassis_3650_motors[CHASSIS_WHEEL_REAR_RIGHT].fg_pulse_count++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Uart1_EnqueueRxByte(uart1_rx_byte);
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1)
  {
    return;
  }

  StepperDebug_ServiceTim1Axes(1.0f / (float)STEPPER_DEBUG_TIM1_SLOT_HZ);
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

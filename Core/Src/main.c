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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
  CHASSIS_CONTROL_MODE_OPEN_LOOP = 0,
  CHASSIS_CONTROL_MODE_CLOSED_LOOP = 1
} ChassisControlMode_t;

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
  uint32_t last_control_pulse_count;
  uint32_t last_report_pulse_count;
  uint32_t rpm;
  uint32_t pwm_command_percent;
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
  int32_t max_angle_deg;
  uint8_t pwm_running;
} HardwareServo_t;

typedef enum
{
  PROTOCOL_PARSER_STATE_WAIT_SOF1 = 0,
  PROTOCOL_PARSER_STATE_WAIT_SOF2,
  PROTOCOL_PARSER_STATE_READ_HEADER,
  PROTOCOL_PARSER_STATE_READ_PAYLOAD_AND_CRC
} ProtocolParserState_t;

typedef struct
{
  ProtocolParserState_t state;
  uint8_t header[5];
  uint8_t header_index;
  uint8_t payload_and_crc[32U + 2U];
  uint8_t payload_and_crc_index;
  uint8_t payload_len;
} ProtocolParser_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 3650-A motor control constants */
#define FRICTION_3650_A_FG_PULSES_PER_REV      6U
#define FRICTION_3650_A_PWM_TIMER_PERIOD       999U
#define FRICTION_3650_A_PWM_COMMAND_LOW_PERCENT   16U
#define FRICTION_3650_A_PWM_COMMAND_HIGH_PERCENT  30U
#define FRICTION_3650_A_DIR_DEFAULT            GPIO_PIN_SET
#define FRICTION_MOTOR_STOP_COMMAND_PERCENT    100U

/* 3650-B motor control constants */
#define FRICTION_3650_B_FG_PULSES_PER_REV       6U
#define FRICTION_3650_B_PWM_TIMER_PERIOD        999U
#define FRICTION_3650_B_PWM_COMMAND_LOW_PERCENT   16U
#define FRICTION_3650_B_PWM_COMMAND_HIGH_PERCENT  30U
#define FRICTION_3650_B_DIR_DEFAULT             GPIO_PIN_SET

/* 2430 block motor control constants */
#define BLOCK_2430_FG_PULSES_PER_REV            9U
#define BLOCK_2430_PWM_TIMER_PERIOD             999U
#define BLOCK_2430_DIR_DEFAULT                  GPIO_PIN_SET

/*
 * Chassis 3650 command values are passed through the same active-low PWM helper
 * used by the belt motors. On the current board wiring/driver chain, a larger
 * command value results in slower wheel speed, and 100 means stop.
 */
#define CHASSIS_3650_FG_PULSES_PER_REV       6U
#define CHASSIS_3650_PWM_TIMER_PERIOD        999U
#define CHASSIS_3650_DIR_DEFAULT             GPIO_PIN_SET
#define CHASSIS_3650_FL_DIR_DEFAULT          GPIO_PIN_RESET
#define CHASSIS_3650_RL_DIR_DEFAULT          GPIO_PIN_RESET
#define STEPPER_DEBUG_FORWARD_RUN_MS         2000U
#define STEPPER_DEBUG_CONTROL_PERIOD_MS      5U
#define STEPPER_AXIS_COUNT                   2U
#define STEPPER_Y_MAX_SPEED_SPS              1500.0f
#define STEPPER_Z_MAX_SPEED_SPS              2200.0f
#define STEPPER_Y_ACCEL_SPS2                 260.0f
#define STEPPER_Z_ACCEL_SPS2                 3000.0f
#define STEPPER_TIM_STOP_THRESHOLD_SPS       1.0f
#define STEPPER_DEBUG_TIM1_SLOT_HZ           10000U
#define STEPPER_TIM_PWM_PULSE_WIDTH_US       4U
#define UART1_RX_CMD_QUEUE_SIZE              128U
#define SERVO_PWM_FRAME_US                   20000U
#define SERVO_PWM_MIN_PULSE_US               500U
#define SERVO_PWM_CENTER_PULSE_US            1500U
#define SERVO_PWM_MAX_PULSE_US               2500U
#define SERVO_MAX_ANGLE_DEG_270              135
#define SERVO_MAX_ANGLE_DEG_180               90
#define SERVO_DEFAULT_ANGLE_DEG              0

#define PROTOCOL_MAX_PAYLOAD_LEN             32U
#define PROTOCOL_TX_FRAME_MAX_LEN            (2U + 5U + PROTOCOL_MAX_PAYLOAD_LEN + 2U)
#define PROTOCOL_SOF1                        0xAAU
#define PROTOCOL_SOF2                        0x55U
#define PROTOCOL_FLAG_NEED_ACK               0x01U
#define PROTOCOL_FLAG_IS_RESPONSE            0x02U
#define PROTOCOL_FLAG_IS_ERROR               0x04U
#define PROTOCOL_FLAG_IS_EVENT               0x08U

#define PROTOCOL_MODULE_SYSTEM               0x00U
#define PROTOCOL_MODULE_CHASSIS              0x10U
#define PROTOCOL_MODULE_STEPPER              0x20U
#define PROTOCOL_MODULE_SERVO                0x30U
#define PROTOCOL_MODULE_VACUUM               0x40U
#define PROTOCOL_MODULE_FRICTION             0x50U
#define PROTOCOL_MODULE_CONVEYOR             0x60U

#define PROTOCOL_CMD_PING                    0x01U
#define PROTOCOL_CMD_SET_VELOCITY            0x01U
#define PROTOCOL_CMD_SET_ANGLE               0x01U
#define PROTOCOL_CMD_SET_OUTPUT              0x01U
#define PROTOCOL_CMD_JOG                     0x01U
#define PROTOCOL_CMD_STOP                    0x02U
#define PROTOCOL_CMD_SET_CONTROL_MODE        0x03U
#define PROTOCOL_CMD_ESTOP                   0x03U
#define PROTOCOL_CMD_CLEAR_ESTOP             0x04U
#define PROTOCOL_CMD_SET_WHEEL_TARGET        0x04U
#define PROTOCOL_CMD_SET_PID                 0x05U
#define PROTOCOL_CMD_SET_WHEEL_TRIM          0x06U
#define PROTOCOL_CMD_CHASSIS_RPM_REPORT      0x81U
#define PROTOCOL_CMD_CHASSIS_CLOSED_LOOP_REPORT 0x82U

#define PROTOCOL_RESULT_OK                   0x00U
#define PROTOCOL_RESULT_BAD_LEN              0x01U
#define PROTOCOL_RESULT_BAD_PARAM            0x02U
#define PROTOCOL_RESULT_BUSY                 0x03U
#define PROTOCOL_RESULT_TIMEOUT              0x04U
#define PROTOCOL_RESULT_UNSUPPORTED          0x05U
#define PROTOCOL_RESULT_CRC_ERROR            0x06U
#define PROTOCOL_RESULT_ESTOP_ACTIVE         0x07U
#define PROTOCOL_RESULT_DEVICE_FAULT         0x08U

#define STEPPER_AXIS_MASK_Y                  0x01U
#define STEPPER_AXIS_MASK_Z                  0x02U
#define STEPPER_AXIS_MASK_ALL                (STEPPER_AXIS_MASK_Y | STEPPER_AXIS_MASK_Z)

#define FRICTION_MOTOR_MASK_A                0x01U
#define FRICTION_MOTOR_MASK_B                0x02U
#define FRICTION_MOTOR_MASK_ALL              (FRICTION_MOTOR_MASK_A | FRICTION_MOTOR_MASK_B)

#define SERVO_ID_ELBOW                       0x01U
#define SERVO_ID_BLOCK                       0x02U
#define SERVO_ID_GRIPPER                     0x03U

#define CHASSIS_PROTOCOL_STOP_COMMAND_PERCENT 100U
#define CHASSIS_PROTOCOL_MIN_COMMAND_PERCENT 20U
#define CHASSIS_PROTOCOL_MAX_LINEAR_MM_S     500
#define CHASSIS_PROTOCOL_MAX_WZ_DPS_X10      900
#define CHASSIS_PROTOCOL_DEMAND_SCALE        1000
#define CHASSIS_TRIM_MODE_SCALE              0U
#define CHASSIS_TRIM_MODE_OFFSET             1U
#define CHASSIS_TRIM_SCALE_X100_DEFAULT      100U
#define CHASSIS_TRIM_SCALE_X100_MAX          200U
#define CHASSIS_TRIM_OFFSET_DEFAULT          0
#define CHASSIS_TRIM_OFFSET_MIN              (-1000)
#define CHASSIS_TRIM_OFFSET_MAX              1000
#define CHASSIS_RPM_REPORT_PERIOD_MS         50U
#define CHASSIS_CONTROL_PERIOD_MS            20U
#define CHASSIS_MAX_TARGET_RPM               500
#define CHASSIS_WHEEL_MASK_FL                0x01U
#define CHASSIS_WHEEL_MASK_FR                0x02U
#define CHASSIS_WHEEL_MASK_RL                0x04U
#define CHASSIS_WHEEL_MASK_RR                0x08U
#define CHASSIS_WHEEL_MASK_ALL               (CHASSIS_WHEEL_MASK_FL | CHASSIS_WHEEL_MASK_FR | CHASSIS_WHEEL_MASK_RL | CHASSIS_WHEEL_MASK_RR)
#define CHASSIS_STATUS_FLAG_TIMEOUT_ACTIVE   0x01U
#define CHASSIS_STATUS_FLAG_ESTOP_ACTIVE     0x02U
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

static FrictionMotor_t block_2430_motor =
{
  &htim4,
  TIM_CHANNEL_1,
  BLOCK_2430_PWM_TIMER_PERIOD,
  BLOCK_2430_FG_PULSES_PER_REV,
  BLOCK_2430_DIR_GPIO_Port,
  BLOCK_2430_DIR_Pin,
  BLOCK_2430_DIR_DEFAULT,
  0U,
  0U,
  0U,
  0U,
  BLOCK_2430_DIR_DEFAULT,
  0U
};

static uint8_t uart1_rx_byte = 0U;
static volatile uint8_t uart1_rx_cmd_queue[UART1_RX_CMD_QUEUE_SIZE];
static volatile uint8_t uart1_rx_cmd_read_index = 0U;
static volatile uint8_t uart1_rx_cmd_write_index = 0U;
static volatile uint8_t uart1_rx_cmd_count = 0U;
static uint8_t protocol_tx_frame_buf[PROTOCOL_TX_FRAME_MAX_LEN];
static ProtocolParser_t uart1_protocol_parser =
{
  PROTOCOL_PARSER_STATE_WAIT_SOF1,
  {0U},
  0U,
  {0U},
  0U,
  0U
};
static uint8_t protocol_estop_active = 0U;
static uint8_t chassis_timeout_active = 0U;
static uint8_t vacuum_timeout_active = 0U;
static uint8_t conveyor_timeout_active = 0U;
static uint32_t chassis_timeout_deadline_tick = 0U;
static uint32_t vacuum_timeout_deadline_tick = 0U;
static uint32_t conveyor_timeout_deadline_tick = 0U;
static uint32_t chassis_rpm_report_last_tick = 0U;
static uint32_t chassis_control_last_tick = 0U;
static ChassisControlMode_t chassis_control_mode = CHASSIS_CONTROL_MODE_OPEN_LOOP;
static int16_t chassis_target_rpm[CHASSIS_WHEEL_COUNT] = {0};
static int16_t chassis_actual_rpm_signed[CHASSIS_WHEEL_COUNT] = {0};
static uint8_t chassis_trim_mode = CHASSIS_TRIM_MODE_SCALE;
static uint16_t chassis_trim_scale_x100[CHASSIS_WHEEL_COUNT] = {100U, 100U, 100U, 100U};
static int16_t chassis_trim_offset[CHASSIS_WHEEL_COUNT] = {0, 0, 0, 0};
static uint16_t chassis_pi_kp_x1000[CHASSIS_WHEEL_COUNT] = {180, 180, 180, 180};
static uint16_t chassis_pi_ki_x1000[CHASSIS_WHEEL_COUNT] = {20, 20, 20, 20};
static uint16_t chassis_pi_kd_x1000[CHASSIS_WHEEL_COUNT] = {0, 0, 0, 0};
static int32_t chassis_pi_integral[CHASSIS_WHEEL_COUNT] = {0};
static ChassisMotor_t chassis_3650_motors[CHASSIS_WHEEL_COUNT] =
{
  {&htim3, TIM_CHANNEL_1, CHASSIS_3650_PWM_TIMER_PERIOD, CHASSIS_3650_FG_PULSES_PER_REV,
   CHASSIS_3650_FL_DIR_GPIO_Port, CHASSIS_3650_FL_DIR_Pin, CHASSIS_3650_FL_DIR_DEFAULT,
   0U, 0U, 0U, 0U, 0U, CHASSIS_3650_FL_DIR_DEFAULT, 0U},
  {&htim2, TIM_CHANNEL_4, CHASSIS_3650_PWM_TIMER_PERIOD, CHASSIS_3650_FG_PULSES_PER_REV,
   CHASSIS_3650_FR_DIR_GPIO_Port, CHASSIS_3650_FR_DIR_Pin, CHASSIS_3650_DIR_DEFAULT,
   0U, 0U, 0U, 0U, 0U, CHASSIS_3650_DIR_DEFAULT, 0U},
  {&htim4, TIM_CHANNEL_3, CHASSIS_3650_PWM_TIMER_PERIOD, CHASSIS_3650_FG_PULSES_PER_REV,
   CHASSIS_3650_RL_DIR_GPIO_Port, CHASSIS_3650_RL_DIR_Pin, CHASSIS_3650_RL_DIR_DEFAULT,
   0U, 0U, 0U, 0U, 0U, CHASSIS_3650_RL_DIR_DEFAULT, 0U},
  {&htim4, TIM_CHANNEL_4, CHASSIS_3650_PWM_TIMER_PERIOD, CHASSIS_3650_FG_PULSES_PER_REV,
   CHASSIS_3650_RR_DIR_GPIO_Port, CHASSIS_3650_RR_DIR_Pin, CHASSIS_3650_DIR_DEFAULT,
   0U, 0U, 0U, 0U, 0U, CHASSIS_3650_DIR_DEFAULT, 0U}
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
  SERVO_MAX_ANGLE_DEG_270,
  0U
};
static HardwareServo_t servo_block =
{
  &htim13,
  TIM_CHANNEL_1,
  SERVO_PWM_CENTER_PULSE_US,
  SERVO_DEFAULT_ANGLE_DEG,
  SERVO_MAX_ANGLE_DEG_270,
  0U
};
static HardwareServo_t servo_gripper =
{
  &htim8,
  TIM_CHANNEL_2,
  SERVO_PWM_CENTER_PULSE_US,
  SERVO_DEFAULT_ANGLE_DEG,
  SERVO_MAX_ANGLE_DEG_180,
  0U
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t FrictionMotor_GetPwmCompareFromCommand(const FrictionMotor_t *motor, uint32_t pwm_command_percent);
static void FrictionMotor_StopOutput(FrictionMotor_t *motor);
static void FrictionMotor_SetOutput(FrictionMotor_t *motor, GPIO_PinState dir, uint32_t pwm_command_percent);
static void FrictionMotor_InitStopped(FrictionMotor_t *motor);
static GPIO_PinState FrictionMotor_GetReverseDirection(const FrictionMotor_t *motor);
static void BeltDrive_StartUartReceive(void);
static void Chassis3650_ApplyToAllWheels(ChassisDirection_t direction, uint32_t pwm_command_percent);
static void Chassis3650_SetWheelOutput(ChassisWheel_t wheel, ChassisDirection_t direction, uint32_t pwm_command_percent);
static void Chassis3650_StopAllWheels(void);
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
static void StepperDebug_RequestStopAllAxes(void);
static void StepperDebug_SetTargetsToCurrentPhase(void);
static void StepperDebug_UpdatePhase(void);
static void StepperDebug_UpdateAxisSpeeds(float delta_s);
static void StepperDebug_ApplyAxisSpeed(StepperAxis_t *axis);
static uint8_t StepperDebug_AllAxesStopped(void);
static void StepperDebug_ServiceTim1Axes(float delta_s);
static uint32_t Servo_ClampPulseWidthUs(uint32_t pulse_width_us);
static int32_t Servo_ClampAngleDeg(const HardwareServo_t *servo, int32_t angle_deg);
static uint32_t Servo_AngleToPulseWidthUs(const HardwareServo_t *servo, int32_t angle_deg);
static void Servo_Init(HardwareServo_t *servo);
static void Servo_SetAngleDegrees(HardwareServo_t *servo, int32_t angle_deg);
static uint16_t Protocol_Crc16Modbus(const uint8_t *data, uint16_t length);
static uint16_t Protocol_ReadU16Le(const uint8_t *data);
static int16_t Protocol_ReadS16Le(const uint8_t *data);
static uint32_t Protocol_ClampPwmCommandPercent(uint32_t pwm_command_percent);
static uint8_t Protocol_IsTickExpired(uint32_t now_tick, uint32_t deadline_tick);
static void Protocol_ClearParser(ProtocolParser_t *parser);
static void Protocol_ProcessRxByte(uint8_t rx_byte);
static void Protocol_HandleFrame(const ProtocolParser_t *parser);
static void Protocol_SendFrame(uint8_t flags, uint8_t seq, uint8_t module, uint8_t cmd, const uint8_t *payload, uint8_t payload_len);
static void Protocol_SendResultResponse(uint8_t seq, uint8_t module, uint8_t cmd, uint8_t result);
static uint8_t Protocol_ShouldBlockForEstop(uint8_t module, uint8_t cmd);
static HardwareServo_t *Servo_GetById(uint8_t servo_id);
static int32_t Servo_DeciDegreesToDegrees(int16_t angle_deg_x10);
static void Chassis3650_ApplyVelocity(int16_t vx_mm_s, int16_t vy_mm_s, int16_t wz_dps_x10);
static uint8_t Chassis3650_GetWheelMask(ChassisWheel_t wheel);
static void Chassis3650_ClearClosedLoopState(void);
static void Chassis3650_SetControlMode(ChassisControlMode_t mode);
static void Chassis3650_SetWheelTargetsRpm(int16_t fl_rpm, int16_t fr_rpm, int16_t rl_rpm, int16_t rr_rpm);
static void Chassis3650_SetWheelTrim(uint8_t trim_mode, int16_t fl_value, int16_t fr_value, int16_t rl_value, int16_t rr_value);
static void Chassis3650_SetPidByMask(uint8_t wheel_mask, uint16_t kp_x1000, uint16_t ki_x1000, uint16_t kd_x1000);
static void Chassis3650_UpdateMeasuredSpeeds(uint32_t elapsed_ms);
static void Chassis3650_RunClosedLoopControl(uint32_t elapsed_ms);
static void Chassis3650_ServiceControlLoop(void);
static void Stepper_SetJog(uint8_t axis_mask, uint8_t direction, uint16_t speed_sps, uint16_t accel_sps2);
static void Stepper_StopAxes(uint8_t axis_mask);
static void Stepper_EmergencyStopAllAxes(void);
static void Vacuum_SetOutput(uint8_t pump_on, uint8_t valve_on);
static void Vacuum_Stop(void);
static void FrictionMotors_SetOutput(uint8_t motor_mask, uint8_t direction, uint8_t pwm_command_percent);
static void FrictionMotors_Stop(void);
static void Conveyor_SetOutput(uint8_t direction, uint8_t pwm_command_percent);
static void Conveyor_Stop(void);
static void Protocol_DispatchPacket(uint8_t flags, uint8_t seq, uint8_t module, uint8_t cmd, const uint8_t *payload, uint8_t payload_len);
static void Protocol_ApplyEmergencyStop(void);
static void Protocol_ClearEmergencyStop(void);
static void Protocol_ServiceTimeouts(void);
static void Protocol_ReportChassisRpmIfReady(void);
static void Protocol_InitRuntime(void);
static void Uart1_EnqueueRxByte(uint8_t rx_byte);
static uint8_t Uart1_TryDequeueRxByte(uint8_t *rx_byte);
static void Uart1_ProcessPendingRxCommands(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Convert logical motor command percent to timer compare for the active-low
 * driver chain used on these 3650 outputs. On current hardware, larger command
 * values produce less effective drive, so 100 maps to stop.
 */
static uint32_t FrictionMotor_GetPwmCompareFromCommand(const FrictionMotor_t *motor, uint32_t pwm_command_percent)
{
  uint32_t inverted_command_percent;

  if (pwm_command_percent >= 100U)
  {
    return 0U;
  }

  inverted_command_percent = 100U - pwm_command_percent;
  return ((motor->pwm_timer_period + 1U) * inverted_command_percent) / 100U;
}

static void FrictionMotor_StopOutput(FrictionMotor_t *motor)
{
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
                        FrictionMotor_GetPwmCompareFromCommand(motor, FRICTION_MOTOR_STOP_COMMAND_PERCENT));
  motor->pwm_command_percent = FRICTION_MOTOR_STOP_COMMAND_PERCENT;
}

static void FrictionMotor_SetOutput(FrictionMotor_t *motor, GPIO_PinState dir, uint32_t pwm_command_percent)
{
  motor->dir_state = dir;
  motor->pwm_command_percent = pwm_command_percent;
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
                        FrictionMotor_GetPwmCompareFromCommand(motor, motor->pwm_command_percent));
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

static void Chassis3650_ApplyToAllWheels(ChassisDirection_t direction, uint32_t pwm_command_percent)
{
  uint32_t wheel_index;

  for (wheel_index = 0U; wheel_index < (uint32_t)CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    Chassis3650_SetWheelOutput((ChassisWheel_t)wheel_index, direction, pwm_command_percent);
  }
}

static void Chassis3650_SetWheelOutput(ChassisWheel_t wheel, ChassisDirection_t direction, uint32_t pwm_command_percent)
{
  ChassisMotor_t *motor;
  GPIO_PinState dir;

  if ((uint32_t)wheel >= (uint32_t)CHASSIS_WHEEL_COUNT)
  {
    return;
  }

  motor = &chassis_3650_motors[(uint32_t)wheel];

  if ((direction == CHASSIS_STOP) || (pwm_command_percent == 0U))
  {
    FrictionMotor_StopOutput(motor);
    return;
  }

  dir = (direction == CHASSIS_FORWARD) ? motor->dir_default : FrictionMotor_GetReverseDirection(motor);
  FrictionMotor_SetOutput(motor, dir, pwm_command_percent);
}

static void Chassis3650_StopAllWheels(void)
{
  Chassis3650_ClearClosedLoopState();
  Chassis3650_ApplyToAllWheels(CHASSIS_STOP, 0U);
}

static uint8_t Chassis3650_GetWheelMask(ChassisWheel_t wheel)
{
  switch (wheel)
  {
    case CHASSIS_WHEEL_FRONT_LEFT:
      return CHASSIS_WHEEL_MASK_FL;
    case CHASSIS_WHEEL_FRONT_RIGHT:
      return CHASSIS_WHEEL_MASK_FR;
    case CHASSIS_WHEEL_REAR_LEFT:
      return CHASSIS_WHEEL_MASK_RL;
    case CHASSIS_WHEEL_REAR_RIGHT:
      return CHASSIS_WHEEL_MASK_RR;
    default:
      return 0U;
  }
}

static void Chassis3650_ClearClosedLoopState(void)
{
  uint32_t wheel_index;

  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    chassis_target_rpm[wheel_index] = 0;
    chassis_actual_rpm_signed[wheel_index] = 0;
    chassis_pi_integral[wheel_index] = 0;
  }
}

static void Chassis3650_SetControlMode(ChassisControlMode_t mode)
{
  chassis_control_mode = mode;
  Chassis3650_ClearClosedLoopState();
}

static void Chassis3650_SetWheelTargetsRpm(int16_t fl_rpm, int16_t fr_rpm, int16_t rl_rpm, int16_t rr_rpm)
{
  chassis_target_rpm[CHASSIS_WHEEL_FRONT_LEFT] = fl_rpm;
  chassis_target_rpm[CHASSIS_WHEEL_FRONT_RIGHT] = fr_rpm;
  chassis_target_rpm[CHASSIS_WHEEL_REAR_LEFT] = rl_rpm;
  chassis_target_rpm[CHASSIS_WHEEL_REAR_RIGHT] = rr_rpm;
}

static void Chassis3650_SetWheelTrim(uint8_t trim_mode, int16_t fl_value, int16_t fr_value, int16_t rl_value, int16_t rr_value)
{
  chassis_trim_mode = trim_mode;
  if (trim_mode == CHASSIS_TRIM_MODE_OFFSET)
  {
    chassis_trim_offset[CHASSIS_WHEEL_FRONT_LEFT] = fl_value;
    chassis_trim_offset[CHASSIS_WHEEL_FRONT_RIGHT] = fr_value;
    chassis_trim_offset[CHASSIS_WHEEL_REAR_LEFT] = rl_value;
    chassis_trim_offset[CHASSIS_WHEEL_REAR_RIGHT] = rr_value;
    chassis_trim_scale_x100[CHASSIS_WHEEL_FRONT_LEFT] = CHASSIS_TRIM_SCALE_X100_DEFAULT;
    chassis_trim_scale_x100[CHASSIS_WHEEL_FRONT_RIGHT] = CHASSIS_TRIM_SCALE_X100_DEFAULT;
    chassis_trim_scale_x100[CHASSIS_WHEEL_REAR_LEFT] = CHASSIS_TRIM_SCALE_X100_DEFAULT;
    chassis_trim_scale_x100[CHASSIS_WHEEL_REAR_RIGHT] = CHASSIS_TRIM_SCALE_X100_DEFAULT;
    return;
  }

  chassis_trim_scale_x100[CHASSIS_WHEEL_FRONT_LEFT] = (uint16_t)fl_value;
  chassis_trim_scale_x100[CHASSIS_WHEEL_FRONT_RIGHT] = (uint16_t)fr_value;
  chassis_trim_scale_x100[CHASSIS_WHEEL_REAR_LEFT] = (uint16_t)rl_value;
  chassis_trim_scale_x100[CHASSIS_WHEEL_REAR_RIGHT] = (uint16_t)rr_value;
  chassis_trim_offset[CHASSIS_WHEEL_FRONT_LEFT] = CHASSIS_TRIM_OFFSET_DEFAULT;
  chassis_trim_offset[CHASSIS_WHEEL_FRONT_RIGHT] = CHASSIS_TRIM_OFFSET_DEFAULT;
  chassis_trim_offset[CHASSIS_WHEEL_REAR_LEFT] = CHASSIS_TRIM_OFFSET_DEFAULT;
  chassis_trim_offset[CHASSIS_WHEEL_REAR_RIGHT] = CHASSIS_TRIM_OFFSET_DEFAULT;
}

static void Chassis3650_SetPidByMask(uint8_t wheel_mask, uint16_t kp_x1000, uint16_t ki_x1000, uint16_t kd_x1000)
{
  uint32_t wheel_index;

  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    if ((wheel_mask & Chassis3650_GetWheelMask((ChassisWheel_t)wheel_index)) == 0U)
    {
      continue;
    }

    chassis_pi_kp_x1000[wheel_index] = kp_x1000;
    chassis_pi_ki_x1000[wheel_index] = ki_x1000;
    chassis_pi_kd_x1000[wheel_index] = kd_x1000;
    chassis_pi_integral[wheel_index] = 0;
  }
}

static void Chassis3650_UpdateMeasuredSpeeds(uint32_t elapsed_ms)
{
  uint32_t wheel_index;

  if (elapsed_ms == 0U)
  {
    return;
  }

  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    uint32_t pulse_snapshot;
    uint32_t pulse_delta;
    uint32_t rpm_abs;
    int32_t signed_rpm;
    ChassisMotor_t *motor = &chassis_3650_motors[wheel_index];

    __disable_irq();
    pulse_snapshot = motor->fg_pulse_count;
    __enable_irq();

    pulse_delta = pulse_snapshot - motor->last_control_pulse_count;
    motor->last_control_pulse_count = pulse_snapshot;

    rpm_abs = (pulse_delta * 60000U) / (motor->fg_pulses_per_rev * elapsed_ms);
    motor->rpm = rpm_abs;

    if (motor->pwm_command_percent >= CHASSIS_PROTOCOL_STOP_COMMAND_PERCENT)
    {
      signed_rpm = 0;
    }
    else if (motor->dir_state == motor->dir_default)
    {
      signed_rpm = (int32_t)rpm_abs;
    }
    else
    {
      signed_rpm = -(int32_t)rpm_abs;
    }

    if (signed_rpm > 32767)
    {
      signed_rpm = 32767;
    }
    else if (signed_rpm < -32768)
    {
      signed_rpm = -32768;
    }

    chassis_actual_rpm_signed[wheel_index] = (int16_t)signed_rpm;
  }
}

static void Chassis3650_RunClosedLoopControl(uint32_t elapsed_ms)
{
  uint32_t wheel_index;

  if (elapsed_ms == 0U)
  {
    return;
  }

  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    int32_t target_rpm = chassis_target_rpm[wheel_index];
    int32_t actual_rpm = chassis_actual_rpm_signed[wheel_index];
    int32_t error_rpm = target_rpm - actual_rpm;
    int32_t p_term;
    int32_t i_term;
    int32_t control_effort;
    int32_t pwm_command_signed;
    uint32_t pwm_command_percent;
    ChassisDirection_t direction = CHASSIS_FORWARD;

    if (target_rpm == 0)
    {
      chassis_pi_integral[wheel_index] = 0;
      Chassis3650_SetWheelOutput((ChassisWheel_t)wheel_index, CHASSIS_STOP, 0U);
      continue;
    }

    chassis_pi_integral[wheel_index] += (error_rpm * (int32_t)elapsed_ms);
    if (chassis_pi_integral[wheel_index] > 200000)
    {
      chassis_pi_integral[wheel_index] = 200000;
    }
    else if (chassis_pi_integral[wheel_index] < -200000)
    {
      chassis_pi_integral[wheel_index] = -200000;
    }

    p_term = (error_rpm * (int32_t)chassis_pi_kp_x1000[wheel_index]) / 1000;
    i_term = (chassis_pi_integral[wheel_index] * (int32_t)chassis_pi_ki_x1000[wheel_index]) / 100000;
    control_effort = p_term + i_term;

    if (target_rpm < 0)
    {
      direction = CHASSIS_REVERSE;
    }

    pwm_command_signed = (int32_t)CHASSIS_PROTOCOL_STOP_COMMAND_PERCENT - control_effort;
    if (pwm_command_signed > (int32_t)CHASSIS_PROTOCOL_STOP_COMMAND_PERCENT)
    {
      pwm_command_percent = CHASSIS_PROTOCOL_STOP_COMMAND_PERCENT;
    }
    else if (pwm_command_signed < (int32_t)CHASSIS_PROTOCOL_MIN_COMMAND_PERCENT)
    {
      pwm_command_percent = CHASSIS_PROTOCOL_MIN_COMMAND_PERCENT;
    }
    else
    {
      pwm_command_percent = (uint32_t)pwm_command_signed;
    }

    Chassis3650_SetWheelOutput((ChassisWheel_t)wheel_index, direction, pwm_command_percent);
  }
}

static void Chassis3650_ServiceControlLoop(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint32_t elapsed_ms;

  if (chassis_control_last_tick == 0U)
  {
    chassis_control_last_tick = now_tick;
    return;
  }

  elapsed_ms = now_tick - chassis_control_last_tick;
  if (elapsed_ms < CHASSIS_CONTROL_PERIOD_MS)
  {
    return;
  }

  chassis_control_last_tick = now_tick;
  Chassis3650_UpdateMeasuredSpeeds(elapsed_ms);

  if (chassis_control_mode == CHASSIS_CONTROL_MODE_CLOSED_LOOP)
  {
    Chassis3650_RunClosedLoopControl(elapsed_ms);
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
  stepper_control_mode = STEPPER_CONTROL_MODE_MANUAL;
  stepper_debug_phase = STEPPER_DEBUG_PHASE_FORWARD_RUN;
  stepper_debug_phase_tick = HAL_GetTick();
  stepper_debug_last_control_tick = stepper_debug_phase_tick;
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

static int32_t Servo_ClampAngleDeg(const HardwareServo_t *servo, int32_t angle_deg)
{
  if (angle_deg < (-servo->max_angle_deg))
  {
    return -servo->max_angle_deg;
  }

  if (angle_deg > servo->max_angle_deg)
  {
    return servo->max_angle_deg;
  }

  return angle_deg;
}

static uint32_t Servo_AngleToPulseWidthUs(const HardwareServo_t *servo, int32_t angle_deg)
{
  int32_t clamped_angle = Servo_ClampAngleDeg(servo, angle_deg);
  int32_t numerator = clamped_angle * (int32_t)(SERVO_PWM_MAX_PULSE_US - SERVO_PWM_CENTER_PULSE_US);
  int32_t offset_us;

  if (numerator >= 0)
  {
    numerator += servo->max_angle_deg / 2;
  }
  else
  {
    numerator -= servo->max_angle_deg / 2;
  }

  offset_us = numerator / servo->max_angle_deg;
  return Servo_ClampPulseWidthUs((uint32_t)((int32_t)SERVO_PWM_CENTER_PULSE_US + offset_us));
}

static void Servo_SetAngleDegrees(HardwareServo_t *servo, int32_t angle_deg)
{
  int32_t clamped_angle = Servo_ClampAngleDeg(servo, angle_deg);

  servo->target_angle_deg = clamped_angle;
  servo->target_pulse_width_us = Servo_AngleToPulseWidthUs(servo, clamped_angle);
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

static uint16_t Protocol_Crc16Modbus(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t byte_index;
  uint8_t bit_index;

  for (byte_index = 0U; byte_index < length; byte_index++)
  {
    crc ^= data[byte_index];
    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

static uint16_t Protocol_ReadU16Le(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int16_t Protocol_ReadS16Le(const uint8_t *data)
{
  return (int16_t)Protocol_ReadU16Le(data);
}

static uint32_t Protocol_ClampPwmCommandPercent(uint32_t pwm_command_percent)
{
  if (pwm_command_percent > 100U)
  {
    return 100U;
  }

  return pwm_command_percent;
}

static uint8_t Protocol_IsTickExpired(uint32_t now_tick, uint32_t deadline_tick)
{
  return ((int32_t)(now_tick - deadline_tick) >= 0) ? 1U : 0U;
}

static void Protocol_ClearParser(ProtocolParser_t *parser)
{
  parser->state = PROTOCOL_PARSER_STATE_WAIT_SOF1;
  parser->header_index = 0U;
  parser->payload_and_crc_index = 0U;
  parser->payload_len = 0U;
}

static void Protocol_ProcessRxByte(uint8_t rx_byte)
{
  switch (uart1_protocol_parser.state)
  {
    case PROTOCOL_PARSER_STATE_WAIT_SOF1:
      if (rx_byte == PROTOCOL_SOF1)
      {
        uart1_protocol_parser.state = PROTOCOL_PARSER_STATE_WAIT_SOF2;
      }
      break;

    case PROTOCOL_PARSER_STATE_WAIT_SOF2:
      if (rx_byte == PROTOCOL_SOF2)
      {
        uart1_protocol_parser.state = PROTOCOL_PARSER_STATE_READ_HEADER;
        uart1_protocol_parser.header_index = 0U;
      }
      else if (rx_byte != PROTOCOL_SOF1)
      {
        uart1_protocol_parser.state = PROTOCOL_PARSER_STATE_WAIT_SOF1;
      }
      break;

    case PROTOCOL_PARSER_STATE_READ_HEADER:
      uart1_protocol_parser.header[uart1_protocol_parser.header_index++] = rx_byte;
      if (uart1_protocol_parser.header_index >= sizeof(uart1_protocol_parser.header))
      {
        uart1_protocol_parser.payload_len = uart1_protocol_parser.header[4];
        if (uart1_protocol_parser.payload_len > PROTOCOL_MAX_PAYLOAD_LEN)
        {
          Protocol_ClearParser(&uart1_protocol_parser);
          break;
        }

        uart1_protocol_parser.payload_and_crc_index = 0U;
        uart1_protocol_parser.state = PROTOCOL_PARSER_STATE_READ_PAYLOAD_AND_CRC;
      }
      break;

    case PROTOCOL_PARSER_STATE_READ_PAYLOAD_AND_CRC:
      uart1_protocol_parser.payload_and_crc[uart1_protocol_parser.payload_and_crc_index++] = rx_byte;
      if (uart1_protocol_parser.payload_and_crc_index >= (uint8_t)(uart1_protocol_parser.payload_len + 2U))
      {
        Protocol_HandleFrame(&uart1_protocol_parser);
        Protocol_ClearParser(&uart1_protocol_parser);
      }
      break;

    default:
      Protocol_ClearParser(&uart1_protocol_parser);
      break;
  }
}

static void Protocol_HandleFrame(const ProtocolParser_t *parser)
{
  uint8_t crc_input[5U + PROTOCOL_MAX_PAYLOAD_LEN];
  uint16_t computed_crc;
  uint16_t received_crc;
  uint8_t payload_len = parser->payload_len;
  uint8_t flags = parser->header[0];
  uint8_t seq = parser->header[1];
  uint8_t module = parser->header[2];
  uint8_t cmd = parser->header[3];

  memcpy(crc_input, parser->header, sizeof(parser->header));
  if (payload_len > 0U)
  {
    memcpy(&crc_input[sizeof(parser->header)], parser->payload_and_crc, payload_len);
  }

  computed_crc = Protocol_Crc16Modbus(crc_input, (uint16_t)(sizeof(parser->header) + payload_len));
  received_crc = Protocol_ReadU16Le(&parser->payload_and_crc[payload_len]);

  if (computed_crc != received_crc)
  {
    Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_CRC_ERROR);
    return;
  }

  Protocol_DispatchPacket(flags, seq, module, cmd, parser->payload_and_crc, payload_len);
}

static void Protocol_SendFrame(uint8_t flags, uint8_t seq, uint8_t module, uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  uint16_t crc;
  uint16_t tx_len;

  if (payload_len > PROTOCOL_MAX_PAYLOAD_LEN)
  {
    return;
  }

  protocol_tx_frame_buf[0] = PROTOCOL_SOF1;
  protocol_tx_frame_buf[1] = PROTOCOL_SOF2;
  protocol_tx_frame_buf[2] = flags;
  protocol_tx_frame_buf[3] = seq;
  protocol_tx_frame_buf[4] = module;
  protocol_tx_frame_buf[5] = cmd;
  protocol_tx_frame_buf[6] = payload_len;

  if ((payload != NULL) && (payload_len > 0U))
  {
    memcpy(&protocol_tx_frame_buf[7], payload, payload_len);
  }

  crc = Protocol_Crc16Modbus(&protocol_tx_frame_buf[2], (uint16_t)(5U + payload_len));
  protocol_tx_frame_buf[7U + payload_len] = (uint8_t)(crc & 0xFFU);
  protocol_tx_frame_buf[8U + payload_len] = (uint8_t)((crc >> 8U) & 0xFFU);
  tx_len = (uint16_t)(9U + payload_len);

  HAL_UART_Transmit(&huart1, protocol_tx_frame_buf, tx_len, 100U);
}

static void Protocol_SendResultResponse(uint8_t seq, uint8_t module, uint8_t cmd, uint8_t result)
{
  uint8_t payload[3];
  uint8_t flags = PROTOCOL_FLAG_IS_RESPONSE;

  payload[0] = result;
  payload[1] = module;
  payload[2] = cmd;
  if (result != PROTOCOL_RESULT_OK)
  {
    flags |= PROTOCOL_FLAG_IS_ERROR;
  }

  Protocol_SendFrame(flags, seq, module, cmd, payload, sizeof(payload));
}

static uint8_t Protocol_ShouldBlockForEstop(uint8_t module, uint8_t cmd)
{
  if (module != PROTOCOL_MODULE_SYSTEM)
  {
    return 1U;
  }

  if ((cmd == PROTOCOL_CMD_PING) || (cmd == PROTOCOL_CMD_ESTOP) || (cmd == PROTOCOL_CMD_CLEAR_ESTOP))
  {
    return 0U;
  }

  return 1U;
}

static HardwareServo_t *Servo_GetById(uint8_t servo_id)
{
  switch (servo_id)
  {
    case SERVO_ID_ELBOW:
      return &servo_elbow;

    case SERVO_ID_BLOCK:
      return &servo_block;

    case SERVO_ID_GRIPPER:
      return &servo_gripper;

    default:
      return NULL;
  }
}

static int32_t Servo_DeciDegreesToDegrees(int16_t angle_deg_x10)
{
  int32_t rounded = angle_deg_x10;

  if (rounded >= 0)
  {
    rounded += 5;
  }
  else
  {
    rounded -= 5;
  }

  return rounded / 10;
}

/*
 * Servo UART commands:
 *   'I' -> center (0 deg)
 *   'J' -> +135 deg
 *   'K' -> -135 deg
 */
static void Chassis3650_ApplyVelocity(int16_t vx_mm_s, int16_t vy_mm_s, int16_t wz_dps_x10)
{
  int32_t demands[CHASSIS_WHEEL_COUNT];
  int32_t normalized_vx;
  int32_t normalized_vy;
  int32_t normalized_wz;
  int32_t max_abs_demand = 0;
  uint32_t wheel_index;

  normalized_vx = ((int32_t)vx_mm_s * CHASSIS_PROTOCOL_DEMAND_SCALE) / CHASSIS_PROTOCOL_MAX_LINEAR_MM_S;
  normalized_vy = ((int32_t)vy_mm_s * CHASSIS_PROTOCOL_DEMAND_SCALE) / CHASSIS_PROTOCOL_MAX_LINEAR_MM_S;
  normalized_wz = ((int32_t)wz_dps_x10 * CHASSIS_PROTOCOL_DEMAND_SCALE) / CHASSIS_PROTOCOL_MAX_WZ_DPS_X10;

  demands[CHASSIS_WHEEL_FRONT_LEFT] = normalized_vx - normalized_vy - normalized_wz;
  demands[CHASSIS_WHEEL_FRONT_RIGHT] = normalized_vx + normalized_vy + normalized_wz;
  demands[CHASSIS_WHEEL_REAR_LEFT] = normalized_vx + normalized_vy - normalized_wz;
  demands[CHASSIS_WHEEL_REAR_RIGHT] = normalized_vx - normalized_vy + normalized_wz;

  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    int32_t abs_demand = demands[wheel_index];
    if (abs_demand < 0)
    {
      abs_demand = -abs_demand;
    }
    if (abs_demand > max_abs_demand)
    {
      max_abs_demand = abs_demand;
    }
  }

  if (max_abs_demand <= 0)
  {
    Chassis3650_StopAllWheels();
    return;
  }

  Chassis3650_SetControlMode(CHASSIS_CONTROL_MODE_OPEN_LOOP);

  if (max_abs_demand > CHASSIS_PROTOCOL_DEMAND_SCALE)
  {
    for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
      demands[wheel_index] = (demands[wheel_index] * CHASSIS_PROTOCOL_DEMAND_SCALE) / max_abs_demand;
    }
  }

  if (chassis_trim_mode == CHASSIS_TRIM_MODE_OFFSET)
  {
    for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
      demands[wheel_index] += (int32_t)chassis_trim_offset[wheel_index];
    }
  }
  else
  {
    for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
      demands[wheel_index] = (demands[wheel_index] * (int32_t)chassis_trim_scale_x100[wheel_index]) / 100;
    }
  }

  max_abs_demand = 0;
  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    int32_t abs_demand = demands[wheel_index];
    if (abs_demand < 0)
    {
      abs_demand = -abs_demand;
    }
    if (abs_demand > max_abs_demand)
    {
      max_abs_demand = abs_demand;
    }
  }

  if (max_abs_demand > CHASSIS_PROTOCOL_DEMAND_SCALE)
  {
    for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
      demands[wheel_index] = (demands[wheel_index] * CHASSIS_PROTOCOL_DEMAND_SCALE) / max_abs_demand;
    }
  }

  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    int32_t demand = demands[wheel_index];
    uint32_t command_percent;
    ChassisDirection_t direction = CHASSIS_FORWARD;

    if (demand < 0)
    {
      direction = CHASSIS_REVERSE;
      demand = -demand;
    }

    command_percent = CHASSIS_PROTOCOL_STOP_COMMAND_PERCENT -
                      (uint32_t)((demand * (int32_t)(CHASSIS_PROTOCOL_STOP_COMMAND_PERCENT - CHASSIS_PROTOCOL_MIN_COMMAND_PERCENT)) /
                                 CHASSIS_PROTOCOL_DEMAND_SCALE);
    if (command_percent < CHASSIS_PROTOCOL_MIN_COMMAND_PERCENT)
    {
      command_percent = CHASSIS_PROTOCOL_MIN_COMMAND_PERCENT;
    }

    Chassis3650_SetWheelOutput((ChassisWheel_t)wheel_index, direction, command_percent);
  }
}

static void Stepper_SetJog(uint8_t axis_mask, uint8_t direction, uint16_t speed_sps, uint16_t accel_sps2)
{
  StepperDebug_EnterManualMode();

  if ((axis_mask & STEPPER_AXIS_MASK_Y) != 0U)
  {
    stepper_axis_y.accel_sps2 = (float)accel_sps2;
    StepperDebug_SetAxisDirection(&stepper_axis_y, direction);
    stepper_axis_y.target_speed_sps = StepperDebug_LimitAxisSpeed(&stepper_axis_y, (float)speed_sps);
  }

  if ((axis_mask & STEPPER_AXIS_MASK_Z) != 0U)
  {
    stepper_axis_z.accel_sps2 = (float)accel_sps2;
    StepperDebug_SetAxisDirection(&stepper_axis_z, direction);
    stepper_axis_z.target_speed_sps = StepperDebug_LimitAxisSpeed(&stepper_axis_z, (float)speed_sps);
  }
}

static void Stepper_StopAxes(uint8_t axis_mask)
{
  StepperDebug_EnterManualMode();

  if ((axis_mask & STEPPER_AXIS_MASK_Y) != 0U)
  {
    stepper_axis_y.target_speed_sps = 0.0f;
  }

  if ((axis_mask & STEPPER_AXIS_MASK_Z) != 0U)
  {
    stepper_axis_z.target_speed_sps = 0.0f;
  }
}

static void Stepper_EmergencyStopAllAxes(void)
{
  uint32_t axis_index;

  StepperDebug_EnterManualMode();
  for (axis_index = 0U; axis_index < STEPPER_AXIS_COUNT; axis_index++)
  {
    StepperAxis_t *axis = stepper_axes[axis_index];
    axis->target_speed_sps = 0.0f;
    axis->current_speed_sps = 0.0f;
    axis->tim_output_running = 0U;
    axis->software_step_accumulator = 0.0f;
    StepperDebug_SetAxisStepLow(axis);
  }
}

static void Vacuum_SetOutput(uint8_t pump_on, uint8_t valve_on)
{
  HAL_GPIO_WritePin(AIR_PUMP_EN_GPIO_Port, AIR_PUMP_EN_Pin, (pump_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SOLENOID_EN_GPIO_Port, SOLENOID_EN_Pin, (valve_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Vacuum_Stop(void)
{
  Vacuum_SetOutput(0U, 0U);
}

static void FrictionMotors_SetOutput(uint8_t motor_mask, uint8_t direction, uint8_t pwm_command_percent)
{
  GPIO_PinState dir_a = friction_3650_a.dir_default;
  GPIO_PinState dir_b = friction_3650_b.dir_default;
  uint32_t pwm = Protocol_ClampPwmCommandPercent((uint32_t)pwm_command_percent);

  if (direction == 0U)
  {
    dir_a = FrictionMotor_GetReverseDirection(&friction_3650_a);
    dir_b = FrictionMotor_GetReverseDirection(&friction_3650_b);
  }

  if ((motor_mask & FRICTION_MOTOR_MASK_A) != 0U)
  {
    FrictionMotor_SetOutput(&friction_3650_a, dir_a, pwm);
  }

  if ((motor_mask & FRICTION_MOTOR_MASK_B) != 0U)
  {
    FrictionMotor_SetOutput(&friction_3650_b, dir_b, pwm);
  }
}

static void FrictionMotors_Stop(void)
{
  FrictionMotor_StopOutput(&friction_3650_a);
  FrictionMotor_StopOutput(&friction_3650_b);
}

static void Conveyor_SetOutput(uint8_t direction, uint8_t pwm_command_percent)
{
  GPIO_PinState dir = block_2430_motor.dir_default;

  if (direction == 0U)
  {
    dir = FrictionMotor_GetReverseDirection(&block_2430_motor);
  }

  FrictionMotor_SetOutput(&block_2430_motor, dir, Protocol_ClampPwmCommandPercent((uint32_t)pwm_command_percent));
}

static void Conveyor_Stop(void)
{
  FrictionMotor_StopOutput(&block_2430_motor);
}

static void Protocol_DispatchPacket(uint8_t flags, uint8_t seq, uint8_t module, uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  uint8_t need_ack = 1U;

  if ((flags & PROTOCOL_FLAG_IS_RESPONSE) != 0U)
  {
    return;
  }

  if ((protocol_estop_active != 0U) && (Protocol_ShouldBlockForEstop(module, cmd) != 0U))
  {
    if (need_ack != 0U)
    {
      Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_ESTOP_ACTIVE);
    }
    return;
  }

  switch (module)
  {
    case PROTOCOL_MODULE_SYSTEM:
      if (payload_len != 0U)
      {
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
        }
        return;
      }

      if (cmd == PROTOCOL_CMD_PING)
      {
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_ESTOP)
      {
        Protocol_ApplyEmergencyStop();
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_CLEAR_ESTOP)
      {
        Protocol_ClearEmergencyStop();
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (need_ack != 0U)
      {
        Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_UNSUPPORTED);
      }
      break;

    case PROTOCOL_MODULE_CHASSIS:
      if (cmd == PROTOCOL_CMD_SET_VELOCITY)
      {
        int16_t vx_mm_s;
        int16_t vy_mm_s;
        int16_t wz_dps_x10;
        uint16_t timeout_ms;

        if (payload_len != 8U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        vx_mm_s = Protocol_ReadS16Le(&payload[0]);
        vy_mm_s = Protocol_ReadS16Le(&payload[2]);
        wz_dps_x10 = Protocol_ReadS16Le(&payload[4]);
        timeout_ms = Protocol_ReadU16Le(&payload[6]);

        if ((vx_mm_s > CHASSIS_PROTOCOL_MAX_LINEAR_MM_S) || (vx_mm_s < (-CHASSIS_PROTOCOL_MAX_LINEAR_MM_S)) ||
            (vy_mm_s > CHASSIS_PROTOCOL_MAX_LINEAR_MM_S) || (vy_mm_s < (-CHASSIS_PROTOCOL_MAX_LINEAR_MM_S)) ||
            (wz_dps_x10 > CHASSIS_PROTOCOL_MAX_WZ_DPS_X10) || (wz_dps_x10 < (-CHASSIS_PROTOCOL_MAX_WZ_DPS_X10)) ||
            (timeout_ms == 0U))
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        Chassis3650_ApplyVelocity(vx_mm_s, vy_mm_s, wz_dps_x10);
        chassis_timeout_active = 1U;
        chassis_timeout_deadline_tick = HAL_GetTick() + timeout_ms;

        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_SET_CONTROL_MODE)
      {
        if (payload_len != 1U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        if (payload[0] > (uint8_t)CHASSIS_CONTROL_MODE_CLOSED_LOOP)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        Chassis3650_SetControlMode((ChassisControlMode_t)payload[0]);
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_SET_WHEEL_TARGET)
      {
        int16_t fl_target_rpm;
        int16_t fr_target_rpm;
        int16_t rl_target_rpm;
        int16_t rr_target_rpm;
        uint16_t timeout_ms;

        if (payload_len != 10U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        fl_target_rpm = Protocol_ReadS16Le(&payload[0]);
        fr_target_rpm = Protocol_ReadS16Le(&payload[2]);
        rl_target_rpm = Protocol_ReadS16Le(&payload[4]);
        rr_target_rpm = Protocol_ReadS16Le(&payload[6]);
        timeout_ms = Protocol_ReadU16Le(&payload[8]);

        if ((timeout_ms == 0U) ||
            (fl_target_rpm > CHASSIS_MAX_TARGET_RPM) || (fl_target_rpm < (-CHASSIS_MAX_TARGET_RPM)) ||
            (fr_target_rpm > CHASSIS_MAX_TARGET_RPM) || (fr_target_rpm < (-CHASSIS_MAX_TARGET_RPM)) ||
            (rl_target_rpm > CHASSIS_MAX_TARGET_RPM) || (rl_target_rpm < (-CHASSIS_MAX_TARGET_RPM)) ||
            (rr_target_rpm > CHASSIS_MAX_TARGET_RPM) || (rr_target_rpm < (-CHASSIS_MAX_TARGET_RPM)))
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        Chassis3650_SetControlMode(CHASSIS_CONTROL_MODE_CLOSED_LOOP);
        Chassis3650_SetWheelTargetsRpm(fl_target_rpm, fr_target_rpm, rl_target_rpm, rr_target_rpm);
        chassis_timeout_active = 1U;
        chassis_timeout_deadline_tick = HAL_GetTick() + timeout_ms;

        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_SET_PID)
      {
        uint8_t wheel_mask;
        uint16_t kp_x1000;
        uint16_t ki_x1000;
        uint16_t kd_x1000;

        if (payload_len != 7U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        wheel_mask = payload[0] & CHASSIS_WHEEL_MASK_ALL;
        kp_x1000 = Protocol_ReadU16Le(&payload[1]);
        ki_x1000 = Protocol_ReadU16Le(&payload[3]);
        kd_x1000 = Protocol_ReadU16Le(&payload[5]);

        if ((wheel_mask == 0U) || (kp_x1000 > 5000U) || (ki_x1000 > 5000U) || (kd_x1000 > 5000U))
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        Chassis3650_SetPidByMask(wheel_mask, kp_x1000, ki_x1000, kd_x1000);
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_SET_WHEEL_TRIM)
      {
        uint8_t trim_mode;
        int16_t fl_value;
        int16_t fr_value;
        int16_t rl_value;
        int16_t rr_value;

        if (payload_len != 9U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        trim_mode = payload[0];
        fl_value = Protocol_ReadS16Le(&payload[1]);
        fr_value = Protocol_ReadS16Le(&payload[3]);
        rl_value = Protocol_ReadS16Le(&payload[5]);
        rr_value = Protocol_ReadS16Le(&payload[7]);

        if (trim_mode > CHASSIS_TRIM_MODE_OFFSET)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        if (trim_mode == CHASSIS_TRIM_MODE_OFFSET)
        {
          if ((fl_value < CHASSIS_TRIM_OFFSET_MIN) || (fl_value > CHASSIS_TRIM_OFFSET_MAX) ||
              (fr_value < CHASSIS_TRIM_OFFSET_MIN) || (fr_value > CHASSIS_TRIM_OFFSET_MAX) ||
              (rl_value < CHASSIS_TRIM_OFFSET_MIN) || (rl_value > CHASSIS_TRIM_OFFSET_MAX) ||
              (rr_value < CHASSIS_TRIM_OFFSET_MIN) || (rr_value > CHASSIS_TRIM_OFFSET_MAX))
          {
            if (need_ack != 0U)
            {
              Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
            }
            return;
          }
        }
        else
        {
          if ((fl_value < 0) || (fl_value > (int16_t)CHASSIS_TRIM_SCALE_X100_MAX) ||
              (fr_value < 0) || (fr_value > (int16_t)CHASSIS_TRIM_SCALE_X100_MAX) ||
              (rl_value < 0) || (rl_value > (int16_t)CHASSIS_TRIM_SCALE_X100_MAX) ||
              (rr_value < 0) || (rr_value > (int16_t)CHASSIS_TRIM_SCALE_X100_MAX))
          {
            if (need_ack != 0U)
            {
              Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
            }
            return;
          }
        }

        Chassis3650_SetWheelTrim(trim_mode, fl_value, fr_value, rl_value, rr_value);
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_STOP)
      {
        if (payload_len != 0U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        Chassis3650_StopAllWheels();
        chassis_timeout_active = 0U;
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (need_ack != 0U)
      {
        Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_UNSUPPORTED);
      }
      break;

    case PROTOCOL_MODULE_STEPPER:
      if (cmd == PROTOCOL_CMD_JOG)
      {
        uint8_t axis_mask;
        uint8_t direction;
        uint16_t speed_sps;
        uint16_t accel_sps2;

        if (payload_len != 6U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        axis_mask = payload[0] & STEPPER_AXIS_MASK_ALL;
        direction = payload[1];
        speed_sps = Protocol_ReadU16Le(&payload[2]);
        accel_sps2 = Protocol_ReadU16Le(&payload[4]);

        if ((axis_mask == 0U) || (direction > 1U) || (speed_sps == 0U) || (accel_sps2 == 0U))
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        Stepper_SetJog(axis_mask, direction, speed_sps, accel_sps2);
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_STOP)
      {
        uint8_t axis_mask;

        if (payload_len != 1U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        axis_mask = payload[0] & STEPPER_AXIS_MASK_ALL;
        if (axis_mask == 0U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        Stepper_StopAxes(axis_mask);
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (need_ack != 0U)
      {
        Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_UNSUPPORTED);
      }
      break;

    case PROTOCOL_MODULE_SERVO:
      if (cmd == PROTOCOL_CMD_SET_ANGLE)
      {
        HardwareServo_t *servo;
        int16_t angle_deg_x10;

        if (payload_len != 5U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        servo = Servo_GetById(payload[0]);
        angle_deg_x10 = Protocol_ReadS16Le(&payload[1]);
        if (servo == NULL)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        Servo_SetAngleDegrees(servo, Servo_DeciDegreesToDegrees(angle_deg_x10));
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (need_ack != 0U)
      {
        Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_UNSUPPORTED);
      }
      break;

    case PROTOCOL_MODULE_VACUUM:
      if (cmd == PROTOCOL_CMD_SET_OUTPUT)
      {
        uint8_t pump_on;
        uint8_t valve_on;
        uint16_t timeout_ms;

        if (payload_len != 4U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        pump_on = payload[0];
        valve_on = payload[1];
        timeout_ms = Protocol_ReadU16Le(&payload[2]);

        if ((pump_on > 1U) || (valve_on > 1U))
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        Vacuum_SetOutput(pump_on, valve_on);
        if (timeout_ms > 0U)
        {
          vacuum_timeout_active = 1U;
          vacuum_timeout_deadline_tick = HAL_GetTick() + timeout_ms;
        }
        else
        {
          vacuum_timeout_active = 0U;
        }

        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_STOP)
      {
        if (payload_len != 0U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        Vacuum_Stop();
        vacuum_timeout_active = 0U;
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (need_ack != 0U)
      {
        Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_UNSUPPORTED);
      }
      break;

    case PROTOCOL_MODULE_FRICTION:
      if (cmd == PROTOCOL_CMD_SET_OUTPUT)
      {
        uint8_t motor_mask;
        uint8_t direction;
        uint8_t pwm_command_percent;

        if (payload_len != 3U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        motor_mask = payload[0] & FRICTION_MOTOR_MASK_ALL;
        direction = payload[1];
        pwm_command_percent = payload[2];

        if ((motor_mask == 0U) || (direction > 1U) || (pwm_command_percent > 100U))
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        FrictionMotors_SetOutput(motor_mask, direction, pwm_command_percent);
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_STOP)
      {
        if (payload_len != 0U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        FrictionMotors_Stop();
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (need_ack != 0U)
      {
        Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_UNSUPPORTED);
      }
      break;

    case PROTOCOL_MODULE_CONVEYOR:
      if (cmd == PROTOCOL_CMD_SET_OUTPUT)
      {
        uint8_t direction;
        uint8_t pwm_command_percent;
        uint16_t timeout_ms;

        if (payload_len != 4U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        direction = payload[0];
        pwm_command_percent = payload[1];
        timeout_ms = Protocol_ReadU16Le(&payload[2]);

        if ((direction > 1U) || (pwm_command_percent > 100U))
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_PARAM);
          }
          return;
        }

        Conveyor_SetOutput(direction, pwm_command_percent);
        if (timeout_ms > 0U)
        {
          conveyor_timeout_active = 1U;
          conveyor_timeout_deadline_tick = HAL_GetTick() + timeout_ms;
        }
        else
        {
          conveyor_timeout_active = 0U;
        }

        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (cmd == PROTOCOL_CMD_STOP)
      {
        if (payload_len != 0U)
        {
          if (need_ack != 0U)
          {
            Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_BAD_LEN);
          }
          return;
        }

        Conveyor_Stop();
        conveyor_timeout_active = 0U;
        if (need_ack != 0U)
        {
          Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_OK);
        }
      }
      else if (need_ack != 0U)
      {
        Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_UNSUPPORTED);
      }
      break;

    default:
      if (need_ack != 0U)
      {
        Protocol_SendResultResponse(seq, module, cmd, PROTOCOL_RESULT_UNSUPPORTED);
      }
      break;
  }
}

static void Protocol_ApplyEmergencyStop(void)
{
  protocol_estop_active = 1U;
  chassis_timeout_active = 0U;
  vacuum_timeout_active = 0U;
  conveyor_timeout_active = 0U;

  Chassis3650_StopAllWheels();
  Stepper_EmergencyStopAllAxes();
  FrictionMotors_Stop();
  Conveyor_Stop();
  Vacuum_Stop();
}

static void Protocol_ClearEmergencyStop(void)
{
  protocol_estop_active = 0U;
}

static void Protocol_ServiceTimeouts(void)
{
  uint32_t now_tick = HAL_GetTick();

  if ((chassis_timeout_active != 0U) && (Protocol_IsTickExpired(now_tick, chassis_timeout_deadline_tick) != 0U))
  {
    chassis_timeout_active = 0U;
    Chassis3650_StopAllWheels();
  }

  if ((vacuum_timeout_active != 0U) && (Protocol_IsTickExpired(now_tick, vacuum_timeout_deadline_tick) != 0U))
  {
    vacuum_timeout_active = 0U;
    Vacuum_Stop();
  }

  if ((conveyor_timeout_active != 0U) && (Protocol_IsTickExpired(now_tick, conveyor_timeout_deadline_tick) != 0U))
  {
    conveyor_timeout_active = 0U;
    Conveyor_Stop();
  }
}

static void Protocol_ReportChassisRpmIfReady(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint32_t elapsed_ms = now_tick - chassis_rpm_report_last_tick;
  uint32_t pulse_snapshot[CHASSIS_WHEEL_COUNT];
  uint32_t pulse_delta[CHASSIS_WHEEL_COUNT];
  uint32_t wheel_index;
  uint8_t payload[8];
  uint8_t closed_loop_payload[22];
  uint8_t status_flags = 0U;

  if (chassis_rpm_report_last_tick == 0U)
  {
    chassis_rpm_report_last_tick = now_tick;
    return;
  }

  if (elapsed_ms < CHASSIS_RPM_REPORT_PERIOD_MS)
  {
    return;
  }

  __disable_irq();
  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    pulse_snapshot[wheel_index] = chassis_3650_motors[wheel_index].fg_pulse_count;
  }
  __enable_irq();

  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    uint32_t rpm_value = 0U;

    pulse_delta[wheel_index] = pulse_snapshot[wheel_index] - chassis_3650_motors[wheel_index].last_report_pulse_count;
    chassis_3650_motors[wheel_index].last_report_pulse_count = pulse_snapshot[wheel_index];

    if (elapsed_ms > 0U)
    {
      rpm_value = (pulse_delta[wheel_index] * 60000U) /
                  (chassis_3650_motors[wheel_index].fg_pulses_per_rev * elapsed_ms);
    }

    chassis_3650_motors[wheel_index].rpm = rpm_value;
    if (rpm_value > 65535U)
    {
      rpm_value = 65535U;
    }

    payload[(wheel_index * 2U)] = (uint8_t)(rpm_value & 0xFFU);
    payload[(wheel_index * 2U) + 1U] = (uint8_t)((rpm_value >> 8U) & 0xFFU);
  }

  chassis_rpm_report_last_tick = now_tick;
  Protocol_SendFrame(PROTOCOL_FLAG_IS_EVENT,
                     0U,
                     PROTOCOL_MODULE_CHASSIS,
                     PROTOCOL_CMD_CHASSIS_RPM_REPORT,
                     payload,
                     sizeof(payload));

  if (chassis_timeout_active != 0U)
  {
    status_flags |= CHASSIS_STATUS_FLAG_TIMEOUT_ACTIVE;
  }
  if (protocol_estop_active != 0U)
  {
    status_flags |= CHASSIS_STATUS_FLAG_ESTOP_ACTIVE;
  }

  for (wheel_index = 0U; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
  {
    int16_t target_rpm = chassis_target_rpm[wheel_index];
    int16_t actual_rpm = chassis_actual_rpm_signed[wheel_index];
    uint32_t base = wheel_index * 2U;

    closed_loop_payload[base] = (uint8_t)(target_rpm & 0xFF);
    closed_loop_payload[base + 1U] = (uint8_t)((target_rpm >> 8) & 0xFF);
    closed_loop_payload[8U + base] = (uint8_t)(actual_rpm & 0xFF);
    closed_loop_payload[8U + base + 1U] = (uint8_t)((actual_rpm >> 8) & 0xFF);
    closed_loop_payload[16U + wheel_index] = (uint8_t)chassis_3650_motors[wheel_index].pwm_command_percent;
  }

  closed_loop_payload[20] = (uint8_t)chassis_control_mode;
  closed_loop_payload[21] = status_flags;
  Protocol_SendFrame(PROTOCOL_FLAG_IS_EVENT,
                     0U,
                     PROTOCOL_MODULE_CHASSIS,
                     PROTOCOL_CMD_CHASSIS_CLOSED_LOOP_REPORT,
                     closed_loop_payload,
                     sizeof(closed_loop_payload));
}

static void Protocol_InitRuntime(void)
{
  Protocol_ClearParser(&uart1_protocol_parser);
  protocol_estop_active = 0U;
  chassis_timeout_active = 0U;
  vacuum_timeout_active = 0U;
  conveyor_timeout_active = 0U;
  chassis_rpm_report_last_tick = 0U;
  chassis_control_last_tick = 0U;
  chassis_control_mode = CHASSIS_CONTROL_MODE_OPEN_LOOP;
  Chassis3650_SetWheelTrim(CHASSIS_TRIM_MODE_SCALE,
                           CHASSIS_TRIM_SCALE_X100_DEFAULT,
                           CHASSIS_TRIM_SCALE_X100_DEFAULT,
                           CHASSIS_TRIM_SCALE_X100_DEFAULT,
                           CHASSIS_TRIM_SCALE_X100_DEFAULT);
  Chassis3650_ClearClosedLoopState();
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
    Protocol_ProcessRxByte(rx_byte);
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
  Chassis3650_StopAllWheels();
  FrictionMotors_Stop();
  FrictionMotor_InitStopped(&block_2430_motor);
  Vacuum_Stop();
  Servo_Init(&servo_elbow);
  Servo_Init(&servo_block);
  Servo_Init(&servo_gripper);
  StepperDebug_Init();
  Protocol_InitRuntime();
  BeltDrive_StartUartReceive();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Uart1_ProcessPendingRxCommands();
    StepperDebug_Run();
    Protocol_ServiceTimeouts();
    Chassis3650_ServiceControlLoop();
    Protocol_ReportChassisRpmIfReady();
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
  else if (GPIO_Pin == BLOCK_2430_FG_Pin)
  {
    block_2430_motor.fg_pulse_count++;
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

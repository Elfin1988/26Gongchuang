# 26Gongchuang

## 当前状态

本工程基于 `STM32H723 + HAL`，目前已经完成：

- 两路 `3650` 电机的 PWM + 方向 + FG 反馈控制
- `USART1` 单字节指令控制两路 `3650`
- 每 `1s` 通过串口上报两路 `3650` 的转速
- 四路底盘 `3650` 电机的 PWM + 方向 + FG 反馈接口

主控逻辑在 `Core/Src/main.c`，`.ioc` 与代码已同步。

## 已启用功能

### 摩擦带双 3650 联动

当前 `USART1` 接收以下指令：

| 指令 | 含义 | 3650_A | 3650_B |
| --- | --- | --- | --- |
| `0` | 停转 | 停 PWM | 停 PWM |
| `1` | 低速一档 | 反转低速 | 正转低速 |
| `2` | 高速一档 | 反转高速 | 正转高速 |

说明：

- “正转/反转” 是按传送带整体运动方向定义的。
- 当前代码里 `0` 指令会直接关闭两路 PWM 输出。
- 当前低速/高速占空比分别为 `16% / 30%`。

### 转速上报

每隔 `1000 ms` 计算一次两路 `3650` 的转速，并通过 `USART1_TX` 发送：

```text
3650_a_rpm=1234,3650_b_rpm=1180
```

当前计算公式：

```text
rpm = pulse_delta * 60000 / (6 * elapsed_ms)
```

含义如下：

| 参数 | 说明 |
| --- | --- |
| `pulse_delta` | 这一统计周期内新增的 FG 脉冲数 |
| `60000` | 把毫秒换算成分钟 |
| `6` | 当前电机每转一圈输出 6 个 FG 脉冲 |
| `elapsed_ms` | 实际统计周期，单位 ms |

## 已封装但未启用的函数

### 控制丝杠上的步进电机

函数：

```c
static void Stepper_UartControlPlaceholder(uint8_t rx_byte);
```

作用：

- 预留给上位机串口控制步进电机
- 规划了“低速 / 高速 + 正转 / 反转”四种指令
- 当前只是占位，主流程没有调用，也没有真正输出步进脉冲

预留指令表：

| 指令 | 方向 | 速度 |
| --- | --- | --- |
| `1` | 正转 | `3 rps` |
| `2` | 正转 | `6 rps` |
| `3` | 反转 | `3 rps` |
| `4` | 反转 | `6 rps` |

### 控制摩擦带上的 3650 电机

函数：

```c
static void Friction3650A_UartControlPlaceholder(uint8_t rx_byte);
static void Friction3650BMotor_UartControlPlaceholder(uint8_t rx_byte);
```

作用：

- 预留给上位机分别单独控制 `3650_A`、`3650_B`
- 每路都支持“两档速度 + 正反转”四种动作
- 当前只是封装保留，主流程没有调用

两个摩擦带 3650 电机的排针布局：

| 电机 | PWM | DIR | FG |
| --- | --- | --- | --- |
| `FRICTION_3650_A` | `PF11 / TIM24_CH1` | `PF13` | `PF15 / EXTI15` |
| `FRICTION_3650_B` | `PF12 / TIM24_CH2` | `PG0` | `PF14 / EXTI14` |

物理位置：上排中间靠右，第 14-16 列；外侧一排是 `PF12 / PF14 / PG0`，内侧一排是 `PF11 / PF13 / PF15`。

### 控制底盘上的 3650 电机

函数：

```c
static void Chassis3650_SetWheelOutput(ChassisWheel_t wheel, ChassisDirection_t direction, uint32_t duty_percent);
static void Chassis3650_StopAllWheels(void);
```

作用：

- 提供四路底盘 3650 的统一控制接口
- 支持单轮 `正转 / 反转 / 停转`
- 速度通过对应 PWM 占空比控制，FG 反馈通过 EXTI 计数
- 当前主流程未启用，仅完成底层封装和引脚配置

控制规则：

| 状态 | DIR | PWM |
| --- | --- | --- |
| 正转 | `dir_default` | 输出占空比 |
| 反转 | `!dir_default` | 输出占空比 |
| 停转 | 保持方向 | 关闭该路 PWM |

四个轮子的排针布局：

| 枚举 | 轮子 | PWM | DIR | FG |
| --- | --- | --- | --- | --- |
| `CHASSIS_WHEEL_FRONT_LEFT` | 前左轮 | `PA6 / TIM3_CH1` | `PC4` | `PB0 / EXTI0` |
| `CHASSIS_WHEEL_FRONT_RIGHT` | 前右轮 | `PA3 / TIM2_CH4` | `PA7` | `PC5 / EXTI5` |
| `CHASSIS_WHEEL_REAR_LEFT` | 后左轮 | `PB8 / TIM4_CH3` | `PE0` | `PB7 / EXTI7` |
| `CHASSIS_WHEEL_REAR_RIGHT` | 后右轮 | `PB9 / TIM4_CH4` | `PE2` | `PE1 / EXTI1` |

## 串口接口

| 项目 | 配置 |
| --- | --- |
| 外设 | `USART1` |
| 波特率 | `115200` |
| 格式 | `8N1` |
| TX | `PB14` |
| RX | `PB15` |

## 当前引脚使用情况

| 引脚 | 名称 | 方向 | 当前用途 |
| --- | --- | --- | --- |
| `PA3` | `CHASSIS_3650_FR_PWM / TIM2_CH4` | 输出 | 前右轮 PWM |
| `PA2` | `SERVO_ELBOW_PWM / TIM15_CH1` | 输出 | 肘击舵机 PWM |
| `PA6` | `CHASSIS_3650_FL_PWM / TIM3_CH1` | 输出 | 前左轮 PWM |
| `PA7` | `CHASSIS_3650_FR_DIR` | 输出 | 前右轮方向 |
| `PA8` | `Y_STEP` | 输出 | Y 导轨 STEP 软件脉冲 |
| `PB0` | `CHASSIS_3650_FL_FG / EXTI0` | 输入 | 前左轮转速反馈 |
| `PB7` | `CHASSIS_3650_RL_FG / EXTI7` | 输入 | 后左轮转速反馈 |
| `PB8` | `CHASSIS_3650_RL_PWM / TIM4_CH3` | 输出 | 后左轮 PWM |
| `PB9` | `CHASSIS_3650_RR_PWM / TIM4_CH4` | 输出 | 后右轮 PWM |
| `PC4` | `CHASSIS_3650_FL_DIR` | 输出 | 前左轮方向 |
| `PC5` | `CHASSIS_3650_FR_FG / EXTI5` | 输入 | 前右轮转速反馈 |
| `PC7` | `SERVO_GRIPPER_PWM / TIM8_CH2` | 输出 | 夹爪舵机 PWM |
| `PC8` | `Z_STEP` | 输出 | Z 导轨 STEP 软件脉冲 |
| `PC9` | `Z_DIR` | 输出 | Z 导轨方向 |
| `PC11` | `Y_DIR` | 输出 | Y 导轨方向 |
| `PB14` | `USART1_TX` | 输出 | 串口发送 |
| `PB15` | `USART1_RX` | 输入 | 串口接收 |
| `PE0` | `CHASSIS_3650_RL_DIR` | 输出 | 后左轮方向 |
| `PE1` | `CHASSIS_3650_RR_FG / EXTI1` | 输入 | 后右轮转速反馈 |
| `PE2` | `CHASSIS_3650_RR_DIR` | 输出 | 后右轮方向 |
| `PF11` | `FRICTION_3650_A_PWM / TIM24_CH1` | 输出 | 3650_A PWM |
| `PF12` | `FRICTION_3650_B_PWM / TIM24_CH2` | 输出 | 3650_B PWM |
| `PF13` | `FRICTION_3650_A_DIR` | 输出 | 3650_A 方向 |
| `PF8` | `SERVO_BLOCK_PWM / TIM13_CH1` | 输出 | 运送物块舵机 PWM |
| `PF14` | `FRICTION_3650_B_FG / EXTI14` | 输入 | 3650_B 转速反馈 |
| `PF15` | `FRICTION_3650_A_FG / EXTI15` | 输入 | 3650_A 转速反馈 |
| `PG0` | `FRICTION_3650_B_DIR` | 输出 | 3650_B 方向 |
| `PG13` | `AIR_PUMP_EN` | 输出 | 气泵 EN 开关，默认低电平 |
| `PG15` | `SOLENOID_EN` | 输出 | 电磁阀 EN 开关，默认低电平 |

## Air Switch Outputs

- Pins: `PG15 -> SOLENOID_EN`, `PG13 -> AIR_PUMP_EN`
- Location: bottom outer header middle, columns 15-16
- Logic: output high enables the external switch module, output low disables it

## 相关文件

| 文件 | 说明 |
| --- | --- |
| `Core/Src/main.c` | 主控制逻辑、串口指令、RPM 计算、底盘 3650 封装 |
| `Core/Inc/main.h` | 所有引脚宏定义 |
| `Core/Src/gpio.c` | GPIO 初始化 |
| `Core/Src/tim.c` | `TIM2 / TIM3 / TIM4 / TIM8 / TIM13 / TIM15 / TIM24` PWM 初始化 |
| `Core/Src/usart.c` | `USART1` 初始化 |
| `26Gongchuang.ioc` | CubeMX 引脚与外设配置源文件 |

## Servo PWM

- Elbow: `PA2 -> SERVO_ELBOW_PWM / TIM15_CH1`
- Block transfer: `PF8 -> SERVO_BLOCK_PWM / TIM13_CH1`
- Gripper: `PC7 -> SERVO_GRIPPER_PWM / TIM8_CH2`
- PWM: hardware timers, `20 ms` frame, `500-2500 us` pulse width
- USART1 commands:
- `I`: move servo to `0 deg`
- `J`: move servo to `+135 deg`
- `K`: move servo to `-135 deg`

## Stepper YZ

- Pins: `PA8 -> Y_STEP`, `PC11 -> Y_DIR`, `PC8 -> Z_STEP`, `PC9 -> Z_DIR`
- STEP output: software GPIO pulse driven by the existing `TIM1` 10kHz update interrupt
- USART1 RX bytes are queued in the interrupt and processed in the main loop.
- Y/Z stepper speed changes keep the existing acceleration ramp and do not jump instantly.
- Commands:
- `0`: stop Y and Z with deceleration
- `3`: Y forward
- `4`: Y reverse
- `5`: Z forward
- `6`: Z reverse

# 26Gongchuang

## 当前状态

本工程基于 `STM32H723 + HAL`，目前已经完成：

- 两路 `3650` 电机的 PWM + 方向 + FG 反馈控制
- `USART1` 单字节指令控制两路 `3650`
- 每 `1s` 通过串口上报两路 `3650` 的转速
- 四路底盘麦轮接口的 GPIO / PWM 资源预留

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

### 控制底盘上的麦轮

函数：

```c
static void Mecanum_SetWheelOutput(MecanumWheel_t wheel, MecanumDirection_t direction, uint32_t duty_percent);
static void Mecanum_StopAllWheels(void);
```

作用：

- 提供四路麦轮的统一控制接口
- 支持单轮 `正转 / 反转 / 停转`
- 速度通过对应 PWM 占空比控制
- 当前主流程未启用，仅完成底层封装和引脚配置

控制规则：

| 状态 | IN1 | IN2 | PWM |
| --- | --- | --- | --- |
| 正转 | 高 | 低 | 输出占空比 |
| 反转 | 低 | 高 | 输出占空比 |
| 停转 | 低 | 低 | 关闭该路 PWM |

四个轮子的枚举对应关系：

| 枚举 | 轮子 | PWM 通道 |
| --- | --- | --- |
| `MECANUM_WHEEL_A` | A 轮 | `TIM3_CH1 / PA6` |
| `MECANUM_WHEEL_B` | B 轮 | `TIM3_CH2 / PA7` |
| `MECANUM_WHEEL_C` | C 轮 | `TIM3_CH3 / PB0` |
| `MECANUM_WHEEL_D` | D 轮 | `TIM3_CH4 / PB1` |

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
| `PA0` | `FRICTION_3650_A_PWM / TIM2_CH1` | 输出 | 3650_A PWM |
| `PA4` | `X_STEP` | 输出 | 丝杠步进电机 STEP（预留） |
| `PA5` | `X_DIR` | 输出 | 丝杠步进电机 DIR（预留） |
| `PA6` | `PWMA / TIM3_CH1` | 输出 | 麦轮 A 路 PWM |
| `PA7` | `PWMB / TIM3_CH2` | 输出 | 麦轮 B 路 PWM |
| `PB0` | `PWMC / TIM3_CH3` | 输出 | 麦轮 C 路 PWM |
| `PB1` | `PWMD / TIM3_CH4` | 输出 | 麦轮 D 路 PWM |
| `PB14` | `USART1_TX` | 输出 | 串口发送 |
| `PB15` | `USART1_RX` | 输入 | 串口接收 |
| `PD1` | `FRICTION_3650_B_DIR` | 输出 | 3650_B 方向 |
| `PD2` | `FRICTION_3650_B_FG / EXTI2` | 输入 | 3650_B 转速反馈 |
| `PD3` | `FRICTION_3650_A_DIR` | 输出 | 3650_A 方向 |
| `PD4` | `FRICTION_3650_A_FG / EXTI4` | 输入 | 3650_A 转速反馈 |
| `PD12` | `FRICTION_3650_B_PWM / TIM4_CH1` | 输出 | 3650_B PWM |
| `PE0` | `AIN1` | 输出 | 麦轮 A 路方向 1 |
| `PE1` | `AIN2` | 输出 | 麦轮 A 路方向 2 |
| `PE2` | `BIN1` | 输出 | 麦轮 B 路方向 1 |
| `PE3` | `BIN2` | 输出 | 麦轮 B 路方向 2 |
| `PE4` | `CIN1` | 输出 | 麦轮 C 路方向 1 |
| `PE5` | `CIN2` | 输出 | 麦轮 C 路方向 2 |
| `PE6` | `DIN1` | 输出 | 麦轮 D 路方向 1 |
| `PE7` | `DIN2` | 输出 | 麦轮 D 路方向 2 |

## 相关文件

| 文件 | 说明 |
| --- | --- |
| `Core/Src/main.c` | 主控制逻辑、串口指令、RPM 计算、麦轮封装 |
| `Core/Inc/main.h` | 所有引脚宏定义 |
| `Core/Src/gpio.c` | GPIO 初始化 |
| `Core/Src/tim.c` | `TIM2 / TIM3 / TIM4` PWM 初始化 |
| `Core/Src/usart.c` | `USART1` 初始化 |
| `26Gongchuang.ioc` | CubeMX 引脚与外设配置源文件 |

# 26Gongchuang

## 当前代码

当前程序复位后会直接运行 PWM 电机测速测试逻辑，不需要上位机发送命令。

主循环现在做两件事：

- 每 `3 秒` 在摩擦带上的 3650 电机两档 PWM 占空比之间切换一次，用于观察两种转速下的 FG 反馈。
- 每 `1 秒` 统计一次 3650 电机的 FG 脉冲数，换算成 `rpm` 后通过 `USART1_TX` 发给上位机。

当前正在运行的是 3650 电机测试速度参数：

- `FRICTION_3650_PWM_DUTY_LOW_PERCENT = 30`
- `FRICTION_3650_PWM_DUTY_HIGH_PERCENT = 60`
- `FRICTION_3650_PWM_SWITCH_PERIOD_MS = 3000`

串口上报格式：

```text
3650_rpm=1234\r\n
```

## 控制丝杠上的步进电机

目前代码里保留了一个“封装好但没有启动”的 USART 命令控制入口。它只是预留接口，当前主循环没有调用，所以不会影响现在的 3 秒自动切速测试。

```c
static void Stepper_UartControlPlaceholder(uint8_t rx_byte);
```

作用：后续接收上位机给 USART 的命令字节后，用它预留步进电机的两档速度和正反转控制。

当前状态：未启用，函数内部只解析命令并留下 `TODO`，还没有接回真实步进电机控制逻辑。

预留指令：

- `1`：步进电机正转低速，参数 `STEPPER_UART_LOW_RPS = 3`
- `2`：步进电机正转高速，参数 `STEPPER_UART_HIGH_RPS = 6`
- `3`：步进电机反转低速，参数 `STEPPER_UART_LOW_RPS = 3`
- `4`：步进电机反转高速，参数 `STEPPER_UART_HIGH_RPS = 6`

以后用法：

```c
Stepper_UartControlPlaceholder(rx_byte);
```

## 控制摩擦带上的3650电机

目前代码里也保留了一个“封装好但没有启动”的 USART 命令控制入口。它同样只是预留接口，当前主循环没有调用，所以不会影响现在的 3 秒自动切速测试。

```c
static void PwmMotor_UartControlPlaceholder(uint8_t rx_byte);
```

作用：后续接收上位机给 USART 的命令字节后，用它控制摩擦带上的 3650 电机，也就是 `PA0 / TIM2_CH1` 的 PWM 占空比和 `PD3` 方向引脚。

当前状态：未启用，函数已经能根据命令设置方向和 PWM 占空比，但当前主循环没有调用它。

预留指令：

- `A`：3650 电机正转低速，占空比 `FRICTION_3650_PWM_DUTY_LOW_PERCENT = 30`
- `B`：3650 电机正转高速，占空比 `FRICTION_3650_PWM_DUTY_HIGH_PERCENT = 60`
- `C`：3650 电机反转低速，占空比 `FRICTION_3650_PWM_DUTY_LOW_PERCENT = 30`
- `D`：3650 电机反转高速，占空比 `FRICTION_3650_PWM_DUTY_HIGH_PERCENT = 60`

以后用法：

```c
PwmMotor_UartControlPlaceholder(rx_byte);
```

## 控制存储模块上的2430电机

目前代码里又增加了一套“封装好但没有启动”的 USART 命令控制入口。它也是预留接口，当前主循环没有调用，但底层 GPIO / PWM / FG 输入已经配置好。

```c
static void StorageMotor2430_UartControlPlaceholder(uint8_t rx_byte);
```

作用：后续接收上位机给 USART 的命令字节后，用它控制存储模块上的 2430 电机，实现两档速度和正反转。对应硬件资源是 `PA6 / TIM3_CH1`、`PA7`、`PA8`。

当前状态：未启用；函数已经能解析命令，并会设置方向脚和 PWM 比较值，但当前主循环没有调用它，也没有把 2430 电机的测速上报放进运行流程。

预留指令：

- `E`：2430 电机正转低速，占空比 `STORAGE_2430_DUTY_LOW_PERCENT = 30`
- `F`：2430 电机正转高速，占空比 `STORAGE_2430_DUTY_HIGH_PERCENT = 60`
- `G`：2430 电机反转低速，占空比 `STORAGE_2430_DUTY_LOW_PERCENT = 30`
- `H`：2430 电机反转高速，占空比 `STORAGE_2430_DUTY_HIGH_PERCENT = 60`

以后用法：

```c
StorageMotor2430_UartControlPlaceholder(rx_byte);
```

## 当前引脚使用情况

当前已经配置和使用的主要引脚如下：

| 引脚     | 当前名称                     | 方向 | 功能                       | 状态                             |
| -------- | ---------------------------- | ---- | -------------------------- | -------------------------------- |
| `PA0`  | `TIM2_CH1` / `FRICTION_3650_PWM` | 输出 | 摩擦带上的 3650 电机 PWM 调速输出 | 已启用，当前主循环正在使用 |
| `PD3`  | `FRICTION_3650_DIR` | 输出 | 摩擦带上的 3650 电机正反转控制 | 已启用，当前主循环正在使用 |
| `PD4`  | `FRICTION_3650_FG` / `EXTI4` | 输入 | 3650 电机转速反馈；每转 6 脉冲 | 已启用，当前中断计数正在使用 |
| `PA6`  | `TIM3_CH1` / `STORAGE_2430_PWM` | 输出 | 存储模块上的 2430 电机 PWM 调速输出 | 已配置，当前主循环未使用 |
| `PA7`  | `STORAGE_2430_DIR` | 输出 | 存储模块上的 2430 电机正反转控制 | 已配置，当前主循环未使用 |
| `PA8`  | `STORAGE_2430_FG` / `EXTI8` | 输入 | 2430 电机转速反馈；每转 6 脉冲 | 已配置，当前主循环未使用 |
| `PB14` | `USART1_TX`                | 输出 | 串口 1 向上位机发送        | 已启用，当前正在使用             |
| `PB15` | `USART1_RX`                | 输入 | 串口 1 接收上位机命令      | 已配置，当前主循环未启用接收逻辑 |
| `PA4`  | `X_STEP`                   | 输出 | 步进电机X STEP             | 已配置，当前主循环未使用         |
| `PA5`  | `X_DIR`                    | 输出 | 步进电机X DIR              | 已配置，当前主循环未使用         |

## rpm 计算

当前 3650 电机和 2430 电机都按“FG 上升沿计数”计算转速。

公式：

```text
rpm = pulse_delta * 60000 / (6 * elapsed_ms)
```

参数含义：

- `pulse_delta` 是统计周期内的 FG 脉冲数。
- `6` 是电机每转输出的 FG 脉冲数。
- `elapsed_ms` 是本次统计实际经过的毫秒数。

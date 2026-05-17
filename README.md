# 26Gongchuang

## 当前状态

本工程基于 `STM32H723 + HAL`，当前主控逻辑已经切换到 `USART1` 二进制协议通信，不再使用早期的单字节串口控制方案。

目前代码中已经接入：

- 两路摩擦带 `3650` 电机的 PWM + 方向 + FG 反馈控制
- 四路底盘 `3650` 电机的 PWM + 方向 + FG 反馈控制
- `2430` 运送电机的 PWM + 方向 + FG 反馈控制
- `Y / Z` 两轴步进电机的软件脉冲输出与加减速控制
- 三路舵机控制
- 抽气模块控制
- `USART1` 中断接收、协议解析、命令分发、应答与超时保护

主控逻辑位于 [Core/Src/main.c](</d:/Users/Administrator/Documents/26Gongchuang/26Gongchuang/Core/Src/main.c>)，CubeMX 配置在 `26Gongchuang.ioc`。

## 通信

### 串口参数

| 项目 | 配置 |
| --- | --- |
| 外设 | `USART1` |
| 波特率 | `115200` |
| 格式 | `8N1` |
| TX | `PB14` |
| RX | `PB15` |

### 接收链路

当前通信链路为：

1. `USART1` 中断按 `1 byte` 接收。
2. 中断回调里只负责把字节塞入环形队列。
3. 主循环从队列中取字节，逐字节喂给协议解析状态机。
4. 状态机拼出完整帧后进行 `CRC16-MODBUS` 校验。
5. 校验通过后按模块号与命令号分发到具体执行机构。

### 协议帧格式

```text
SOF1 SOF2 FLAGS SEQ MODULE CMD LEN PAYLOAD CRC_L CRC_H
```

固定字段：

- `SOF1 = 0xAA`
- `SOF2 = 0x55`
- `CRC = CRC16-MODBUS`
- 多字节整数采用小端序

更完整的协议说明见  
[通信/上位机下位机通信协议_竞赛最小版.md](</d:/Users/Administrator/Documents/26Gongchuang/26Gongchuang/通信/上位机下位机通信协议_竞赛最小版.md>)

### 当前已实现模块

| 模块 | 编号 | 当前命令 |
| --- | --- | --- |
| `SYSTEM` | `0x00` | `PING` / `ESTOP` / `CLEAR_ESTOP` |
| `CHASSIS` | `0x10` | `SET_VELOCITY` / `STOP` |
| `STEPPER` | `0x20` | `JOG` / `STOP` |
| `SERVO` | `0x30` | `SET_ANGLE` |
| `VACUUM` | `0x40` | `SET_OUTPUT` / `STOP` |
| `FRICTION` | `0x50` | `SET_OUTPUT` / `STOP` |
| `CONVEYOR` | `0x60` | `SET_OUTPUT` / `STOP` |

### 安全特性

- 支持系统级 `ESTOP`
- 底盘支持 `timeout_ms` 超时自动停车
- 抽气模块支持 `timeout_ms` 超时自动关闭
- 运送电机支持 `timeout_ms` 超时自动停止
- 请求帧可通过 `need_ack` 请求应答帧

## 执行机构

### 底盘 3650

- 四轮麦轮底盘通过统一接口控制
- 速度语义由协议层给出，底层映射到 PWM 命令值
- 当前这套驱动链路是“命令值越大，轮子越慢”，`100` 表示停止

轮子定义：

| 枚举 | 轮子 | PWM | DIR | FG |
| --- | --- | --- | --- | --- |
| `CHASSIS_WHEEL_FRONT_LEFT` | 前左轮 | `PA6 / TIM3_CH1` | `PC4` | `PB0 / EXTI0` |
| `CHASSIS_WHEEL_FRONT_RIGHT` | 前右轮 | `PA3 / TIM2_CH4` | `PA7` | `PC5 / EXTI5` |
| `CHASSIS_WHEEL_REAR_LEFT` | 后左轮 | `PB8 / TIM4_CH3` | `PE0` | `PB7 / EXTI7` |
| `CHASSIS_WHEEL_REAR_RIGHT` | 后右轮 | `PB9 / TIM4_CH4` | `PE2` | `PE1 / EXTI1` |

### 摩擦带双 3650

| 电机 | PWM | DIR | FG |
| --- | --- | --- | --- |
| `FRICTION_3650_A` | `PF11 / TIM24_CH1` | `PF13` | `PF15 / EXTI15` |
| `FRICTION_3650_B` | `PF12 / TIM24_CH2` | `PG0` | `PF14 / EXTI14` |

物理位置：上排中间靠右，第 `14-16` 列；外侧一排是 `PF12 / PF14 / PG0`，内侧一排是 `PF11 / PF13 / PF15`。

### 运送电机 2430

- 使用单路 PWM + DIR 控制
- 已接入协议模块 `CONVEYOR`
- 支持超时自动关闭

### 步进 Y/Z

- 引脚：`PA8 -> Y_STEP`，`PC11 -> Y_DIR`，`PC8 -> Z_STEP`，`PC9 -> Z_DIR`
- STEP 脉冲由 `TIM1` 的 `10 kHz` 更新中断驱动软件输出
- 支持加减速，不做瞬时跳速
- 通过协议模块 `STEPPER` 控制
- 轴掩码：`Y = 0x01`，`Z = 0x02`

### 舵机

- Elbow: `PA2 -> SERVO_ELBOW_PWM / TIM15_CH1`
- Block transfer: `PF8 -> SERVO_BLOCK_PWM / TIM13_CH1`
- Gripper: `PC7 -> SERVO_GRIPPER_PWM / TIM8_CH2`
- PWM：硬件定时器输出，`20 ms` 周期，`500-2500 us` 脉宽
- 通过协议模块 `SERVO` 控制
- 舵机编号：
  - `0x01` 肘部舵机
  - `0x02` 运送物块舵机
  - `0x03` 夹爪舵机

### 抽气

- `PG13 -> AIR_PUMP_EN`
- `PG15 -> SOLENOID_EN`
- 输出高电平使能外部开关模块
- 通过协议模块 `VACUUM` 控制

## 当前引脚使用情况

| 引脚 | 名称 | 方向 | 当前用途 |
| --- | --- | --- | --- |
| `PA2` | `SERVO_ELBOW_PWM / TIM15_CH1` | 输出 | 肘击舵机 PWM |
| `PA3` | `CHASSIS_3650_FR_PWM / TIM2_CH4` | 输出 | 前右轮 PWM |
| `PA6` | `CHASSIS_3650_FL_PWM / TIM3_CH1` | 输出 | 前左轮 PWM |
| `PA7` | `CHASSIS_3650_FR_DIR` | 输出 | 前右轮方向 |
| `PA8` | `Y_STEP` | 输出 | Y 导轨 STEP 软件脉冲 |
| `PB0` | `CHASSIS_3650_FL_FG / EXTI0` | 输入 | 前左轮转速反馈 |
| `PB7` | `CHASSIS_3650_RL_FG / EXTI7` | 输入 | 后左轮转速反馈 |
| `PB8` | `CHASSIS_3650_RL_PWM / TIM4_CH3` | 输出 | 后左轮 PWM |
| `PB9` | `CHASSIS_3650_RR_PWM / TIM4_CH4` | 输出 | 后右轮 PWM |
| `PB14` | `USART1_TX` | 输出 | 串口发送 |
| `PB15` | `USART1_RX` | 输入 | 串口接收 |
| `PC4` | `CHASSIS_3650_FL_DIR` | 输出 | 前左轮方向 |
| `PC5` | `CHASSIS_3650_FR_FG / EXTI5` | 输入 | 前右轮转速反馈 |
| `PC7` | `SERVO_GRIPPER_PWM / TIM8_CH2` | 输出 | 夹爪舵机 PWM |
| `PC8` | `Z_STEP` | 输出 | Z 导轨 STEP 软件脉冲 |
| `PC9` | `Z_DIR` | 输出 | Z 导轨方向 |
| `PC11` | `Y_DIR` | 输出 | Y 导轨方向 |
| `PE0` | `CHASSIS_3650_RL_DIR` | 输出 | 后左轮方向 |
| `PE1` | `CHASSIS_3650_RR_FG / EXTI1` | 输入 | 后右轮转速反馈 |
| `PE2` | `CHASSIS_3650_RR_DIR` | 输出 | 后右轮方向 |
| `PF8` | `SERVO_BLOCK_PWM / TIM13_CH1` | 输出 | 运送物块舵机 PWM |
| `PF11` | `FRICTION_3650_A_PWM / TIM24_CH1` | 输出 | 3650_A PWM |
| `PF12` | `FRICTION_3650_B_PWM / TIM24_CH2` | 输出 | 3650_B PWM |
| `PF13` | `FRICTION_3650_A_DIR` | 输出 | 3650_A 方向 |
| `PF14` | `FRICTION_3650_B_FG / EXTI14` | 输入 | 3650_B 转速反馈 |
| `PF15` | `FRICTION_3650_A_FG / EXTI15` | 输入 | 3650_A 转速反馈 |
| `PG0` | `FRICTION_3650_B_DIR` | 输出 | 3650_B 方向 |
| `PG13` | `AIR_PUMP_EN` | 输出 | 气泵 EN 开关，默认低电平 |
| `PG15` | `SOLENOID_EN` | 输出 | 电磁阀 EN 开关，默认低电平 |

## 相关文件

| 文件 | 说明 |
| --- | --- |
| `Core/Src/main.c` | 主控制逻辑、协议解析与分发、执行机构控制 |
| `Core/Inc/main.h` | 所有引脚宏定义 |
| `Core/Src/gpio.c` | GPIO 初始化 |
| `Core/Src/tim.c` | PWM 与定时器初始化 |
| `Core/Src/usart.c` | `USART1` 初始化 |
| `Core/Src/stm32h7xx_it.c` | 中断入口 |
| `26Gongchuang.ioc` | CubeMX 引脚与外设配置 |


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CHASSIS_3650_RR_DIR_Pin GPIO_PIN_2
#define CHASSIS_3650_RR_DIR_GPIO_Port GPIOE
#define SERVO_ELBOW_PWM_Pin GPIO_PIN_2
#define SERVO_ELBOW_PWM_GPIO_Port GPIOA
#define CHASSIS_3650_FR_PWM_Pin GPIO_PIN_3
#define CHASSIS_3650_FR_PWM_GPIO_Port GPIOA
#define CHASSIS_3650_FL_DIR_Pin GPIO_PIN_4
#define CHASSIS_3650_FL_DIR_GPIO_Port GPIOC
#define CHASSIS_3650_FR_FG_Pin GPIO_PIN_5
#define CHASSIS_3650_FR_FG_GPIO_Port GPIOC
#define CHASSIS_3650_FR_FG_EXTI_IRQn EXTI9_5_IRQn
#define CHASSIS_3650_FL_PWM_Pin GPIO_PIN_6
#define CHASSIS_3650_FL_PWM_GPIO_Port GPIOA
#define CHASSIS_3650_FR_DIR_Pin GPIO_PIN_7
#define CHASSIS_3650_FR_DIR_GPIO_Port GPIOA
#define CHASSIS_3650_FL_FG_Pin GPIO_PIN_0
#define CHASSIS_3650_FL_FG_GPIO_Port GPIOB
#define CHASSIS_3650_FL_FG_EXTI_IRQn EXTI0_IRQn
#define CHASSIS_3650_RR_FG_Pin GPIO_PIN_1
#define CHASSIS_3650_RR_FG_GPIO_Port GPIOE
#define CHASSIS_3650_RR_FG_EXTI_IRQn EXTI1_IRQn
#define CHASSIS_3650_RL_FG_Pin GPIO_PIN_7
#define CHASSIS_3650_RL_FG_GPIO_Port GPIOB
#define CHASSIS_3650_RL_FG_EXTI_IRQn EXTI9_5_IRQn
#define CHASSIS_3650_RL_PWM_Pin GPIO_PIN_8
#define CHASSIS_3650_RL_PWM_GPIO_Port GPIOB
#define CHASSIS_3650_RR_PWM_Pin GPIO_PIN_9
#define CHASSIS_3650_RR_PWM_GPIO_Port GPIOB
#define SERVO_GRIPPER_PWM_Pin GPIO_PIN_7
#define SERVO_GRIPPER_PWM_GPIO_Port GPIOC
#define Y_STEP_Pin GPIO_PIN_8
#define Y_STEP_GPIO_Port GPIOA
#define Z_STEP_Pin GPIO_PIN_8
#define Z_STEP_GPIO_Port GPIOC
#define Z_DIR_Pin GPIO_PIN_9
#define Z_DIR_GPIO_Port GPIOC
#define Y_DIR_Pin GPIO_PIN_11
#define Y_DIR_GPIO_Port GPIOC
#define FRICTION_3650_A_PWM_Pin GPIO_PIN_11
#define FRICTION_3650_A_PWM_GPIO_Port GPIOF
#define FRICTION_3650_B_PWM_Pin GPIO_PIN_12
#define FRICTION_3650_B_PWM_GPIO_Port GPIOF
#define FRICTION_3650_A_DIR_Pin GPIO_PIN_13
#define FRICTION_3650_A_DIR_GPIO_Port GPIOF
#define SERVO_BLOCK_PWM_Pin GPIO_PIN_8
#define SERVO_BLOCK_PWM_GPIO_Port GPIOF
#define FRICTION_3650_B_FG_Pin GPIO_PIN_14
#define FRICTION_3650_B_FG_GPIO_Port GPIOF
#define FRICTION_3650_B_FG_EXTI_IRQn EXTI15_10_IRQn
#define FRICTION_3650_A_FG_Pin GPIO_PIN_15
#define FRICTION_3650_A_FG_GPIO_Port GPIOF
#define FRICTION_3650_A_FG_EXTI_IRQn EXTI15_10_IRQn
#define CHASSIS_3650_RL_DIR_Pin GPIO_PIN_0
#define CHASSIS_3650_RL_DIR_GPIO_Port GPIOE
#define FRICTION_3650_B_DIR_Pin GPIO_PIN_0
#define FRICTION_3650_B_DIR_GPIO_Port GPIOG
#define AIR_PUMP_EN_Pin GPIO_PIN_13
#define AIR_PUMP_EN_GPIO_Port GPIOG
#define SOLENOID_EN_Pin GPIO_PIN_15
#define SOLENOID_EN_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

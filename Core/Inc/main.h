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
#include "stm32f4xx_hal.h"

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

/**
 * @brief 当前PBRoBot构建的应用配置。
 *
 * 五个值必须且只能有一个为1。当前选择物理X+厘米路线候选；已实物通过的
 * 四路脉冲档、IMU短回归档和恒速HOLD档保留为回归配置。
 */
#define APP_PROFILE_STEP_TEST             0U
#define APP_PROFILE_IMU_SHORT_REGRESSION  0U
#define APP_PROFILE_CONSTANT_HEADING_HOLD 0U
#define APP_PROFILE_CM_ROUTE_X_POSITIVE   1U
#define APP_PROFILE_LEGACY_MODULES        0U

#if (APP_PROFILE_STEP_TEST + APP_PROFILE_IMU_SHORT_REGRESSION + \
     APP_PROFILE_CONSTANT_HEADING_HOLD + APP_PROFILE_CM_ROUTE_X_POSITIVE + \
     APP_PROFILE_LEGACY_MODULES) != 1U
#error "Exactly one PBRoBot application profile must be selected"
#endif

/**
 * @brief 底盘执行层后端选择。
 *
 * 当前已实物通过的原版脉冲档和恒速航向保持档都使用直连STEP/DIR/EN后端；
 * 保留的旧应用档仍使用USART3 StepMotor后端。任何新脉冲整车档都必须显式
 * 加入本表达式，不能靠恢复旧StepMotor入口间接驱动本PCB。
 */
#define CHASSIS_USE_STEP_BACKEND  \
  (APP_PROFILE_STEP_TEST || APP_PROFILE_CONSTANT_HEADING_HOLD || \
   APP_PROFILE_CM_ROUTE_X_POSITIVE)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief 关闭中断并永久停机；仅用于不可恢复的初始化或时钟配置失败。
  * @return 无；该函数不返回。
  */
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

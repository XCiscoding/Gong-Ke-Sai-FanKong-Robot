#ifndef MOTOR_BOARD_CONFIG_H
#define MOTOR_BOARD_CONFIG_H

#include "stm32f4xx_hal.h"

/**
 * @file motor_board_config.h
 * @brief 学长配套PCB的四路STEP/DIR/EN固定映射。
 *
 * 本文件只描述板级接线，不包含电机控制策略。更换PCB时只允许在取得
 * 新原理图或实物引脚证据后修改这里，电机驱动与测试流程不应感知板级差异。
 */

#define MOTOR1_STEP_PORT                    GPIOA
#define MOTOR1_STEP_PIN                     GPIO_PIN_5
#define MOTOR1_STEP_AF                      GPIO_AF1_TIM2
#define MOTOR1_TIMER_INSTANCE               TIM2
#define MOTOR1_TIMER_IRQ                    TIM2_IRQn
#define MOTOR1_DIR_PORT                     GPIOC
#define MOTOR1_DIR_PIN                      GPIO_PIN_2
#define MOTOR1_EN_PORT                      GPIOC
#define MOTOR1_EN_PIN                       GPIO_PIN_1
#define MOTOR1_REVERSE_GPIO_POLARITY         0U

#define MOTOR2_STEP_PORT                    GPIOA
#define MOTOR2_STEP_PIN                     GPIO_PIN_6
#define MOTOR2_STEP_AF                      GPIO_AF2_TIM3
#define MOTOR2_TIMER_INSTANCE               TIM3
#define MOTOR2_TIMER_IRQ                    TIM3_IRQn
#define MOTOR2_DIR_PORT                     GPIOB
#define MOTOR2_DIR_PIN                      GPIO_PIN_13
#define MOTOR2_EN_PORT                      GPIOB
#define MOTOR2_EN_PIN                       GPIO_PIN_14
#define MOTOR2_REVERSE_GPIO_POLARITY         1U

#define MOTOR3_STEP_PORT                    GPIOD
#define MOTOR3_STEP_PIN                     GPIO_PIN_12
#define MOTOR3_STEP_AF                      GPIO_AF2_TIM4
#define MOTOR3_TIMER_INSTANCE               TIM4
#define MOTOR3_TIMER_IRQ                    TIM4_IRQn
#define MOTOR3_DIR_PORT                     GPIOE
#define MOTOR3_DIR_PIN                      GPIO_PIN_8
#define MOTOR3_EN_PORT                      GPIOE
#define MOTOR3_EN_PIN                       GPIO_PIN_7
#define MOTOR3_REVERSE_GPIO_POLARITY         0U

#define MOTOR4_STEP_PORT                    GPIOA
#define MOTOR4_STEP_PIN                     GPIO_PIN_0
#define MOTOR4_STEP_AF                      GPIO_AF2_TIM5
#define MOTOR4_TIMER_INSTANCE               TIM5
#define MOTOR4_TIMER_IRQ                    TIM5_IRQn
#define MOTOR4_DIR_PORT                     GPIOE
#define MOTOR4_DIR_PIN                      GPIO_PIN_1
#define MOTOR4_EN_PORT                      GPIOE
#define MOTOR4_EN_PIN                       GPIO_PIN_0
#define MOTOR4_REVERSE_GPIO_POLARITY         1U

/* 学长配套PCB的KEY0，低电平表示按下。 */
#define MOTOR_TEST_KEY_PORT                 GPIOB
#define MOTOR_TEST_KEY_PIN                  GPIO_PIN_10

#endif /* MOTOR_BOARD_CONFIG_H */

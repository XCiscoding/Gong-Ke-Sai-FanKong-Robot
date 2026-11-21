#include "motor.h"
#include "motor_board_config.h"

#define MOTOR_TIMER_TICK_HZ       1000000U
#define MOTOR_MIN_PERIOD_TICKS    32U
#define MOTOR_MAX_PERIOD_TICKS    65536U

typedef struct
{
  TIM_HandleTypeDef *timer;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_TypeDef *enable_port;
  uint16_t enable_pin;
  uint8_t reverse_gpio_polarity;
} MotorChannel;

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;
static TIM_HandleTypeDef htim5;

static volatile uint32_t motor_step_count[MOTOR_ID_COUNT];
static volatile int32_t motor_signed_step_count[MOTOR_ID_COUNT];
static volatile int8_t motor_command_sign[MOTOR_ID_COUNT];
static volatile uint32_t motor_step_limit[MOTOR_ID_COUNT];

static MotorChannel motor_channels[MOTOR_ID_COUNT] =
{
  {&htim2, MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, MOTOR1_EN_PORT, MOTOR1_EN_PIN, MOTOR1_REVERSE_GPIO_POLARITY},
  {&htim3, MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, MOTOR2_EN_PORT, MOTOR2_EN_PIN, MOTOR2_REVERSE_GPIO_POLARITY},
  {&htim4, MOTOR3_DIR_PORT, MOTOR3_DIR_PIN, MOTOR3_EN_PORT, MOTOR3_EN_PIN, MOTOR3_REVERSE_GPIO_POLARITY},
  {&htim5, MOTOR4_DIR_PORT, MOTOR4_DIR_PIN, MOTOR4_EN_PORT, MOTOR4_EN_PIN, MOTOR4_REVERSE_GPIO_POLARITY}
};

/**
 * @brief 判断电机标识是否可用于内部数组访问。
 * @param motor 待检查的电机标识。
 * @return 非零表示有效。
 */
static uint8_t Motor_IsValidId(MotorId motor)
{
  return ((uint32_t)motor < (uint32_t)MOTOR_ID_COUNT) ? 1U : 0U;
}

/**
 * @brief 读取APB1定时器实际输入时钟，并处理APB分频不为1时的倍频规则。
 * @return TIM2—TIM5共用的输入时钟频率，单位Hz。
 */
static uint32_t Motor_GetApb1TimerClock(void)
{
  uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();

  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1)
  {
    timer_clock *= 2U;
  }

  return timer_clock;
}

/**
 * @brief 将指定定时器初始化为CH1 PWM STEP发生器。
 * @param timer HAL定时器句柄。
 * @param instance 定时器外设实例。
 * @param prescaler 将定时器时钟分频到1 MHz的预分频值。
 * @return HAL初始化结果。
 */
static HAL_StatusTypeDef Motor_InitTimer(TIM_HandleTypeDef *timer,
                                         TIM_TypeDef *instance,
                                         uint32_t prescaler)
{
  TIM_OC_InitTypeDef pwm = {0};

  timer->Instance = instance;
  timer->Init.Prescaler = prescaler;
  timer->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer->Init.Period = 65535U;
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_PWM_Init(timer) != HAL_OK)
  {
    return HAL_ERROR;
  }

  pwm.OCMode = TIM_OCMODE_PWM1;
  pwm.Pulse = 25U;
  pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm.OCFastMode = TIM_OCFAST_DISABLE;

  return HAL_TIM_PWM_ConfigChannel(timer, &pwm, TIM_CHANNEL_1);
}

/**
 * @brief 将一个STEP引脚配置为对应定时器CH1复用推挽输出。
 * @param port GPIO端口。
 * @param pin GPIO引脚掩码。
 * @param alternate GPIO复用编号。
 */
static void Motor_InitStepPin(GPIO_TypeDef *port, uint16_t pin, uint32_t alternate)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = alternate;
  HAL_GPIO_Init(port, &gpio);
}

/**
 * @brief 按PCB定义和单路极性修正设置电机方向电平。
 * @param index 内部电机数组下标，调用前必须已验证。
 * @param reverse 非零表示反向。
 */
static void Motor_SetDirection(uint32_t index, uint8_t reverse)
{
  MotorChannel *channel = &motor_channels[index];
  GPIO_PinState level = ((reverse ^ channel->reverse_gpio_polarity) != 0U) ?
                        GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(channel->dir_port, channel->dir_pin, level);
}

/**
 * @brief 设置驱动器使能电平；配套PCB定义高电平使能。
 * @param index 内部电机数组下标，调用前必须已验证。
 * @param enabled 非零使能，0失能。
 */
static void Motor_SetEnable(uint32_t index, uint8_t enabled)
{
  HAL_GPIO_WritePin(motor_channels[index].enable_port,
                    motor_channels[index].enable_pin,
                    (enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 将RPM换算为1 MHz计数基准下的单个STEP周期。
 * @param rpm 有符号转速；这里只使用绝对值。
 * @return 周期计数；接近0 RPM时返回0。
 */
static uint32_t Motor_RpmToPeriodTicks(float rpm)
{
  float absolute_rpm = (rpm >= 0.0f) ? rpm : -rpm;
  float period_ticks;

  if (absolute_rpm < 0.001f)
  {
    return 0U;
  }

  period_ticks = ((float)MOTOR_TIMER_TICK_HZ * 60.0f) /
                 ((float)MOTOR_STEPS_PER_REV * absolute_rpm);

  if (period_ticks < (float)MOTOR_MIN_PERIOD_TICKS)
  {
    period_ticks = (float)MOTOR_MIN_PERIOD_TICKS;
  }
  if (period_ticks > (float)MOTOR_MAX_PERIOD_TICKS)
  {
    period_ticks = (float)MOTOR_MAX_PERIOD_TICKS;
  }

  return (uint32_t)(period_ticks + 0.5f);
}

/**
 * @brief 在保持驱动器失能的条件下准备一路DIR、ARR、CCR和脉冲计数起点。
 * @param index 内部电机数组下标，调用前必须已验证。
 * @param period_ticks 已验证的STEP周期；0表示该路不参与启动。
 * @param reverse 非零表示供体语义下的反向。
 * @return HAL_OK表示寄存器预装完成；写入更新事件失败时返回HAL_ERROR。
 */
static HAL_StatusTypeDef Motor_PrepareStep(uint32_t index,
                                             uint32_t period_ticks,
                                             uint8_t reverse)
{
  MotorChannel *channel = &motor_channels[index];

  __HAL_TIM_DISABLE_IT(channel->timer, TIM_IT_UPDATE);
  (void)HAL_TIM_PWM_Stop(channel->timer, TIM_CHANNEL_1);
  Motor_SetEnable(index, 0U);
  motor_command_sign[index] = (period_ticks == 0U) ? 0 :
                              ((reverse != 0U) ? -1 : 1);
  if (period_ticks == 0U)
  {
    return HAL_OK;
  }

  Motor_SetDirection(index, reverse);
  __HAL_TIM_SET_AUTORELOAD(channel->timer, period_ticks - 1U);
  __HAL_TIM_SET_COMPARE(channel->timer, TIM_CHANNEL_1, period_ticks / 2U);
  if (HAL_TIM_GenerateEvent(channel->timer, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  __HAL_TIM_SET_COUNTER(channel->timer, 0U);
  __HAL_TIM_CLEAR_FLAG(channel->timer, TIM_FLAG_UPDATE);
  return HAL_OK;
}

/**
 * @brief 启动已准备的一路PWM，但保持驱动器失能以等待批量提交。
 * @param index 内部电机数组下标，调用前必须已验证。
 * @param period_ticks 已验证的STEP周期；0表示保持该路失能。
 * @return HAL_OK表示该路PWM已启动且仍保持失能；PWM启动失败时保持失能并返回HAL_ERROR。
 */
static HAL_StatusTypeDef Motor_StartPrepared(uint32_t index, uint32_t period_ticks)
{
  MotorChannel *channel = &motor_channels[index];

  if (period_ticks == 0U)
  {
    return HAL_OK;
  }
  __HAL_TIM_ENABLE_IT(channel->timer, TIM_IT_UPDATE);
  if (HAL_TIM_PWM_Start(channel->timer, TIM_CHANNEL_1) != HAL_OK)
  {
    __HAL_TIM_DISABLE_IT(channel->timer, TIM_IT_UPDATE);
    Motor_SetEnable(index, 0U);
    return HAL_ERROR;
  }
  return HAL_OK;
}

/**
 * @brief 使能已经完成PWM启动的一路驱动器。
 * @param index 内部电机数组下标，调用前必须已验证。
 * @param period_ticks 已验证的STEP周期；0表示保持该路失能。
 */
static void Motor_EnablePrepared(uint32_t index, uint32_t period_ticks)
{
  if (period_ticks != 0U)
  {
    Motor_SetEnable(index, 1U);
  }
}

/**
 * @brief 将已经验证的单路RPM命令应用到定时器和DIR/EN引脚。
 * @param index 内部电机数组下标，调用前必须已验证。
 * @param rpm 有符号目标转速。
 * @return HAL_OK表示输出已启动或已安全停止；启动失败返回HAL_ERROR并保持失能。
 */
static HAL_StatusTypeDef Motor_ApplyRpm(uint32_t index, float rpm)
{
  uint32_t period_ticks = Motor_RpmToPeriodTicks(rpm);

  if (Motor_PrepareStep(index, period_ticks, (rpm < 0.0f) ? 1U : 0U) != HAL_OK)
  {
    Motor_SetEnable(index, 0U);
    return HAL_ERROR;
  }
  if (Motor_StartPrepared(index, period_ticks) != HAL_OK)
  {
    return HAL_ERROR;
  }
  Motor_EnablePrepared(index, period_ticks);
  return HAL_OK;
}

/**
 * @brief 初始化四路固定PCB脉冲驱动，任何失败均保持全部电机停止。
 * @return HAL_OK表示四路初始化完成；否则返回HAL_ERROR。
 */
HAL_StatusTypeDef Motor_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t timer_clock;
  uint32_t prescaler;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();

  Motor_InitStepPin(MOTOR1_STEP_PORT, MOTOR1_STEP_PIN, MOTOR1_STEP_AF);
  Motor_InitStepPin(MOTOR2_STEP_PORT, MOTOR2_STEP_PIN, MOTOR2_STEP_AF);
  Motor_InitStepPin(MOTOR3_STEP_PORT, MOTOR3_STEP_PIN, MOTOR3_STEP_AF);
  Motor_InitStepPin(MOTOR4_STEP_PORT, MOTOR4_STEP_PIN, MOTOR4_STEP_AF);

  gpio.Pin = MOTOR1_DIR_PIN | MOTOR1_EN_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin = MOTOR2_DIR_PIN | MOTOR2_EN_PIN;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = MOTOR3_DIR_PIN | MOTOR3_EN_PIN | MOTOR4_DIR_PIN | MOTOR4_EN_PIN;
  HAL_GPIO_Init(GPIOE, &gpio);

  HAL_GPIO_WritePin(MOTOR1_EN_PORT, MOTOR1_EN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR2_EN_PORT, MOTOR2_EN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR3_EN_PORT, MOTOR3_EN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR4_EN_PORT, MOTOR4_EN_PIN, GPIO_PIN_RESET);

  timer_clock = Motor_GetApb1TimerClock();
  if ((timer_clock < MOTOR_TIMER_TICK_HZ) ||
      ((timer_clock % MOTOR_TIMER_TICK_HZ) != 0U))
  {
    return HAL_ERROR;
  }
  prescaler = (timer_clock / MOTOR_TIMER_TICK_HZ) - 1U;

  if (Motor_InitTimer(&htim2, TIM2, prescaler) != HAL_OK) return HAL_ERROR;
  if (Motor_InitTimer(&htim3, TIM3, prescaler) != HAL_OK) return HAL_ERROR;
  if (Motor_InitTimer(&htim4, TIM4, prescaler) != HAL_OK) return HAL_ERROR;
  if (Motor_InitTimer(&htim5, TIM5, prescaler) != HAL_OK) return HAL_ERROR;

  HAL_NVIC_SetPriority(TIM2_IRQn, 5U, 0U);
  HAL_NVIC_SetPriority(TIM3_IRQn, 5U, 0U);
  HAL_NVIC_SetPriority(TIM4_IRQn, 5U, 0U);
  HAL_NVIC_SetPriority(TIM5_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

  for (uint32_t index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index)
  {
    Motor_ResetStepCount((MotorId)index);
    Motor_SetStepLimit((MotorId)index, 0U);
  }
  return HAL_OK;
}

/**
 * @brief 设置单路电机RPM，不包含底盘运动学或测试编排语义。
 * @param motor 电机标识。
 * @param rpm 有符号目标转速。
 * @return HAL命令结果；标识无效时返回HAL_ERROR。
 */
HAL_StatusTypeDef Motor_SetRpm(MotorId motor, float rpm)
{
  if (Motor_IsValidId(motor) == 0U)
  {
    return HAL_ERROR;
  }
  return Motor_ApplyRpm((uint32_t)motor, rpm);
}

/**
 * @brief 以同一提交流程准备并启动四路STEP脉冲，保留供体的四轮命令语义。
 * @param motor1_rpm 至 motor4_rpm 四路有符号目标RPM；0表示该路保持失能。
 * @return HAL_OK表示四路均已提交；任一准备或启动失败时全部四路失能并返回HAL_ERROR。
 */
HAL_StatusTypeDef Motor_SetFourRpm(float motor1_rpm,
                                   float motor2_rpm,
                                   float motor3_rpm,
                                   float motor4_rpm)
{
  const float rpm[MOTOR_ID_COUNT] = {motor1_rpm, motor2_rpm, motor3_rpm, motor4_rpm};
  uint32_t period_ticks[MOTOR_ID_COUNT];
  uint32_t index;

  for (index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index)
  {
    period_ticks[index] = Motor_RpmToPeriodTicks(rpm[index]);
  }

  Motor_StopAll();
  for (index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index)
  {
    if (Motor_PrepareStep(index, period_ticks[index], (rpm[index] < 0.0f) ? 1U : 0U) != HAL_OK)
    {
      Motor_StopAll();
      return HAL_ERROR;
    }
  }
  for (index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index)
  {
    if (Motor_StartPrepared(index, period_ticks[index]) != HAL_OK)
    {
      Motor_StopAll();
      return HAL_ERROR;
    }
  }
  for (index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index)
  {
    Motor_EnablePrepared(index, period_ticks[index]);
  }
  return HAL_OK;
}

/**
 * @brief 停止并失能指定电机；无效标识不触碰硬件。
 * @param motor 电机标识。
 */
void Motor_Stop(MotorId motor)
{
  uint32_t index = (uint32_t)motor;

  if (Motor_IsValidId(motor) != 0U)
  {
    __HAL_TIM_DISABLE_IT(motor_channels[index].timer, TIM_IT_UPDATE);
    (void)HAL_TIM_PWM_Stop(motor_channels[index].timer, TIM_CHANNEL_1);
    Motor_SetEnable(index, 0U);
    motor_command_sign[index] = 0;
  }
}

/** @brief 停止并失能全部电机。 */
void Motor_StopAll(void)
{
  uint32_t index;

  for (index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index)
  {
    Motor_Stop((MotorId)index);
  }
}

/**
 * @brief 原子清零指定电机的脉冲计数。
 * @param motor 电机标识。
 */
void Motor_ResetStepCount(MotorId motor)
{
  uint32_t saved_primask;

  if (Motor_IsValidId(motor) == 0U)
  {
    return;
  }
  saved_primask = __get_PRIMASK();
  __disable_irq();
  motor_step_count[(uint32_t)motor] = 0U;
  motor_signed_step_count[(uint32_t)motor] = 0;
  if (saved_primask == 0U)
  {
    __enable_irq();
  }
}

/**
 * @brief 设置指定电机的独立自动停机脉冲上限。
 * @param motor 电机标识。
 * @param step_limit 0表示关闭上限，其余值表示达到该计数后停止该路。
 */
void Motor_SetStepLimit(MotorId motor, uint32_t step_limit)
{
  if (Motor_IsValidId(motor) != 0U)
  {
    motor_step_limit[(uint32_t)motor] = step_limit;
  }
}

/**
 * @brief 原子读取指定电机的STEP计数。
 * @param motor 电机标识。
 * @return 当前计数；标识无效时返回0。
 */
uint32_t Motor_GetStepCount(MotorId motor)
{
  uint32_t saved_primask;
  uint32_t count;

  if (Motor_IsValidId(motor) == 0U)
  {
    return 0U;
  }

  saved_primask = __get_PRIMASK();
  __disable_irq();
  count = motor_step_count[(uint32_t)motor];
  if (saved_primask == 0U)
  {
    __enable_irq();
  }

  return count;
}

/**
 * @brief 原子读取指定电机按当前RPM命令方向累计的STEP数。
 * @param motor 电机标识。
 * @return 正RPM为正、负RPM为负；无效标识返回0。
 */
int32_t Motor_GetSignedStepCount(MotorId motor)
{
  uint32_t saved_primask;
  int32_t count;

  if (Motor_IsValidId(motor) == 0U)
  {
    return 0;
  }

  saved_primask = __get_PRIMASK();
  __disable_irq();
  count = motor_signed_step_count[(uint32_t)motor];
  if (saved_primask == 0U)
  {
    __enable_irq();
  }
  return count;
}

/**
 * @brief 处理一路定时器更新中断，累计STEP并执行该路独立脉冲上限停车。
 * @param index 内部电机数组下标，由固定IRQ入口提供。
 */
static void Motor_HandleUpdate(uint32_t index)
{
  TIM_HandleTypeDef *timer = motor_channels[index].timer;

  if ((__HAL_TIM_GET_FLAG(timer, TIM_FLAG_UPDATE) != RESET) &&
      (__HAL_TIM_GET_IT_SOURCE(timer, TIM_IT_UPDATE) != RESET))
  {
    __HAL_TIM_CLEAR_IT(timer, TIM_IT_UPDATE);
    motor_step_count[index]++;
    motor_signed_step_count[index] += motor_command_sign[index];

    if ((motor_step_limit[index] != 0U) &&
        (motor_step_count[index] >= motor_step_limit[index]))
    {
      __HAL_TIM_DISABLE_IT(timer, TIM_IT_UPDATE);
      __HAL_TIM_DISABLE(timer);
      Motor_SetEnable(index, 0U);
    }
  }
}

/** @brief TIM2更新中断入口，对应M1 STEP计数。 */
void TIM2_IRQHandler(void)
{
  Motor_HandleUpdate(0U);
}

/** @brief TIM3更新中断入口，对应M2 STEP计数。 */
void TIM3_IRQHandler(void)
{
  Motor_HandleUpdate(1U);
}

/** @brief TIM4更新中断入口，对应M3 STEP计数。 */
void TIM4_IRQHandler(void)
{
  Motor_HandleUpdate(2U);
}

/** @brief TIM5更新中断入口，对应M4 STEP计数。 */
void TIM5_IRQHandler(void)
{
  Motor_HandleUpdate(3U);
}

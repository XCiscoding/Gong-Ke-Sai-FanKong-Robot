#include "main.h"
#include "arm_tune.h"

#define PCA9685_ADDR 0x80U
#define PCA_MODE1 0x00U
#define PCA_MODE2 0x01U
#define PCA_PRE_SCALE 0xFEU
#define PCA_LED0_ON_L 0x06U
#define PCA_ALL_LED_OFF_H 0xFDu
#define ARM_FAULT_I2C 1U
#define ARM_FAULT_PARAM 2U

/* 校准层：只描述单个关节如何把逻辑角度换成PWM；不保存抓球、视觉等任务姿态。
 * 注意：下面每行第一个0U是普通手动模式的启用位；顺序测试时不用改它。 */
ArmJointParam arm_user_table[ARM_JOINT_COUNT] = {
  /* CH0底座：物理135°中位 = 逻辑0°，逻辑范围候选-135..+135；保持禁用待实体确认。 */
  {1U, 0U, 0, -135, 135, 1280U, 1, 74U, 25U, 1000U, 1U}, /* CH0：正=左；速度6度/秒。 */
  /* CH1大臂：用户确认的逻辑0°=1200us；正方向=上；范围仍待单关节实体证据。 */
  {1U, 1U, 0, 0, 180, 1250U, 1, 110U, 25U, 1000U, 1U}, /* CH1：正=上；速度6度/秒。 */
  {1U, 2U, 0, 0, 120, 900U, 1, 110U, 25U, 1000U, 1U},  /* CH2：正=后；速度6度/秒。 */
  {1U, 3U, 0, 0, 120, 2000U, 1, 110U, 25U, 1000U, 1U},  /* CH3：正=上；速度6度/秒。 */
  {1U, 4U, 0, 0, 120, 1400U, 1, 110U, 25U, 1000U, 1U}   /* CH4：正=闭合；速度6度/秒。 */
};

/* 调试计划只选择一个已知关节；不通过enabled表扫描，更不会依次驱动其它通道。 */
#define ARM_TUNE_PROBE_ENABLED  APP_PROFILE_ARM_TUNE
#define ARM_TUNE_PROBE_JOINT    ARM_JOINT_BASE
#define ARM_TUNE_PROBE_DEG      87
#define ARM_TUNE_PROBE_WAIT_MS  1000U
#define ARM_TUNE_TARGET_CH0_DEG  0   /* 底座：居中；CH0当前禁用。 */
#define ARM_TUNE_TARGET_CH1_DEG  0   /* 大臂：可填0..30。 */
#define ARM_TUNE_TARGET_CH2_DEG  0   /* 小臂：未启用。 */
#define ARM_TUNE_TARGET_CH3_DEG  0   /* 手腕：未启用。 */
#define ARM_TUNE_TARGET_CH4_DEG  0   /* 夹爪：未启用。 */
const ArmPose arm_tune_manual_pose = {{
  ARM_TUNE_TARGET_CH0_DEG, ARM_TUNE_TARGET_CH1_DEG, ARM_TUNE_TARGET_CH2_DEG,
  ARM_TUNE_TARGET_CH3_DEG, ARM_TUNE_TARGET_CH4_DEG
}};

/* 参数表顺序：{enabled, channel, home_deg, min_deg, max_deg, zero_us, direction,
 * us_per_deg_x10, speed_deg_s, settle_ms, output_ready}。
 * home_deg是启动后要去的安全逻辑姿态；手动测试目标在arm_tune_manual_pose，不再混入参数表。
 * zero_us是逻辑0°电气基准；CH0的逻辑0°就是物理135°中位。
 * us_per_deg_x10的110=11.0us/度、74=7.4us/度；它不是速度。
 * 用户日常只改enabled、home_deg、zero_us、speed_deg_s、settle_ms和手动姿态目标。
 * channel、direction、比例、限位与output_ready必须跟随单关节证据，不作为日常调参项。 */

volatile ArmTuneState arm_tune_state = ARM_TUNE_IDLE;
volatile uint32_t arm_tune_fault = 0U;
static int16_t current_deg[ARM_JOINT_COUNT];
static int16_t requested_deg[ARM_JOINT_COUNT];
static uint32_t active_mask;
static uint32_t hold_mask;
static uint32_t ramp_mask;
static uint32_t last_step_ms;
static uint32_t entry_hold_ms;
static uint32_t ramp_not_before_ms;
static uint8_t initial_output_pending = 1U;
#if ARM_TUNE_PROBE_ENABLED
typedef enum { ARM_PROBE_ZERO = 0U, ARM_PROBE_HOLD_ZERO, ARM_PROBE_TO_TARGET,
               ARM_PROBE_HOLD_TARGET, ARM_PROBE_TO_HOME, ARM_PROBE_HOLD_HOME,
               ARM_PROBE_DONE } ArmTuneProbeState;
static ArmTuneProbeState arm_probe_state = ARM_PROBE_ZERO;
static uint32_t arm_probe_deadline_ms;
#endif

static uint32_t Arm_StepPeriodMs(void)
{
  uint32_t i;
  for (i = 0U; i < ARM_JOINT_COUNT; ++i)
  {
    if (arm_user_table[i].enabled != 0U && arm_user_table[i].speed_deg_s != 0U)
      return 1000U / arm_user_table[i].speed_deg_s;
  }
  return 1000U;
}

static uint8_t I2c2_Wait(uint32_t flag, uint8_t set)
{
  uint32_t timeout = 100000U;
  while (((I2C2->SR1 & flag) != 0U) != (set != 0U)) if (--timeout == 0U) return 0U;
  return 1U;
}
static uint8_t I2c2_Write(uint8_t reg, uint8_t value)
{
  I2C2->CR1 |= I2C_CR1_START;
  if (!I2c2_Wait(I2C_SR1_SB, 1U)) return 0U;
  I2C2->DR = PCA9685_ADDR;
  if (!I2c2_Wait(I2C_SR1_ADDR, 1U)) return 0U;
  (void)I2C2->SR1; (void)I2C2->SR2;
  I2C2->DR = reg; if (!I2c2_Wait(I2C_SR1_TXE, 1U)) return 0U;
  I2C2->DR = value; if (!I2c2_Wait(I2C_SR1_BTF, 1U)) return 0U;
  I2C2->CR1 |= I2C_CR1_STOP;
  return 1U;
}
/* PCA9685已开启自动地址递增；四个PWM寄存器必须一笔写完，避免中途形成半旧半新的脉宽。 */
static uint8_t I2c2_Write4(uint8_t reg, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  uint8_t i, data[4] = {b0, b1, b2, b3};
  I2C2->CR1 |= I2C_CR1_START;
  if (!I2c2_Wait(I2C_SR1_SB, 1U)) return 0U;
  I2C2->DR = PCA9685_ADDR;
  if (!I2c2_Wait(I2C_SR1_ADDR, 1U)) return 0U;
  (void)I2C2->SR1; (void)I2C2->SR2;
  I2C2->DR = reg;
  if (!I2c2_Wait(I2C_SR1_TXE, 1U)) return 0U;
  for (i = 0U; i < 4U; ++i) {
    I2C2->DR = data[i];
    if (!I2c2_Wait((i == 3U) ? I2C_SR1_BTF : I2C_SR1_TXE, 1U)) return 0U;
  }
  I2C2->CR1 |= I2C_CR1_STOP;
  return 1U;
}
static uint8_t Pca_WritePulse(uint8_t channel, uint16_t us)
{
  uint16_t count;
  if (channel >= ARM_JOINT_COUNT || us < 500U || us > 2500U) return 0U;
  count = (uint16_t)((us * 4096U + 10000U) / 20000U);
  return I2c2_Write4((uint8_t)(PCA_LED0_ON_L + 4U * channel), 0U, 0U,
                     (uint8_t)count, (uint8_t)(count >> 8));
}
static uint8_t Arm_PulseForDeg(const ArmJointParam *p, int16_t deg, uint16_t *pulse_us)
{
  int32_t offset_x10 = (int32_t)p->direction * (int32_t)deg * (int32_t)p->us_per_deg_x10;
  int32_t offset_us = (offset_x10 >= 0) ? (offset_x10 + 5) / 10 : (offset_x10 - 5) / 10;
  int32_t pulse = (int32_t)p->zero_us + offset_us;
  if (pulse < 500 || pulse > 2500) return 0U;
  *pulse_us = (uint16_t)pulse;
  return 1U;
}

#if ARM_TUNE_PROBE_ENABLED
static void ArmTune_ProbeFault(void)
{
  arm_tune_fault = ARM_FAULT_PARAM;
  arm_tune_state = ARM_TUNE_FAULT_LOCKED;
}

static void ArmTune_SingleProbeService(uint32_t now_ms)
{
  ArmJointParam *p; uint16_t pulse; int16_t next; uint32_t period_ms;
  if (arm_probe_state == ARM_PROBE_DONE || arm_tune_state == ARM_TUNE_FAULT_LOCKED) return;
  if (ARM_TUNE_PROBE_JOINT >= ARM_JOINT_COUNT) { ArmTune_ProbeFault(); return; }
  p = &arm_user_table[ARM_TUNE_PROBE_JOINT];
  if (p->output_ready == 0U || p->speed_deg_s == 0U || ARM_TUNE_PROBE_DEG < p->min_deg ||
      ARM_TUNE_PROBE_DEG > p->max_deg || !Arm_PulseForDeg(p, ARM_TUNE_PROBE_DEG, &pulse)) { ArmTune_ProbeFault(); return; }
  period_ms = 1000U / p->speed_deg_s;

  if (arm_probe_state == ARM_PROBE_ZERO) {
    if (!Arm_PulseForDeg(p, 0, &pulse) || !Pca_WritePulse(p->channel, pulse)) { arm_tune_fault = ARM_FAULT_I2C; arm_tune_state = ARM_TUNE_FAULT_LOCKED; return; }
    current_deg[ARM_TUNE_PROBE_JOINT] = 0; arm_probe_deadline_ms = now_ms + ARM_TUNE_PROBE_WAIT_MS; arm_probe_state = ARM_PROBE_HOLD_ZERO; return;
  }
  if (arm_probe_state == ARM_PROBE_HOLD_ZERO) { if ((int32_t)(now_ms - arm_probe_deadline_ms) >= 0) { last_step_ms = now_ms; arm_probe_state = ARM_PROBE_TO_TARGET; } return; }
  if (arm_probe_state == ARM_PROBE_TO_TARGET) {
    if ((uint32_t)(now_ms - last_step_ms) < period_ms) return;
    last_step_ms = now_ms;
    if (current_deg[ARM_TUNE_PROBE_JOINT] == ARM_TUNE_PROBE_DEG) { arm_probe_deadline_ms = now_ms + ARM_TUNE_PROBE_WAIT_MS; arm_probe_state = ARM_PROBE_HOLD_TARGET; return; }
    next = (int16_t)(current_deg[ARM_TUNE_PROBE_JOINT] + 1);
    if (!Arm_PulseForDeg(p, next, &pulse) || !Pca_WritePulse(p->channel, pulse)) { arm_tune_fault = ARM_FAULT_I2C; arm_tune_state = ARM_TUNE_FAULT_LOCKED; return; }
    current_deg[ARM_TUNE_PROBE_JOINT] = next; return;
  }
  if (arm_probe_state == ARM_PROBE_HOLD_TARGET) { if ((int32_t)(now_ms - arm_probe_deadline_ms) >= 0) { last_step_ms = now_ms; arm_probe_state = ARM_PROBE_TO_HOME; } return; }
  if (arm_probe_state == ARM_PROBE_TO_HOME) {
    if ((uint32_t)(now_ms - last_step_ms) < period_ms) return;
    last_step_ms = now_ms;
    if (current_deg[ARM_TUNE_PROBE_JOINT] == 0) { arm_probe_deadline_ms = now_ms + ARM_TUNE_PROBE_WAIT_MS; arm_probe_state = ARM_PROBE_HOLD_HOME; return; }
    next = (int16_t)(current_deg[ARM_TUNE_PROBE_JOINT] - 1);
    if (!Arm_PulseForDeg(p, next, &pulse) || !Pca_WritePulse(p->channel, pulse)) { arm_tune_fault = ARM_FAULT_I2C; arm_tune_state = ARM_TUNE_FAULT_LOCKED; return; }
    current_deg[ARM_TUNE_PROBE_JOINT] = next; return;
  }
  if (arm_probe_state == ARM_PROBE_HOLD_HOME && (int32_t)(now_ms - arm_probe_deadline_ms) >= 0) { arm_probe_state = ARM_PROBE_DONE; arm_tune_state = ARM_TUNE_HOLD; }
}
#endif
static uint8_t Arm_Validate(void)
{
  uint32_t i; uint8_t active = 0U; uint16_t pulse;
  for (i = 0U; i < ARM_JOINT_COUNT; ++i) if ((active_mask & (1UL << i)) != 0U) {
    active++;
    if (arm_user_table[i].output_ready == 0U || requested_deg[i] < arm_user_table[i].min_deg || requested_deg[i] > arm_user_table[i].max_deg || !Arm_PulseForDeg(&arm_user_table[i], requested_deg[i], &pulse)) return 0U;
  }
  return active != 0U;
}

static uint8_t Arm_TargetValid(const ArmJointParam *p, int16_t target_deg)
{
  uint16_t pulse;
  return (p->output_ready != 0U &&
          target_deg >= p->min_deg && target_deg <= p->max_deg && Arm_PulseForDeg(p, target_deg, &pulse)) ? 1U : 0U;
}

uint8_t ArmTune_RequestPose(const ArmPose *pose)
{
  uint32_t i;
  if (pose == 0 || arm_tune_state == ARM_TUNE_FAULT_LOCKED) return 0U;
  active_mask = 0U; hold_mask = 0U; ramp_mask = 0U;
  for (i = 0U; i < ARM_JOINT_COUNT; ++i)
    if (arm_user_table[i].enabled != 0U) {
      if (!Arm_TargetValid(&arm_user_table[i], pose->target_deg[i])) return 0U;
      active_mask |= (1UL << i); ramp_mask |= (1UL << i);
    }
  for (i = 0U; i < ARM_JOINT_COUNT; ++i)
    if ((active_mask & (1UL << i)) != 0U) requested_deg[i] = pose->target_deg[i];
  /* 只更新目的地；current_deg保留当前运行态，不触发启动零位输出。 */
  if (arm_tune_state != ARM_TUNE_IDLE) arm_tune_state = ARM_TUNE_RAMP;
  return 1U;
}

uint8_t ArmTune_RequestMaskedPose(const ArmPose *pose, uint32_t requested_hold_mask, uint32_t requested_ramp_mask,
                                  int16_t requested_ramp_start_deg, uint32_t requested_entry_hold_ms)
{
  uint32_t i, allowed_mask = (1UL << ARM_JOINT_COUNT) - 1UL;
  if (pose == 0 || arm_tune_state == ARM_TUNE_FAULT_LOCKED || requested_ramp_mask == 0U ||
      (requested_ramp_mask & (requested_ramp_mask - 1UL)) != 0U ||
      (requested_hold_mask & requested_ramp_mask) != 0U ||
      ((requested_hold_mask | requested_ramp_mask) & ~allowed_mask) != 0U) return 0U;
  active_mask = requested_hold_mask | requested_ramp_mask;
  hold_mask = requested_hold_mask;
  ramp_mask = requested_ramp_mask;
  for (i = 0U; i < ARM_JOINT_COUNT; ++i) {
    if ((active_mask & (1UL << i)) == 0U) continue;
    if (!Arm_TargetValid(&arm_user_table[i], pose->target_deg[i])) return 0U;
    if ((requested_ramp_mask & (1UL << i)) != 0U &&
        !Arm_TargetValid(&arm_user_table[i], requested_ramp_start_deg)) return 0U;
    requested_deg[i] = pose->target_deg[i];
    current_deg[i] = ((hold_mask & (1UL << i)) != 0U) ? pose->target_deg[i] : requested_ramp_start_deg;
  }
  entry_hold_ms = requested_entry_hold_ms;
  ramp_not_before_ms = 0U;
  initial_output_pending = 1U;
  if (arm_tune_state != ARM_TUNE_IDLE) arm_tune_state = ARM_TUNE_RAMP;
  return 1U;
}
uint8_t ArmTune_RequestHome(void)
{
  ArmPose home = {{0, 0, 0, 0, 0}}; uint32_t i;
  for (i = 0U; i < ARM_JOINT_COUNT; ++i) home.target_deg[i] = arm_user_table[i].home_deg;
  return ArmTune_RequestPose(&home);
}
uint8_t ArmTune_RequestManualPose(void) { return ArmTune_RequestPose(&arm_tune_manual_pose); }

uint8_t ArmTune_IsAtTarget(void)
{
  uint32_t i;
  if (arm_tune_state != ARM_TUNE_HOLD) return 0U;
  for (i = 0U; i < ARM_JOINT_COUNT; ++i)
    if ((active_mask & (1UL << i)) != 0U && current_deg[i] != requested_deg[i]) return 0U;
  return 1U;
}
uint8_t ArmTune_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_I2C2_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11; gpio.Mode = GPIO_MODE_AF_OD; gpio.Pull = GPIO_PULLUP; gpio.Speed = GPIO_SPEED_FREQ_LOW; gpio.Alternate = GPIO_AF4_I2C2; HAL_GPIO_Init(GPIOB, &gpio);
  I2C2->CR1 = I2C_CR1_SWRST; I2C2->CR1 = 0U; I2C2->CR2 = 16U; I2C2->CCR = 80U; I2C2->TRISE = 17U; I2C2->CR1 = I2C_CR1_PE;
  if (!I2c2_Write(PCA_MODE1, 0x10U) || !I2c2_Write(PCA_PRE_SCALE, 121U) || !I2c2_Write(PCA_MODE1, 0x20U) || !I2c2_Write(PCA_MODE2, 0x04U) || !I2c2_Write(PCA_ALL_LED_OFF_H, 0x10U)) { arm_tune_fault = ARM_FAULT_I2C; arm_tune_state = ARM_TUNE_FAULT_LOCKED; return 0U; }
  return 1U;
}
void ArmTune_Service(uint32_t now_ms)
{
  uint32_t i; int16_t delta, next; ArmJointParam *p; uint16_t pulse;
#if ARM_TUNE_PROBE_ENABLED
  ArmTune_SingleProbeService(now_ms);
  return;
#endif
  if (arm_tune_state == ARM_TUNE_FAULT_LOCKED) return;
  if (arm_tune_state == ARM_TUNE_IDLE) { if (!Arm_Validate()) return; arm_tune_state = ARM_TUNE_RAMP; last_step_ms = now_ms; initial_output_pending = 1U; }
  /* 即使目标就是0°也必须先写一次零点保持脉冲；不能把RAM初值误当作PCA已输出。 */
  if (initial_output_pending != 0U) {
    for (i = 0U; i < ARM_JOINT_COUNT; ++i) { p = &arm_user_table[i]; if ((active_mask & (1UL << i)) != 0U && (!Arm_PulseForDeg(p, current_deg[i], &pulse) || !Pca_WritePulse(p->channel, pulse))) { arm_tune_fault = ARM_FAULT_I2C; arm_tune_state = ARM_TUNE_FAULT_LOCKED; return; } }
    initial_output_pending = 0U;
    last_step_ms = now_ms;
    ramp_not_before_ms = now_ms + entry_hold_ms;
    return;
  }
  if ((int32_t)(now_ms - ramp_not_before_ms) < 0) return;
  if (arm_tune_state != ARM_TUNE_RAMP || (uint32_t)(now_ms - last_step_ms) < Arm_StepPeriodMs()) return;
  last_step_ms = now_ms;
  for (i = 0U; i < ARM_JOINT_COUNT; ++i) { p = &arm_user_table[i]; if ((ramp_mask & (1UL << i)) == 0U) continue; delta = (int16_t)(requested_deg[i] - current_deg[i]); if (delta == 0) continue; next = (int16_t)(current_deg[i] + ((delta > 0) ? 1 : -1)); if (!Arm_PulseForDeg(p, next, &pulse) || !Pca_WritePulse(p->channel, pulse)) { arm_tune_fault = ARM_FAULT_I2C; arm_tune_state = ARM_TUNE_FAULT_LOCKED; return; } current_deg[i] = next; return; }
  arm_tune_state = ARM_TUNE_HOLD;
}

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PBRoBot 应用入口与受控配置选择。
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

#if APP_PROFILE_STEP_TEST
#include "motor.h"
#elif APP_PROFILE_IMU_SHORT_REGRESSION
#include "motor.h"
#include "imu_ms901m.h"
#elif APP_PROFILE_CONSTANT_HEADING_HOLD
#include "motor.h"
#include "imu_ms901m.h"
#include "chassis.h"
#include "chassis_heading.h"
#elif APP_PROFILE_CM_ROUTE_TEST || APP_PROFILE_VISION_DISTANCE_ALIGN
#include "motor.h"
#include "imu_ms901m.h"
#include "chassis.h"
#include "chassis_heading.h"
#include "chassis_move.h"
#if APP_PROFILE_CM_ROUTE_TEST || APP_PROFILE_VISION_DISTANCE_ALIGN
#include "vision.h"
#endif
#if APP_PROFILE_VISION_DISTANCE_ALIGN
#include "vision_controller.h"
#endif
#if APP_PROFILE_CM_ROUTE_TEST
#include "mission.h"
#endif
#elif APP_PROFILE_PASSIVE_VISION_REPORT
#include "vision.h"
#include "vision_controller_observer.h"
#elif APP_PROFILE_ARM_TUNE || APP_PROFILE_ARM_GRASP_CAPTURE
#include "arm_tune.h"
#if APP_PROFILE_ARM_GRASP_CAPTURE
#include "arm_grasp_stage_panel.h"
#endif
#else
#include "imu_ms901m.h"
#include "step_motor.h"
#include "motor_test.h"
#include "chassis.h"
#include "chassis_heading.h"
#include "vision.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#if APP_PROFILE_STEP_TEST
/* 四路一次性证明保持可配置，但可见运行时长和总超时必须始终有界。 */
#define MOTOR_TEST_RPM                     30.0f
#define MOTOR_TEST_STEPS                  9600U
#define MOTOR_TEST_STABILIZE_MS             500U
#define MOTOR_TEST_TIMEOUT_MARGIN_MS      1000U
#define MOTOR_TEST_MIN_VISIBLE_DURATION_MS 3000U
#define MOTOR_TEST_MAX_VISIBLE_DURATION_MS 15000U
#endif

#if APP_PROFILE_CONSTANT_HEADING_HOLD
/** 恒速复刻档的纵向基础速度；第一版不直接进入150 RPM目标档。 */
#define CONSTANT_HOLD_FORWARD_RPM             100.0f
/** 上电后保持全停并等待MS901M建零的最短时间。 */
#define CONSTANT_HOLD_STABILIZE_MS            3000U
/** IMU始终未就绪时的启动总超时；超时后全停锁存。 */
#define CONSTANT_HOLD_START_TIMEOUT_MS        5000U
/** 真正恒速、无正弦加减速的单次运行时长。 */
#define CONSTANT_HOLD_RUN_MS                  3000U
/** RAM中的活动画像标记：ASCII "HOLD"。 */
#define APP_CONSTANT_HEADING_HOLD_MARKER 0x484F4C44U
#endif

#if APP_PROFILE_CM_ROUTE_TEST || APP_PROFILE_VISION_DISTANCE_ALIGN
/** RAM中的活动画像标记：ASCII "CMR1"。 */
#define APP_CM_ROUTE_TEST_MARKER 0x434D5231U
#endif

#if APP_PROFILE_VISION_DISTANCE_ALIGN
/* 距离对齐只允许小段纯X；实物结果返回前不扩大此限幅或速度。 */
#define VISION_ALIGN_TOTAL_TIMEOUT_MS          8000U
#define VISION_ALIGN_CENTER_CONFIRM_SAMPLES       3U
#define VISION_ALIGN_X_SPEED_RPM                  50
#define VISION_ALIGN_MAX_DISTANCE_CM               3U
/* K230固定帧：dx=2*(474-M)。17px约1cm，所以1个dx约0.294mm。 */
#define VISION_ALIGN_DX_TO_X_MM             0.29411765f
#define VISION_ALIGN_ENTER_DEADBAND_DX              6U
#define VISION_ALIGN_EXIT_DEADBAND_DX              10U
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#if APP_PROFILE_STEP_TEST
typedef enum
{
  STEP_TEST_BOOT_SAFE = 0,
  STEP_TEST_STABILIZE,
  STEP_TEST_RUN_ONCE,
  STEP_TEST_STOP_LATCHED
} StepTestState;

static StepTestState motor_test_state = STEP_TEST_BOOT_SAFE;
static uint32_t motor_test_state_started_at = 0U;
#endif

#if APP_PROFILE_IMU_SHORT_REGRESSION || APP_PROFILE_CONSTANT_HEADING_HOLD || \
    APP_PROFILE_CM_ROUTE_TEST || APP_PROFILE_VISION_DISTANCE_ALIGN
#define APP_IMU_SHORT_REGRESSION_MARKER 0x494D5531U
volatile uint32_t app_runtime_profile_marker = 0U;
#endif

#if APP_PROFILE_VISION_DISTANCE_ALIGN
typedef enum
{
  VISION_ALIGN_BOOT_SAFE = 0U,
  VISION_ALIGN_ACQUIRE,
  VISION_ALIGN_MOVING,
  VISION_ALIGN_CENTER_CONFIRM,
  VISION_ALIGN_COMPLETE,
  VISION_ALIGN_FAILED
} VisionAlignState;

static VisionController vision_align_controller;
static VisionAlignState vision_align_state = VISION_ALIGN_BOOT_SAFE;
static uint32_t vision_align_started_ms = 0U;
static uint32_t vision_align_last_center_sample_id = 0U;
static uint8_t vision_align_center_sample_count = 0U;
#endif /* APP_PROFILE_VISION_DISTANCE_ALIGN */

#if APP_PROFILE_CONSTANT_HEADING_HOLD
/** 恒速HOLD一次性测试的运行状态；到达STOP_LATCHED后只允许复位重启。 */
typedef enum
{
  CONSTANT_HOLD_BOOT_SAFE = 0U,
  CONSTANT_HOLD_STABILIZE,
  CONSTANT_HOLD_RUN_ONCE,
  CONSTANT_HOLD_STOP_LATCHED
} ConstantHoldState;

/** 恒速HOLD停止原因，供停机后通过RAM区分正常完成与首个失败层。 */
typedef enum
{
  CONSTANT_HOLD_STOP_NONE = 0U,
  CONSTANT_HOLD_STOP_COMPLETE,
  CONSTANT_HOLD_STOP_IMU_NOT_READY,
  CONSTANT_HOLD_STOP_HEADING_ARM,
  CONSTANT_HOLD_STOP_HEADING_FAULT,
  CONSTANT_HOLD_STOP_MOTOR_SUBMIT
} ConstantHoldStopReason;

/**
 * @brief 一次性恒速HOLD的只读运行诊断。
 *
 * 所有时间均为HAL毫秒计数；速度均为RPM。该结构只提供数字证据，不替代
 * 轮向、纠偏极性、30°恢复能力或地面轨迹的实物观察。
 */
typedef struct
{
  uint32_t state;
  uint32_t stop_reason;
  uint32_t state_started_ms;
  uint32_t run_started_ms;
  uint32_t last_applied_control_update;
  uint32_t command_submissions;
  uint32_t completed;
  float base_forward_rpm;
  float last_heading_w_rpm;
} ConstantHoldDiagnostics;

volatile ConstantHoldDiagnostics constant_hold_diag;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if APP_PROFILE_STEP_TEST
/**
 * @brief 停止全部电机并清除有限脉冲上限。
 *
 * 本测试不拥有路线或底盘状态；正常完成、启动失败和超时都复用同一全停出口。
 */
static void StepTest_Stop(void)
{
  uint32_t index;

  Motor_StopAll();
  for (index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index)
  {
    Motor_SetStepLimit((MotorId)index, 0U);
  }
}

/**
 * @brief 从统一脉冲数和RPM计算四路测试的理论时长。
 * @return 理论时长，单位毫秒。
 */
static uint32_t StepTest_ExpectedTimeMs(void)
{
  return (uint32_t)(((float)MOTOR_TEST_STEPS * 60000.0f) /
                    ((float)MOTOR_STEPS_PER_REV * MOTOR_TEST_RPM) + 0.5f);
}

/**
 * @brief 稳定窗口结束后只提交一次四路有限脉冲动作。
 *
 * 该画像不等待板载按键；进入终态锁存后不能再次提交动作。
 */
static void StepTest_StartRun(void)
{
  uint32_t index;

  StepTest_Stop();
  for (index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index)
  {
    Motor_ResetStepCount((MotorId)index);
    Motor_SetStepLimit((MotorId)index, MOTOR_TEST_STEPS);
  }
  if (Motor_SetFourRpm(MOTOR_TEST_RPM,
                       MOTOR_TEST_RPM,
                       MOTOR_TEST_RPM,
                       MOTOR_TEST_RPM) != HAL_OK)
  {
    StepTest_Stop();
    motor_test_state = STEP_TEST_STOP_LATCHED;
    return;
  }

  motor_test_state_started_at = HAL_GetTick();
  motor_test_state = STEP_TEST_RUN_ONCE;
}

/**
 * @brief 在四路驱动器全部失能时进入安全启动状态。
 */
static void StepTest_Init(void)
{
  StepTest_Stop();
  motor_test_state_started_at = HAL_GetTick();
  motor_test_state = STEP_TEST_BOOT_SAFE;
}

/**
 * @brief 非阻塞推进一次性脉冲测试。
 *
 * 状态固定为BOOT_SAFE→STABILIZE→RUN_ONCE→STOP_LATCHED。正常完成、总超时
 * 或批量启动失败都会进入同一停车终态；只有硬件复位才允许重新运行。
 */
static void StepTest_Service(void)
{
  uint32_t index;
  uint8_t complete;

  switch (motor_test_state)
  {
    case STEP_TEST_BOOT_SAFE:
      StepTest_Stop();
      motor_test_state_started_at = HAL_GetTick();
      motor_test_state = STEP_TEST_STABILIZE;
      break;

    case STEP_TEST_STABILIZE:
      if ((HAL_GetTick() - motor_test_state_started_at) >= MOTOR_TEST_STABILIZE_MS)
      {
        StepTest_StartRun();
      }
      break;

    case STEP_TEST_RUN_ONCE:
      complete = 1U;
      for (index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index)
      {
        if (Motor_GetStepCount((MotorId)index) < MOTOR_TEST_STEPS)
        {
          complete = 0U;
        }
      }
      if ((complete != 0U) ||
          ((HAL_GetTick() - motor_test_state_started_at) >=
           (StepTest_ExpectedTimeMs() + MOTOR_TEST_TIMEOUT_MARGIN_MS)))
      {
        StepTest_Stop();
        motor_test_state = STEP_TEST_STOP_LATCHED;
      }
      break;

    case STEP_TEST_STOP_LATCHED:
    default:
      break;
  }
}
#endif

#if APP_PROFILE_CONSTANT_HEADING_HOLD
/**
 * @brief 停止恒速HOLD并进入不可自动恢复的终态。
 * @param reason 正常完成或首个失败原因。
 *
 * 本函数同时清零控制器历史和四轮输出。任何故障、运行超时或正常结束都走
 * 同一停车路径，避免某个失败分支遗漏失能。STOP_LATCHED不会自动重新启动。
 */
static void ConstantHold_Stop(ConstantHoldStopReason reason)
{
  Chassis_Stop();
  ChassisHeading_EndMotion();
  constant_hold_diag.last_heading_w_rpm = 0.0f;
  constant_hold_diag.stop_reason = (uint32_t)reason;
  constant_hold_diag.completed =
      (reason == CONSTANT_HOLD_STOP_COMPLETE) ? 1U : 0U;
  constant_hold_diag.state = CONSTANT_HOLD_STOP_LATCHED;
  constant_hold_diag.state_started_ms = HAL_GetTick();
}

/**
 * @brief 初始化恒速HOLD画像并保证四路电机保持失能。
 *
 * 初始化后先进入BOOT_SAFE；不会因本函数调用而启动电机。只有MS901M完成
 * 建零、航向目标锁定且最短稳定时间满足后，Service才允许进入RUN_ONCE。
 */
static void ConstantHold_Init(void)
{
  const ConstantHoldDiagnostics initial = {0};

  Chassis_Stop();
  ChassisHeading_EndMotion();
  constant_hold_diag = initial;
  constant_hold_diag.state = CONSTANT_HOLD_BOOT_SAFE;
  constant_hold_diag.state_started_ms = HAL_GetTick();
  constant_hold_diag.base_forward_rpm = CONSTANT_HOLD_FORWARD_RPM;
}

/**
 * @brief 按非阻塞状态机推进一次恒速、纯P航向保持测试。
 *
 * 状态顺序固定为BOOT_SAFE→STABILIZE→RUN_ONCE→STOP_LATCHED。RUN_ONCE
 * 期间基础纵向速度始终为100 RPM，不调用Chassis_SINAccel或任何斜坡函数；
 * 只有ChassisHeading_Compute()产生新的10ms控制更新时，才通过
 * Chassis_MoveConstant()重新提交四轮恒频STEP命令。
 *
 * IMU未就绪、航向健康检查失败、40°保护线触发、调度超时或任一四轮提交
 * 失败都会立即全停并锁存。正常运行满3秒同样全停，且绝不自动循环。
 */
static void ConstantHold_Service(void)
{
  uint32_t now = HAL_GetTick();
  float heading_w = 0.0f;

  switch ((ConstantHoldState)constant_hold_diag.state)
  {
    case CONSTANT_HOLD_BOOT_SAFE:
      Chassis_Stop();
      constant_hold_diag.state_started_ms = now;
      constant_hold_diag.state = CONSTANT_HOLD_STABILIZE;
      break;

    case CONSTANT_HOLD_STABILIZE:
      Chassis_Stop();
      if ((uint32_t)(now - constant_hold_diag.state_started_ms) >=
          CONSTANT_HOLD_START_TIMEOUT_MS)
      {
        ConstantHold_Stop(CONSTANT_HOLD_STOP_IMU_NOT_READY);
        break;
      }
      if ((uint32_t)(now - constant_hold_diag.state_started_ms) <
          CONSTANT_HOLD_STABILIZE_MS || imu_ms901m_diag.zero_ready == 0U)
      {
        break;
      }
      if (ChassisHeading_ArmCurrent() == 0U)
      {
        ConstantHold_Stop(CONSTANT_HOLD_STOP_HEADING_ARM);
        break;
      }
      if (ChassisHeading_Compute(&heading_w) == 0U)
      {
        ConstantHold_Stop(CONSTANT_HOLD_STOP_HEADING_FAULT);
        break;
      }
      if (Chassis_MoveConstant(0.0f, CONSTANT_HOLD_FORWARD_RPM, heading_w) == 0U)
      {
        ConstantHold_Stop(CONSTANT_HOLD_STOP_MOTOR_SUBMIT);
        break;
      }
      constant_hold_diag.last_heading_w_rpm = heading_w;
      constant_hold_diag.last_applied_control_update =
          chassis_heading_diag.control_updates;
      constant_hold_diag.command_submissions = 1U;
      constant_hold_diag.run_started_ms = now;
      constant_hold_diag.state_started_ms = now;
      constant_hold_diag.state = CONSTANT_HOLD_RUN_ONCE;
      break;

    case CONSTANT_HOLD_RUN_ONCE:
      if ((uint32_t)(now - constant_hold_diag.run_started_ms) >=
          CONSTANT_HOLD_RUN_MS)
      {
        ConstantHold_Stop(CONSTANT_HOLD_STOP_COMPLETE);
        break;
      }
      if (ChassisHeading_Compute(&heading_w) == 0U)
      {
        ConstantHold_Stop(CONSTANT_HOLD_STOP_HEADING_FAULT);
        break;
      }
      if (chassis_heading_diag.control_updates !=
          constant_hold_diag.last_applied_control_update)
      {
        if (Chassis_MoveConstant(0.0f, CONSTANT_HOLD_FORWARD_RPM, heading_w) == 0U)
        {
          ConstantHold_Stop(CONSTANT_HOLD_STOP_MOTOR_SUBMIT);
          break;
        }
        constant_hold_diag.last_heading_w_rpm = heading_w;
        constant_hold_diag.last_applied_control_update =
            chassis_heading_diag.control_updates;
        constant_hold_diag.command_submissions++;
      }
      break;

    case CONSTANT_HOLD_STOP_LATCHED:
    default:
      Chassis_Stop();
      break;
  }
}
#endif

#if APP_PROFILE_VISION_DISTANCE_ALIGN
/**
 * @brief 初始化路线前的视觉对齐闸门。
 *
 * 视觉收帧、协议校验和横向控制器分别由已有模块拥有；本函数只建立
 * 任务层状态。此时全车保持停车，直到Service确认连续有效新帧。
 */
static void VisionAlign_Init(void)
{
  VisionControllerConfig config;

  VisionController_GetDefaultConfig(&config);
  config.raw_dx_bias = 0;
  config.raw_dx_to_vehicle_mm = VISION_ALIGN_DX_TO_X_MM;
  config.enter_deadband = VISION_ALIGN_ENTER_DEADBAND_DX;
  config.exit_deadband = VISION_ALIGN_EXIT_DEADBAND_DX;
  (void)VisionController_Init(&vision_align_controller, &config);
  vision_align_state = VISION_ALIGN_BOOT_SAFE;
  vision_align_started_ms = HAL_GetTick();
  vision_align_last_center_sample_id = 0U;
  vision_align_center_sample_count = 0U;
  Chassis_Stop();
}

/**
 * @brief 将控制器的已冻结粗物理X误差量化成一次有限厘米移动。
 *
 * error_mm的正负继承固定帧dx=2*(474-M)的纠偏方向：正值请求X+。
 * 不足1cm的非零误差向远离零取整，距离始终限在3cm，不能演变成连续
 * 速度控制。
 */
static uint8_t VisionAlign_BuildMove(const VisionControllerReport *report,
                                     int16_t *x_rpm, uint16_t *distance_cm)
{
  int32_t signed_cm;

  if ((report == 0) || (x_rpm == 0) || (distance_cm == 0) ||
      report->reason != VISION_CONTROLLER_REASON_TRACKING)
  {
    return 0U;
  }
  if (report->error_mm > 0.0f)
  {
    signed_cm = (int32_t)((report->error_mm + 9.999f) / 10.0f);
  }
  else if (report->error_mm < 0.0f)
  {
    signed_cm = -(int32_t)(((-report->error_mm + 9.999f) / 10.0f));
  }
  else
  {
    return 0U;
  }
  if (signed_cm > (int32_t)VISION_ALIGN_MAX_DISTANCE_CM)
    signed_cm = (int32_t)VISION_ALIGN_MAX_DISTANCE_CM;
  else if (signed_cm < -(int32_t)VISION_ALIGN_MAX_DISTANCE_CM)
    signed_cm = -(int32_t)VISION_ALIGN_MAX_DISTANCE_CM;
  *x_rpm = (signed_cm > 0) ? VISION_ALIGN_X_SPEED_RPM : -VISION_ALIGN_X_SPEED_RPM;
  *distance_cm = (uint16_t)((signed_cm > 0) ? signed_cm : -signed_cm);
  return 1U;
}

/**
 * @brief 将已验帧的视觉观测交给纯控制器；本函数不直接操作电机。
 */
static uint8_t VisionAlign_UpdateController(uint32_t now_ms,
                                             VisionControllerReport *report,
                                             uint32_t *sample_id)
{
  VisionObservation latest;
  VisionControllerObservation observation;

  if ((report == 0) || (sample_id == 0)) return 0U;
  if (Vision_GetLatest(&latest) == 0U)
  {
    (void)VisionController_Update(&vision_align_controller, 1U, now_ms, 0, report);
    *sample_id = 0U;
    return 0U;
  }

  observation.sample_id = latest.sample_id;
  observation.received_ms = latest.received_ms;
  observation.dx = latest.dx;
  observation.valid = latest.valid;
  (void)VisionController_Update(&vision_align_controller, 1U, now_ms,
                                &observation, report);
  *sample_id = latest.sample_id;
  return 1U;
}

/**
 * @brief 执行STOP→连续新帧→有界纯X→连续居中确认→STOP的任务层状态机。
 *
 * 短暂失帧会立即停车并回到采集；总超时或连续运动超时均锁存失败，
 * 不会继续路线，也不会自动重启。
 */
static void VisionAlign_Service(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t sample_id = 0U;
  uint8_t have_observation;
  int16_t x_rpm;
  uint16_t distance_cm;
  ChassisMoveStatus move_status;
  VisionControllerReport report;

  if (vision_align_state == VISION_ALIGN_COMPLETE)
  {
    return;
  }
  if (vision_align_state == VISION_ALIGN_FAILED)
  {
    Chassis_Stop();
    return;
  }

  have_observation = VisionAlign_UpdateController(now_ms, &report, &sample_id);
  if ((uint32_t)(now_ms - vision_align_started_ms) > VISION_ALIGN_TOTAL_TIMEOUT_MS)
  {
    Chassis_Stop();
    vision_align_state = VISION_ALIGN_FAILED;
    return;
  }

  switch (vision_align_state)
  {
    case VISION_ALIGN_BOOT_SAFE:
      Chassis_Stop();
      vision_align_state = VISION_ALIGN_ACQUIRE;
      break;

    case VISION_ALIGN_ACQUIRE:
      Chassis_Stop();
      if ((have_observation != 0U) && (report.control_valid != 0U))
      {
        if (report.reason == VISION_CONTROLLER_REASON_CENTERED)
        {
          vision_align_center_sample_count = 0U;
          vision_align_last_center_sample_id = 0U;
          vision_align_state = VISION_ALIGN_CENTER_CONFIRM;
        }
        else if (report.reason == VISION_CONTROLLER_REASON_TRACKING)
        {
          if (VisionAlign_BuildMove(&report, &x_rpm, &distance_cm) == 0U ||
              Chassis_MoveCm(x_rpm, 0, distance_cm) == 0U)
          {
            Chassis_Stop();
            vision_align_state = VISION_ALIGN_FAILED;
          }
          else
          {
            vision_align_state = VISION_ALIGN_MOVING;
          }
        }
      }
      break;

    case VISION_ALIGN_MOVING:
      if ((have_observation == 0U) || (report.control_valid == 0U))
      {
        Chassis_Stop();
        vision_align_state = VISION_ALIGN_ACQUIRE;
      }
      else
      {
        move_status = Chassis_Service();
        if (move_status == CHASSIS_MOVE_FAILED)
        {
          Chassis_Stop();
          vision_align_state = VISION_ALIGN_FAILED;
        }
        else if (move_status == CHASSIS_MOVE_COMPLETE)
        {
          Chassis_Stop();
          VisionController_Reset(&vision_align_controller);
          vision_align_state = VISION_ALIGN_ACQUIRE;
        }
      }
      break;

    case VISION_ALIGN_CENTER_CONFIRM:
      Chassis_Stop();
      if ((have_observation == 0U) || (report.control_valid == 0U))
      {
        vision_align_state = VISION_ALIGN_ACQUIRE;
      }
      else if (report.reason == VISION_CONTROLLER_REASON_TRACKING)
      {
        if (VisionAlign_BuildMove(&report, &x_rpm, &distance_cm) == 0U ||
            Chassis_MoveCm(x_rpm, 0, distance_cm) == 0U)
        {
          Chassis_Stop();
          vision_align_state = VISION_ALIGN_FAILED;
        }
        else
        {
          vision_align_state = VISION_ALIGN_MOVING;
        }
      }
      else if (report.reason == VISION_CONTROLLER_REASON_CENTERED &&
               sample_id != vision_align_last_center_sample_id)
      {
        vision_align_last_center_sample_id = sample_id;
        ++vision_align_center_sample_count;
        if (vision_align_center_sample_count >= VISION_ALIGN_CENTER_CONFIRM_SAMPLES)
        {
          Chassis_Stop();
          vision_align_state = VISION_ALIGN_COMPLETE;
        }
      }
      break;

    default:
      Chassis_Stop();
      vision_align_state = VISION_ALIGN_FAILED;
      break;
  }
}
#endif

/* USER CODE END 0 */

/**
 * @brief 初始化平台并只启动当前应用配置所拥有的模块。
 *
 * 一次性脉冲档只运行四路STEP/DIR/EN证明；IMU短回归档初始化同一电机
 * 后端后持续全停；恒速HOLD档只运行一次100 RPM测试；厘米路线档在IMU
 * 建零后按数据表顺序运行单轴厘米段，并在每段停车完成后才进入下一段。
 * 视觉、主动转向和USART3电机协议均与当前画像隔离。
 * @retval int 本嵌入式入口不返回。
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

#if APP_PROFILE_STEP_TEST
  /* 独立四路脉冲测试入口：本工程不初始化陀螺仪、视觉、底盘路线或机械臂。 */
  if (Motor_Init() != HAL_OK)
  {
    Error_Handler();
  }
  StepTest_Init();
#elif APP_PROFILE_IMU_SHORT_REGRESSION
  if (Motor_Init() != HAL_OK)
  {
    Error_Handler();
  }
  Motor_StopAll();
  app_runtime_profile_marker = APP_IMU_SHORT_REGRESSION_MARKER;
  ImuMs901m_Init();
#elif APP_PROFILE_CONSTANT_HEADING_HOLD
  if (Motor_Init() != HAL_OK)
  {
    Error_Handler();
  }
  Motor_StopAll();
  ImuMs901m_Init();
  ChassisHeading_Init();
  Chassis_Init();
  app_runtime_profile_marker = APP_CONSTANT_HEADING_HOLD_MARKER;
  ConstantHold_Init();
#elif APP_PROFILE_CM_ROUTE_TEST

  if (Motor_Init() != HAL_OK)
  {
    Error_Handler();
  }
  Motor_StopAll();
  ImuMs901m_Init();
  ChassisHeading_Init();
  Chassis_Init();
  app_runtime_profile_marker = APP_CM_ROUTE_TEST_MARKER;
  Vision_Init();
  Vision_EnableReceiveOnly();

  if (Mission_Init() == 0U)
  {
    Chassis_Stop();
    Error_Handler();
  }

#elif APP_PROFILE_VISION_DISTANCE_ALIGN

  if (Motor_Init() != HAL_OK)
  {
    Error_Handler();
  }
  Motor_StopAll();
  ImuMs901m_Init();
  ChassisHeading_Init();
  Chassis_Init();
  app_runtime_profile_marker = APP_CM_ROUTE_TEST_MARKER;
  Vision_Init();
  VisionAlign_Init();

#elif APP_PROFILE_PASSIVE_VISION_REPORT
  /* Passive report image: vision and UART4 diagnostics only, no motor or chassis init. */
  Vision_Init();
  VisionControllerObserver_Init();

#elif APP_PROFILE_ARM_TUNE || APP_PROFILE_ARM_GRASP_CAPTURE
  /* 机械臂调试画像只拥有I2C2/PCA9685；默认五关节均禁用，绝不自动动作。 */
  if (ArmTune_Init() == 0U)
  {
    Error_Handler();
  }
#if APP_PROFILE_ARM_GRASP_CAPTURE
  if (ArmGraspStagePanel_Request() == 0U)
#else
  if (ArmTune_RequestManualPose() == 0U)
#endif
  {
    Error_Handler();
  }

#else
  ImuMs901m_Init();
  ChassisHeading_Init();

#if VISION_LINK_TEST_ACTIVE
  Vision_Init();

#endif

#if !IMU_ONLY_TEST_ACTIVE && !VISION_LINK_TEST_ACTIVE
  StepMotor_Init();
  Chassis_Init();
  MotorTest_Init();

#endif
#endif

  while (1)
  {
    /*
     * APP_PROFILE_*在编译期保证只有一个值为1，因此下面只会有一个分支
     * 进入最终固件；它不是运行时轮询所有模式，也不会执行完一支再切下一支。
     */
#if APP_PROFILE_STEP_TEST
    /* 四路STEP/DIR一次性有限动作验证：只服务脉冲测试状态机。 */
    StepTest_Service();
    HAL_Delay(1U);
#elif APP_PROFILE_IMU_SHORT_REGRESSION
    /* 无动力IMU短回归：持续强制停车，只处理MS901M数据。 */
    Motor_StopAll();
    ImuMs901m_Service();
    HAL_Delay(1U);
#elif APP_PROFILE_CONSTANT_HEADING_HOLD
    /* 恒速航向保持候选：先更新IMU，再让HOLD状态机消费最新航向。 */
    ImuMs901m_Service();
    ConstantHold_Service();
    HAL_Delay(1U);
#elif APP_PROFILE_CM_ROUTE_TEST
    /* 纯路线只接收二维码结果，不发送模式帧或运行视觉对齐；路线状态机继续负责IMU建零、航向与超时停车。 */
    ImuMs901m_Service();
    Vision_Service();

    if (Mission_Service() == MISSION_FAILED)
    {
      Chassis_Stop();
      Error_Handler();
    }
    HAL_Delay(1U);
#elif APP_PROFILE_VISION_DISTANCE_ALIGN
    ImuMs901m_Service();
    Vision_Service();
    VisionAlign_Service();
    if (vision_align_state == VISION_ALIGN_COMPLETE)
    {
      Chassis_Stop();
    }
    HAL_Delay(1U);
#elif APP_PROFILE_PASSIVE_VISION_REPORT
    /* Only observe K230 frames and emit the controller report; never command motion. */
    Vision_Service();
    VisionControllerObserver_Service(HAL_GetTick());
    HAL_Delay(1U);
#elif APP_PROFILE_ARM_TUNE || APP_PROFILE_ARM_GRASP_CAPTURE
    /* 探测画像只服务单关节；抓球画像只服务阶段面板的掩码请求。 */
    ArmTune_Service(HAL_GetTick());
    HAL_Delay(1U);
#else
    /* 历史综合档：依次服务IMU、航向、可选视觉和旧MotorTest编排。 */
    ImuMs901m_Service();
    ChassisHeading_Service();
#if VISION_LINK_TEST_ACTIVE
    Vision_Service();
#endif
#if !IMU_ONLY_TEST_ACTIVE && !VISION_LINK_TEST_ACTIVE
    MotorTest_Service();
    if (!MotorTest_NeedsFastService())
    {
      HAL_Delay(1U);
    }
#else
    HAL_Delay(1U);
#endif
#endif
  }
}

/**
 * @brief 配置 HSI 直驱的系统时钟，提供脉冲模块可查询的 APB1 时钟来源。
 *
 * 电机模块自行根据实际 APB1 时钟计算预分频；本函数不包含任何电机引脚、
 * 定时器或运动策略。时钟配置失败时立即进入 Error_Handler，不能启动电机。
 * @return 无。
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief 初始化与应用无关的安全 GPIO 默认状态。
 *
 * PD14受外部电路控制，启动时保持低电平；脉冲测试的STEP/DIR/EN由专用驱动
 * 模块初始化，避免把板级细节耦合进组合入口。
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  gpio.Pin = GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &gpio);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief 关闭中断并永久停机，防止初始化失败后继续进入任何测试流程。
  * @return 无；该函数不返回。
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
#ifdef USE_FULL_ASSERT
/**
  * @brief 预留 HAL 参数断言的诊断入口；默认不改变执行器状态或输出串口信息。
  * @param file 触发断言的源码文件名。
  * @param line 触发断言的源码行号。
  * @retval 无。
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

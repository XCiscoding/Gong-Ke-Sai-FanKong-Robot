#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>
/* Same X layout and equal-weight initial settings as the junior's Chassis.c. */
#define CHASSIS_WHEEL_PATTERN_X 1U
/* Body +X points right and +Y points forward. On 2026-10-03, after the
 * wheel installation was corrected, reference vx=-50 produced physical +X.
 * Calibrate only X at the reference-mixer boundary; do not couple W to it. */
#define CHASSIS_X_INPUT_POLARITY (-1.0f)

/* The direct-step donor numbers its physical wheels as M1=left-front,
 * M2=right-front, M3=left-rear and M4=right-rear.  Direction-pin polarity is
 * already absorbed by motor_board_config.h, so wheel RPM signs below must not
 * repeat the old USART3 bus-ID polarity conversion. */
#define CHASSIS_DIRECT_X_INPUT_POLARITY (1.0f)
#define CHASSIS_DIRECT_MAX_WHEEL_RPM    (120.0f)

/**
 * @brief 直脉冲后端一次混控计算得到的四路有符号轮速。
 *
 * 单位均为RPM；正负号只表达供体轮序下的转向语义，DIR引脚的物理极性由
 * motor_board_config.h吸收。调用方不得再次按旧USART3电机ID翻转符号。
 */
typedef struct {
    float motor1_rpm;
    float motor2_rpm;
    float motor3_rpm;
    float motor4_rpm;
} ChassisDirectWheelRpm;
/** @brief 供独立基础动作测试使用的车体运动方向。 */
typedef enum {
    CHASSIS_FORWARD, CHASSIS_BACKWARD, CHASSIS_LEFT, CHASSIS_RIGHT,
    CHASSIS_ROTATE_LEFT, CHASSIS_ROTATE_RIGHT
} ChassisMotion;
/**
 * @brief 初始化底盘执行层并保持所有电机停止。
 *
 * 本函数不初始化IMU、不锁定航向，也不会自动开始任何移动。
 */
void Chassis_Init(void);

/**
 * @brief 立即停止当前底盘输出。
 *
 * 直脉冲后端会停止PWM并失能全部四路；旧USART3后端仅清空历史目标缓存。
 * 恒速HOLD测试的正常结束、超时和全部故障都必须收敛到本函数。
 */
void Chassis_Stop(void);

/**
 * @brief 将路线XY、视觉X附加量和航向W混合为四路直脉冲轮速。
 * @param route_vx 路线横向基础速度，单位RPM，车体向右为正。
 * @param route_vy 路线纵向基础速度，单位RPM，车体向前为正。
 * @param vision_delta_vx 视觉层仅允许贡献的横向附加量，单位RPM。
 * @param heading_w 航向控制器旋转修正量，单位RPM；物理正方向待实车验证。
 * @param wheel_rpm 输出四路轮速；不得为空。
 * @return 1表示输入有效且完成混控；0表示参数无效，未产生硬件动作。
 *
 * 超过CHASSIS_DIRECT_MAX_WHEEL_RPM时，四路按同一比例缩放，禁止单轮裁剪
 * 破坏平移与旋转的比例关系。
 */
uint8_t Chassis_ComputeDirectWheelRpm(float route_vx, float route_vy,
                                      float vision_delta_vx, float heading_w,
                                      ChassisDirectWheelRpm *wheel_rpm);

/**
 * @brief 立即提交一次底盘恒定速度命令，不生成任何加速或减速曲线。
 * @param route_vx 路线横向基础速度，单位RPM，车体向右为正。
 * @param route_vy 路线纵向基础速度，单位RPM，车体向前为正。
 * @param vision_delta_vx 视觉横向附加量，单位RPM；本轮HOLD固定传0。
 * @param heading_w 航向旋转修正量，单位RPM。
 * @return 1表示四路命令提交成功；0表示参数或驱动失败且已全停。
 *
 * 本函数对应供体“匀速移动”的执行语义，但被改造成非阻塞单次提交：
 * 不调用Chassis_SINAccel，不包含Delay，不拥有持续时间。上层状态机负责按
 * 控制周期调用、决定何时停止，并处理IMU与总超时。
 */
uint8_t Chassis_ApplyCommand(float route_vx, float route_vy,
                             float vision_delta_vx, float heading_w);

/**
 * @brief 提交一次不含视觉附加量的恒速车体命令。
 * @param vx 横向恒定基础速度，单位RPM，向右为正。
 * @param vy 纵向恒定基础速度，单位RPM，向前为正。
 * @param heading_w 纯P航向环生成的旋转修正量，单位RPM。
 * @return 1表示提交成功；0表示失败且已全停。
 *
 * 这是本轮复刻后供恒速HOLD使用的明确入口。它不会自动循环、不会自动
 * 加减速，也不会在停止后自行重新启动。
 */
uint8_t Chassis_MoveConstant(float vx, float vy, float heading_w);

/**
 * @brief 兼容历史调用的车体逆运动学入口。
 * @param v_x 横向速度，单位RPM。
 * @param v_y 纵向速度，单位RPM。
 * @param w 旋转速度，单位RPM。
 * @return 1表示目标已接受；0表示输入或后端失败。
 */
uint8_t Chassis_InverseMotionControl(float v_x, float v_y, float w);

/**
 * @brief 兼容旧路线层的坐标旋转与航向闭环入口。
 * @param vx 路线坐标横向速度，单位RPM。
 * @param vy 路线坐标纵向速度，单位RPM。
 * @param theta_degrees 路线坐标到车体坐标的角度差，单位度。
 * @return 1表示命令提交成功；0表示航向或执行失败并停车。
 */
uint8_t Chassis_SetSpeed(float vx, float vy, float theta_degrees);

/**
 * @brief 为独立方向测试生成一组固定车体速度目标。
 * @param motion 前进、后退、横移或原地旋转枚举。
 * @param speed_rpm 无符号速度幅值，单位RPM。
 * @return 1表示目标已接受；0表示枚举无效并停车。
 */
uint8_t Chassis_PrepareMotion(ChassisMotion motion, uint16_t speed_rpm);
#endif

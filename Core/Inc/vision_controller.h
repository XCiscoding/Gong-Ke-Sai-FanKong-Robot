#ifndef VISION_CONTROLLER_H
#define VISION_CONTROLLER_H

#include <stdint.h>

/* Pure controller: protocol parsing, task adoption and chassis commands stay
 * outside this module. */
typedef struct {
    uint32_t sample_id;
    uint32_t received_ms;
    int16_t dx;
    uint8_t valid;
} VisionControllerObservation;

typedef struct {
    /* 原始相机dx的安装零偏与到车体横向误差的粗比例。 */
    int16_t raw_dx_bias;
    float raw_dx_to_vehicle_mm;
    uint16_t enter_deadband;
    uint16_t exit_deadband;
    float kp_rpm_per_mm;
    float max_delta_vx_rpm;
    float max_slew_rpm_per_s;
    uint32_t stale_timeout_ms;
    uint8_t reacquire_samples;
    int8_t error_to_delta_vx_polarity;
} VisionControllerConfig;

typedef enum {
    VISION_CONTROLLER_DISABLED = 0,
    VISION_CONTROLLER_REACQUIRING,
    VISION_CONTROLLER_TRACKING,
    VISION_CONTROLLER_LOST
} VisionControllerState;

typedef enum {
    VISION_CONTROLLER_REASON_DISABLED = 0,
    VISION_CONTROLLER_REASON_REACQUIRING,
    VISION_CONTROLLER_REASON_CENTERED,
    VISION_CONTROLLER_REASON_TRACKING,
    VISION_CONTROLLER_REASON_INVALID_OBSERVATION,
    VISION_CONTROLLER_REASON_STALE_OBSERVATION
} VisionControllerReason;

typedef struct {
    float error_mm;
    float delta_vx_rpm;
    uint8_t control_valid;
    VisionControllerState state;
    VisionControllerReason reason;
} VisionControllerReport;

#define VISION_CONTROLLER_DX_HISTORY_LENGTH 3U

typedef struct {
    VisionControllerConfig config;
    int16_t dx_history[VISION_CONTROLLER_DX_HISTORY_LENGTH];
    uint32_t last_sample_id;
    uint32_t last_observation_ms;
    uint32_t last_update_ms;
    float applied_delta_vx_rpm;
    uint8_t dx_count;
    uint8_t dx_next;
    uint8_t reacquire_count;
    uint8_t have_sample;
    uint8_t have_update_time;
    uint8_t in_deadband;
} VisionController;

void VisionController_GetDefaultConfig(VisionControllerConfig *config);
uint8_t VisionController_ConfigIsValid(const VisionControllerConfig *config);
uint8_t VisionController_Init(VisionController *controller,
                              const VisionControllerConfig *config);
void VisionController_Reset(VisionController *controller);
uint8_t VisionController_Update(VisionController *controller,
                                uint8_t enabled,
                                uint32_t now_ms,
                                const VisionControllerObservation *observation,
                                VisionControllerReport *report);

#endif /* VISION_CONTROLLER_H */

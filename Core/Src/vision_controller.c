#include "vision_controller.h"

static float VisionController_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float VisionController_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void VisionController_WriteReport(VisionControllerReport *report,
                                         float error_mm,
                                         float delta_vx_rpm,
                                         uint8_t control_valid,
                                         VisionControllerState state,
                                         VisionControllerReason reason)
{
    report->error_mm = error_mm;
    report->delta_vx_rpm = delta_vx_rpm;
    report->control_valid = control_valid;
    report->state = state;
    report->reason = reason;
}

static void VisionController_ClearRuntime(VisionController *controller)
{
    uint8_t index;
    for (index = 0U; index < VISION_CONTROLLER_DX_HISTORY_LENGTH; ++index)
        controller->dx_history[index] = 0;
    controller->last_sample_id = 0U;
    controller->last_observation_ms = 0U;
    controller->last_update_ms = 0U;
    controller->applied_delta_vx_rpm = 0.0f;
    controller->dx_count = 0U;
    controller->dx_next = 0U;
    controller->reacquire_count = 0U;
    controller->have_sample = 0U;
    controller->have_update_time = 0U;
    controller->in_deadband = 1U;
}

static void VisionController_AddDx(VisionController *controller, int16_t dx)
{
    controller->dx_history[controller->dx_next] = dx;
    controller->dx_next = (uint8_t)((controller->dx_next + 1U) %
                                    VISION_CONTROLLER_DX_HISTORY_LENGTH);
    if (controller->dx_count < VISION_CONTROLLER_DX_HISTORY_LENGTH)
        ++controller->dx_count;
}

static int16_t VisionController_MedianDx(const VisionController *controller)
{
    int16_t ordered[VISION_CONTROLLER_DX_HISTORY_LENGTH];
    uint8_t count = controller->dx_count;
    uint8_t outer;
    uint8_t inner;
    for (outer = 0U; outer < count; ++outer) ordered[outer] = controller->dx_history[outer];
    for (outer = 0U; outer < count; ++outer) {
        for (inner = (uint8_t)(outer + 1U); inner < count; ++inner) {
            if (ordered[inner] < ordered[outer]) {
                const int16_t temporary = ordered[outer];
                ordered[outer] = ordered[inner];
                ordered[inner] = temporary;
            }
        }
    }
    return ordered[count / 2U];
}

static float VisionController_ApplySlew(VisionController *controller,
                                        float target_rpm, uint32_t now_ms)
{
    float maximum_change;
    float delta;
    uint32_t elapsed_ms;
    if (controller->have_update_time == 0U) {
        controller->last_update_ms = now_ms;
        controller->have_update_time = 1U;
        return controller->applied_delta_vx_rpm;
    }
    elapsed_ms = (uint32_t)(now_ms - controller->last_update_ms);
    controller->last_update_ms = now_ms;
    maximum_change = controller->config.max_slew_rpm_per_s * ((float)elapsed_ms / 1000.0f);
    delta = VisionController_Clamp(target_rpm - controller->applied_delta_vx_rpm,
                                   -maximum_change, maximum_change);
    controller->applied_delta_vx_rpm += delta;
    return controller->applied_delta_vx_rpm;
}

void VisionController_GetDefaultConfig(VisionControllerConfig *config)
{
    if (config == 0) return;
    config->raw_dx_bias = 0;
    config->raw_dx_to_vehicle_mm = 1.0f;
    config->enter_deadband = 40U;
    config->exit_deadband = 50U;
    config->kp_rpm_per_mm = 0.05f;
    config->max_delta_vx_rpm = 12.0f;
    config->max_slew_rpm_per_s = 30.0f;
    config->stale_timeout_ms = 200U;
    config->reacquire_samples = 3U;
    config->error_to_delta_vx_polarity = 1;
}

uint8_t VisionController_ConfigIsValid(const VisionControllerConfig *config)
{
    if (config == 0 || config->enter_deadband > config->exit_deadband ||
        config->raw_dx_to_vehicle_mm <= 0.0f || config->kp_rpm_per_mm < 0.0f ||
        config->max_delta_vx_rpm < 0.0f ||
        config->max_slew_rpm_per_s < 0.0f || config->stale_timeout_ms == 0U ||
        config->reacquire_samples == 0U ||
        (config->error_to_delta_vx_polarity != 1 &&
         config->error_to_delta_vx_polarity != -1))
        return 0U;
    return 1U;
}

uint8_t VisionController_Init(VisionController *controller,
                              const VisionControllerConfig *config)
{
    VisionControllerConfig defaults;
    if (controller == 0) return 0U;
    if (config == 0) {
        VisionController_GetDefaultConfig(&defaults);
        controller->config = defaults;
    } else if (VisionController_ConfigIsValid(config) != 0U) {
        controller->config = *config;
    } else {
        VisionController_GetDefaultConfig(&defaults);
        controller->config = defaults;
        VisionController_ClearRuntime(controller);
        return 0U;
    }
    VisionController_ClearRuntime(controller);
    return 1U;
}

void VisionController_Reset(VisionController *controller)
{
    if (controller != 0) VisionController_ClearRuntime(controller);
}

uint8_t VisionController_Update(VisionController *controller, uint8_t enabled,
                                uint32_t now_ms,
                                const VisionControllerObservation *observation,
                                VisionControllerReport *report)
{
    float target_rpm;
    float filtered_dx;
    float error_mm;
    uint8_t is_new_sample;
    if (controller == 0 || report == 0) return 0U;
    if (enabled == 0U) {
        VisionController_Reset(controller);
        VisionController_WriteReport(report, 0.0f, 0.0f, 0U, VISION_CONTROLLER_DISABLED,
                                     VISION_CONTROLLER_REASON_DISABLED);
        return 1U;
    }
    if (observation == 0) {
        VisionController_Reset(controller);
        VisionController_WriteReport(report, 0.0f, 0.0f, 0U, VISION_CONTROLLER_LOST,
                                     VISION_CONTROLLER_REASON_INVALID_OBSERVATION);
        return 1U;
    }
    is_new_sample = (controller->have_sample == 0U ||
                     observation->sample_id != controller->last_sample_id) ? 1U : 0U;
    if (observation->valid == 0U) {
        VisionController_Reset(controller);
        VisionController_WriteReport(report, 0.0f, 0.0f, 0U, VISION_CONTROLLER_LOST,
                                     VISION_CONTROLLER_REASON_INVALID_OBSERVATION);
        return 1U;
    }
    if (is_new_sample != 0U) {
        controller->last_sample_id = observation->sample_id;
        controller->last_observation_ms = observation->received_ms;
        controller->have_sample = 1U;
        VisionController_AddDx(controller, observation->dx);
        if (controller->reacquire_count < controller->config.reacquire_samples)
            ++controller->reacquire_count;
    }
    if (controller->have_sample == 0U ||
        (uint32_t)(now_ms - controller->last_observation_ms) >
        controller->config.stale_timeout_ms) {
        VisionController_ClearRuntime(controller);
        VisionController_WriteReport(report, 0.0f, 0.0f, 0U, VISION_CONTROLLER_LOST,
                                     VISION_CONTROLLER_REASON_STALE_OBSERVATION);
        return 1U;
    }
    if (controller->reacquire_count < controller->config.reacquire_samples) {
        controller->applied_delta_vx_rpm = 0.0f;
        controller->last_update_ms = now_ms;
        controller->have_update_time = 1U;
        VisionController_WriteReport(report, 0.0f, 0.0f, 0U, VISION_CONTROLLER_REACQUIRING,
                                     VISION_CONTROLLER_REASON_REACQUIRING);
        return 1U;
    }
    filtered_dx = (float)VisionController_MedianDx(controller);
    error_mm = (filtered_dx - (float)controller->config.raw_dx_bias) *
               controller->config.raw_dx_to_vehicle_mm;
    if (controller->in_deadband != 0U) {
        if (VisionController_Abs(filtered_dx - (float)controller->config.raw_dx_bias) >
            (float)controller->config.exit_deadband)
            controller->in_deadband = 0U;
    } else if (VisionController_Abs(filtered_dx - (float)controller->config.raw_dx_bias) <=
               (float)controller->config.enter_deadband) {
        controller->in_deadband = 1U;
    }
    if (controller->in_deadband != 0U) {
        target_rpm = 0.0f;
        VisionController_WriteReport(report, error_mm,
                                     VisionController_ApplySlew(controller, target_rpm, now_ms),
                                     1U, VISION_CONTROLLER_TRACKING,
                                     VISION_CONTROLLER_REASON_CENTERED);
        return 1U;
    }
    target_rpm = error_mm * controller->config.kp_rpm_per_mm *
                 (float)controller->config.error_to_delta_vx_polarity;
    target_rpm = VisionController_Clamp(target_rpm, -controller->config.max_delta_vx_rpm,
                                        controller->config.max_delta_vx_rpm);
    VisionController_WriteReport(report, error_mm,
                                 VisionController_ApplySlew(controller, target_rpm, now_ms),
                                 1U, VISION_CONTROLLER_TRACKING,
                                 VISION_CONTROLLER_REASON_TRACKING);
    return 1U;
}

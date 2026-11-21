#include <math.h>
#include <stdio.h>
#include <string.h>

#include "vision_controller.h"

static int expect_true(int condition, const char *message)
{
    if (!condition) {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        return 0;
    }
    return 1;
}

static void feed(VisionController *controller, uint32_t sample_id,
                 uint32_t now_ms, int16_t dx, uint8_t valid,
                 VisionControllerReport *report)
{
    VisionControllerObservation observation;
    observation.sample_id = sample_id;
    observation.received_ms = now_ms;
    observation.dx = dx;
    observation.valid = valid;
    (void)VisionController_Update(controller, 1U, now_ms, &observation, report);
}

int main(void)
{
    VisionController controller;
    VisionControllerConfig config;
    VisionControllerReport report;

    (void)memset(&controller, 0, sizeof(controller));
    VisionController_GetDefaultConfig(&config);
    config.raw_dx_bias = 342;
    config.raw_dx_to_vehicle_mm = 0.353f;
    config.enter_deadband = 12U;
    config.exit_deadband = 18U;
    config.kp_rpm_per_mm = 0.40f;
    config.max_delta_vx_rpm = 8.0f;
    config.error_to_delta_vx_polarity = -1;
    if (!expect_true(VisionController_Init(&controller, &config) != 0U,
                     "rough profile initializes")) return 1;

    feed(&controller, 1U, 0U, 257, 1U, &report);
    feed(&controller, 2U, 100U, 257, 1U, &report);
    feed(&controller, 3U, 200U, 257, 1U, &report);
    if (!expect_true(report.control_valid != 0U &&
                     report.reason == VISION_CONTROLLER_REASON_TRACKING,
                     "three fresh samples below the current bias enter tracking")) return 1;
    if (!expect_true(fabsf(report.error_mm + 30.005f) < 0.02f,
                     "85 dx below the current bias maps to about -30mm")) return 1;
    if (!expect_true(report.delta_vx_rpm > 0.0f,
                     "negative error reports physical X+ correction")) return 1;

    VisionController_Reset(&controller);
    feed(&controller, 4U, 300U, 342, 1U, &report);
    feed(&controller, 5U, 400U, 342, 1U, &report);
    feed(&controller, 6U, 500U, 342, 1U, &report);
    if (!expect_true(fabsf(report.error_mm) < 0.001f &&
                     fabsf(report.delta_vx_rpm) < 0.001f &&
                     report.reason == VISION_CONTROLLER_REASON_CENTERED,
                     "bias dx maps to zero correction")) return 1;

    feed(&controller, 7U, 600U, 342, 0U, &report);
    if (!expect_true(report.control_valid == 0U &&
                     fabsf(report.delta_vx_rpm) < 0.001f &&
                     report.state == VISION_CONTROLLER_LOST,
                     "lost sample clears report delta")) return 1;

    (void)puts("PASS vision_controller_rough_mapping_contract");
    return 0;
}

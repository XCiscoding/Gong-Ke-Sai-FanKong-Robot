#include "vision_controller_observer.h"

#include <stdio.h>
#include "main.h"
#include "vision.h"

#define VISION_CONTROLLER_OBSERVER_LOG_INTERVAL_MS 250U

volatile VisionControllerObserverDiagnostics vision_controller_observer_diag;
static VisionController controller;
static uint32_t last_log_ms;

static void VisionControllerObserver_InitUart4(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gpio);
    UART4->CR1 = UART4->CR2 = UART4->CR3 = 0U;
    UART4->BRR = (HAL_RCC_GetPCLK1Freq() + 57600U) / 115200U;
    UART4->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void VisionControllerObserver_Write(const char *text)
{
    uint32_t started = HAL_GetTick();
    while (*text != '\0') {
        while ((UART4->SR & USART_SR_TXE) == 0U) {
            if ((uint32_t)(HAL_GetTick() - started) > 25U) return;
        }
        UART4->DR = (uint8_t)*text++;
    }
}

static void VisionControllerObserver_Log(void)
{
    char line[176];
    long error_x10 = (long)(vision_controller_observer_diag.report.error_mm * 10.0f);
    long delta_x100 = (long)(vision_controller_observer_diag.report.delta_vx_rpm * 100.0f);
    (void)snprintf(line, sizeof(line),
                   "VCR sid=%lu seq=%u dx=%d e10=%ld valid=%u age=%lu cv=%u st=%u rs=%u d100=%ld\r\n",
                   (unsigned long)vision_controller_observer_diag.sample_id,
                   (unsigned int)vision_controller_observer_diag.sequence,
                   (int)vision_controller_observer_diag.dx,
                   error_x10,
                   (unsigned int)vision_controller_observer_diag.valid,
                   (unsigned long)vision_controller_observer_diag.age_ms,
                   (unsigned int)vision_controller_observer_diag.report.control_valid,
                   (unsigned int)vision_controller_observer_diag.report.state,
                   (unsigned int)vision_controller_observer_diag.report.reason,
                   delta_x100);
    VisionControllerObserver_Write(line);
}

void VisionControllerObserver_Init(void)
{
    VisionControllerConfig rough_profile;
    const VisionControllerObserverDiagnostics initial = {0};
    vision_controller_observer_diag = initial;
    VisionController_GetDefaultConfig(&rough_profile);
    /* 当前居中实测dx=342；比例仍只是原有的粗方向候选。 */
    rough_profile.raw_dx_bias = 342;
    rough_profile.raw_dx_to_vehicle_mm = 0.353f;
    rough_profile.enter_deadband = 12U;
    rough_profile.exit_deadband = 18U;
    rough_profile.kp_rpm_per_mm = 0.40f;
    rough_profile.max_delta_vx_rpm = 8.0f;
    rough_profile.error_to_delta_vx_polarity = -1;
    (void)VisionController_Init(&controller, &rough_profile);
    VisionControllerObserver_InitUart4();
    last_log_ms = HAL_GetTick();
    VisionControllerObserver_Write("VCR READY; MOTOR OFF; ROUGH b342 k0.353 s+1; K230 USART6 PC6/PC7; LOG UART4 PC10 115200\r\n");
}

void VisionControllerObserver_Service(uint32_t now_ms)
{
    VisionObservation latest;
    VisionControllerObservation observation;
    VisionControllerReport report;
    uint8_t have_observation = Vision_GetLatest(&latest);
    if (have_observation != 0U) {
        observation.sample_id = latest.sample_id;
        observation.received_ms = latest.received_ms;
        observation.dx = latest.dx;
        observation.valid = latest.valid;
        (void)VisionController_Update(&controller, 1U, now_ms, &observation, &report);
        vision_controller_observer_diag.sample_id = latest.sample_id;
        vision_controller_observer_diag.sequence = latest.sequence;
        vision_controller_observer_diag.dx = latest.dx;
        vision_controller_observer_diag.valid = latest.valid;
        vision_controller_observer_diag.received_ms = latest.received_ms;
        vision_controller_observer_diag.age_ms = (uint32_t)(now_ms - latest.received_ms);
    } else {
        (void)VisionController_Update(&controller, 1U, now_ms, 0, &report);
        vision_controller_observer_diag.sample_id = 0U;
        vision_controller_observer_diag.sequence = 0U;
        vision_controller_observer_diag.dx = 0;
        vision_controller_observer_diag.valid = 0U;
        vision_controller_observer_diag.received_ms = 0U;
        vision_controller_observer_diag.age_ms = UINT32_MAX;
    }
    vision_controller_observer_diag.report = report;
    if ((uint32_t)(now_ms - last_log_ms) >= VISION_CONTROLLER_OBSERVER_LOG_INTERVAL_MS) {
        last_log_ms = now_ms;
        VisionControllerObserver_Log();
    }
}

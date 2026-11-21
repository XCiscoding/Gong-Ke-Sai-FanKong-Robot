#include "vision.h"
#include "main.h"

volatile VisionObservation vision_latest;
volatile VisionDiagnostics vision_diag;

static void InvalidateLatest(void)
{
    vision_latest.valid = 0U;
    vision_latest.quality = 0U;
    vision_latest.dx = 0;
    vision_latest.dy = 0;
    vision_latest.angle_tenths = 0;
}

void Vision_Init(void)
{
    const VisionDiagnostics initial = {0};
    const VisionObservation no_observation = {0};
    vision_diag = initial;
    vision_latest = no_observation;
    VisionLink_Init();
}

void Vision_Service(void)
{
    uint32_t now_ms = HAL_GetTick();
    VisionLinkMessage message;
    VisionLinkDiagnostics link_diagnostics;
    VisionUartDiagnostics uart_diagnostics;

    VisionLink_Service(now_ms);
    if (VisionLink_TakeMessage(&message)) {
        vision_latest.mode = VISION_MODE_TRACK;
        vision_latest.target = VISION_TARGET_NONE;
        vision_latest.sequence = message.sequence;
        vision_latest.received_ms = now_ms;
        vision_latest.dy = 0;
        vision_latest.angle_tenths = 0;
        vision_latest.color = 0U;
        vision_latest.task_code = 0U;
        if (message.type == VISION_LINK_TYPE_TRACK) {
            vision_latest.valid = 1U;
            vision_latest.quality = 100U;
            vision_latest.dx = message.data;
        } else {
            InvalidateLatest();
            vision_latest.mode = VISION_MODE_TRACK;
            vision_latest.sequence = message.sequence;
            vision_latest.received_ms = now_ms;
        }
    }
    if (!VisionLink_IsActive()) InvalidateLatest();
    VisionLink_GetDiagnostics(&link_diagnostics);
    VisionUart_GetDiagnostics(&uart_diagnostics);
    vision_diag.link = link_diagnostics;
    vision_diag.uart = uart_diagnostics;
}

uint8_t Vision_GetLatest(VisionObservation *observation)
{
    uint32_t primask;
    uint8_t available;
    if (observation == 0) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    *observation = vision_latest;
    available = vision_latest.received_ms != 0U;
    if (primask == 0U) __enable_irq();
    return available;
}

uint8_t Vision_IsFresh(uint32_t now_ms)
{
    return VisionLink_IsActive() && vision_latest.valid &&
           vision_latest.quality == 100U &&
           vision_latest.received_ms != 0U &&
           (uint32_t)(now_ms - vision_latest.received_ms) <= VISION_LINK_FRESH_MS;
}

void Vision_GetDiagnostics(VisionDiagnostics *diagnostics)
{
    VisionLinkDiagnostics link_diagnostics;
    VisionUartDiagnostics uart_diagnostics;
    if (diagnostics == 0) return;
    VisionLink_GetDiagnostics(&link_diagnostics);
    VisionUart_GetDiagnostics(&uart_diagnostics);
    vision_diag.link = link_diagnostics;
    vision_diag.uart = uart_diagnostics;
    *diagnostics = vision_diag;
}

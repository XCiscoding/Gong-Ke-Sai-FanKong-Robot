#include "vision.h"
#include "main.h"

volatile VisionObservation vision_latest;
volatile VisionDiagnostics vision_diag;
static uint8_t qr_ready_pending;
static uint16_t qr_ready_task_code;

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
    qr_ready_pending = 0U;
    qr_ready_task_code = 0U;
    VisionLink_Init();
}

void Vision_EnableReceiveOnly(void)
{
    VisionLink_EnableReceiveOnly();
}

void Vision_Service(void)
{
    uint32_t now_ms = HAL_GetTick();
    VisionLinkMessage message;
    VisionLinkDiagnostics link_diagnostics;
    VisionUartDiagnostics uart_diagnostics;

    VisionLink_Service(now_ms);
    if (VisionLink_TakeMessage(&message)) {
        if (message.type == VISION_LINK_TYPE_QR_READY) {
            if (message.data > 0 && message.data <= 999) {
                qr_ready_task_code = (uint16_t)message.data;
                qr_ready_pending = 1U;
            }
        } else {
        vision_latest.mode = VISION_MODE_TRACK;
        vision_latest.target = VISION_TARGET_NONE;
        vision_latest.sequence = message.sequence;
        ++vision_latest.sample_id;
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
    }
    if (!VisionLink_IsActive()) InvalidateLatest();
    VisionLink_GetDiagnostics(&link_diagnostics);
    VisionUart_GetDiagnostics(&uart_diagnostics);
    vision_diag.link = link_diagnostics;
    vision_diag.uart = uart_diagnostics;
}

uint8_t Vision_RequestQr(void)
{
    qr_ready_pending = 0U;
    return VisionLink_RequestMode(VISION_LINK_TYPE_MODE_QR);
}

uint8_t Vision_RequestTarget(void)
{
    return VisionLink_RequestMode(VISION_LINK_TYPE_MODE_TARGET);
}

uint8_t Vision_RequestTrack(void)
{
    return VisionLink_RequestMode(VISION_LINK_TYPE_MODE_TRACK);
}

uint8_t Vision_TakeQrReady(uint16_t *task_code)
{
    if (task_code == 0 || qr_ready_pending == 0U) return 0U;
    *task_code = qr_ready_task_code;
    qr_ready_pending = 0U;
    return 1U;
}

uint8_t Vision_IsLinkActive(void)
{
    return VisionLink_IsActive();
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

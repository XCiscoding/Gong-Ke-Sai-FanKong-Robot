#ifndef VISION_H
#define VISION_H

#include <stdint.h>
#include "vision_link.h"
#include "vision_uart.h"

/* This image is a passive UART/vision proof. Keep every motor path suppressed
 * until the binary protocol, sign, LOST and timeout behaviour pass on hardware. */
#ifndef VISION_LINK_TEST_ACTIVE
#define VISION_LINK_TEST_ACTIVE 1U
#endif
#if VISION_LINK_TEST_ACTIVE != 0U && VISION_LINK_TEST_ACTIVE != 1U
#error "Invalid VISION_LINK_TEST_ACTIVE"
#endif

typedef enum {
    VISION_MODE_IDLE = 0,
    VISION_MODE_TRACK,
    VISION_MODE_RECOG
} VisionMode;

#ifndef VISION_STARTUP_MODE
#define VISION_STARTUP_MODE VISION_MODE_TRACK
#endif

typedef enum {
    VISION_TARGET_NONE = 0,
    VISION_TARGET_QR,
    VISION_TARGET_BALL,
    VISION_TARGET_TARGET
} VisionTarget;

typedef struct {
    VisionMode mode;
    VisionTarget target;
    uint8_t valid;
    uint16_t sequence;
    int16_t dx;
    int16_t dy;
    int16_t angle_tenths;
    uint8_t quality;
    uint8_t color;
    uint16_t task_code;
    uint32_t received_ms;
} VisionObservation;

typedef struct {
    VisionLinkDiagnostics link;
    VisionUartDiagnostics uart;
} VisionDiagnostics;

extern volatile VisionObservation vision_latest;
extern volatile VisionDiagnostics vision_diag;

void Vision_Init(void);
void Vision_Service(void);
uint8_t Vision_GetLatest(VisionObservation *observation);
uint8_t Vision_IsFresh(uint32_t now_ms);
void Vision_GetDiagnostics(VisionDiagnostics *diagnostics);

#endif /* VISION_H */

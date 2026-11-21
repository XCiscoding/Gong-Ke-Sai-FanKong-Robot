#ifndef VISION_CONTROLLER_OBSERVER_H
#define VISION_CONTROLLER_OBSERVER_H

#include <stdint.h>
#include "vision_controller.h"

/* Passive evidence only. Nothing in this module may adopt its report as motion. */
typedef struct {
    uint32_t sample_id;
    uint16_t sequence;
    int16_t dx;
    uint8_t valid;
    uint32_t received_ms;
    uint32_t age_ms;
    VisionControllerReport report;
} VisionControllerObserverDiagnostics;

extern volatile VisionControllerObserverDiagnostics vision_controller_observer_diag;

void VisionControllerObserver_Init(void);
void VisionControllerObserver_Service(uint32_t now_ms);

#endif /* VISION_CONTROLLER_OBSERVER_H */

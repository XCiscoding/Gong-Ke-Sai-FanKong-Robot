#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdint.h>

#define VISION_LINK_FRAME_SIZE 9U
#define VISION_LINK_HEAD_0 0xA5U
#define VISION_LINK_HEAD_1 0x5AU
#define VISION_LINK_TAIL_0 0x0DU
#define VISION_LINK_TAIL_1 0x0AU
#define VISION_LINK_FRESH_MS 200U
#define VISION_LINK_RETRY_MS 500U

typedef enum {
    VISION_LINK_TYPE_MODE_TRACK = 0x01,
    VISION_LINK_TYPE_TRACK = 0x10,
    VISION_LINK_TYPE_LOST = 0x11,
    VISION_LINK_TYPE_ACK_TRACK = 0x81
} VisionLinkType;

typedef struct {
    uint8_t type;
    uint8_t sequence;
    int16_t data;
} VisionLinkMessage;

typedef struct {
    uint32_t complete_frames;
    uint32_t bad_checksum;
    uint32_t bad_tail;
    uint32_t unknown_type;
    uint32_t acknowledgements;
    uint32_t stale_sequences;
    uint32_t unexpected_frames;
    uint32_t session_resets;
    uint32_t link_timeouts;
    uint32_t track_frames;
    uint32_t lost_frames;
    uint32_t last_ack_ms;
    uint32_t last_progress_ms;
    uint8_t link_active;
    uint8_t last_sequence;
    uint8_t request_sequence;
} VisionLinkDiagnostics;

void VisionLink_Init(void);
void VisionLink_Service(uint32_t now_ms);
uint8_t VisionLink_TakeMessage(VisionLinkMessage *message);
uint8_t VisionLink_IsActive(void);
void VisionLink_GetDiagnostics(VisionLinkDiagnostics *diagnostics);
void VisionLink_EncodeFrame(uint8_t type, uint8_t sequence, int16_t data,
                            uint8_t frame[VISION_LINK_FRAME_SIZE]);

#endif /* VISION_LINK_H */

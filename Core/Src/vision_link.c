#include "vision_link.h"
#include "vision_uart.h"

static uint8_t rx_frame[VISION_LINK_FRAME_SIZE];
static uint8_t rx_length;
static uint8_t request_sent;
static uint8_t sequence_initialized;
static uint32_t last_request_ms;
static VisionLinkMessage pending_message;
static uint8_t message_pending;
static VisionLinkDiagnostics link_diag;

static uint8_t CheckByte(uint8_t type, uint8_t sequence,
                         uint8_t data_low, uint8_t data_high)
{
    return (uint8_t)(0U - (uint8_t)(type + sequence + data_low + data_high));
}

void VisionLink_EncodeFrame(uint8_t type, uint8_t sequence, int16_t data,
                            uint8_t frame[VISION_LINK_FRAME_SIZE])
{
    uint16_t raw = (uint16_t)data;
    frame[0] = VISION_LINK_HEAD_0;
    frame[1] = VISION_LINK_HEAD_1;
    frame[2] = type;
    frame[3] = sequence;
    frame[4] = (uint8_t)(raw & 0xFFU);
    frame[5] = (uint8_t)(raw >> 8);
    frame[6] = CheckByte(frame[2], frame[3], frame[4], frame[5]);
    frame[7] = VISION_LINK_TAIL_0;
    frame[8] = VISION_LINK_TAIL_1;
}

static void ParserResync(void)
{
    uint8_t start;
    for (start = 1U; start + 1U < rx_length; start++) {
        if (rx_frame[start] == VISION_LINK_HEAD_0 &&
            rx_frame[start + 1U] == VISION_LINK_HEAD_1) {
            uint8_t remaining = (uint8_t)(rx_length - start);
            uint8_t index;
            for (index = 0U; index < remaining; index++)
                rx_frame[index] = rx_frame[start + index];
            rx_length = remaining;
            return;
        }
    }
    if (rx_length != 0U && rx_frame[rx_length - 1U] == VISION_LINK_HEAD_0) {
        rx_frame[0] = VISION_LINK_HEAD_0;
        rx_length = 1U;
    } else {
        rx_length = 0U;
    }
}

static uint8_t ParseByte(uint8_t byte, VisionLinkMessage *message)
{
    if (rx_length == 0U) {
        if (byte == VISION_LINK_HEAD_0) rx_frame[rx_length++] = byte;
        return 0U;
    }
    if (rx_length == 1U) {
        if (byte == VISION_LINK_HEAD_1) rx_frame[rx_length++] = byte;
        else if (byte != VISION_LINK_HEAD_0) rx_length = 0U;
        return 0U;
    }

    rx_frame[rx_length++] = byte;
    if (rx_length < VISION_LINK_FRAME_SIZE) return 0U;

    if (rx_frame[7] != VISION_LINK_TAIL_0 ||
        rx_frame[8] != VISION_LINK_TAIL_1) {
        link_diag.bad_tail++;
        ParserResync();
        return 0U;
    }
    if (rx_frame[6] != CheckByte(rx_frame[2], rx_frame[3],
                                 rx_frame[4], rx_frame[5])) {
        link_diag.bad_checksum++;
        ParserResync();
        return 0U;
    }

    message->type = rx_frame[2];
    message->sequence = rx_frame[3];
    message->data = (int16_t)((uint16_t)rx_frame[4] |
                              ((uint16_t)rx_frame[5] << 8));
    link_diag.complete_frames++;
    rx_length = 0U;
    return 1U;
}

static void EndSession(void)
{
    link_diag.link_active = 0U;
    sequence_initialized = 0U;
    request_sent = 0U;
    message_pending = 0U;
}

static void AcceptData(const VisionLinkMessage *message, uint32_t now_ms)
{
    uint8_t delta;
    if (!link_diag.link_active) {
        link_diag.unexpected_frames++;
        return;
    }
    if (sequence_initialized) {
        delta = (uint8_t)(message->sequence - link_diag.last_sequence);
        if (delta == 0U || delta > 0x7FU) {
            link_diag.stale_sequences++;
            return;
        }
    }
    sequence_initialized = 1U;
    link_diag.last_sequence = message->sequence;
    link_diag.last_progress_ms = now_ms;
    pending_message = *message;
    message_pending = 1U;
    if (message->type == VISION_LINK_TYPE_TRACK) link_diag.track_frames++;
    else link_diag.lost_frames++;
}

static void Dispatch(const VisionLinkMessage *message, uint32_t now_ms)
{
    if (message->type == VISION_LINK_TYPE_ACK_TRACK) {
        if (message->data != 0 || !request_sent ||
            message->sequence != link_diag.request_sequence) {
            link_diag.unexpected_frames++;
            return;
        }
        link_diag.acknowledgements++;
        link_diag.session_resets++;
        link_diag.last_ack_ms = now_ms;
        link_diag.last_progress_ms = now_ms;
        link_diag.link_active = 1U;
        sequence_initialized = 0U;
        message_pending = 0U;
        return;
    }
    if (message->type == VISION_LINK_TYPE_TRACK) {
        AcceptData(message, now_ms);
        return;
    }
    if (message->type == VISION_LINK_TYPE_LOST) {
        if (message->data != 0) {
            link_diag.unexpected_frames++;
            return;
        }
        AcceptData(message, now_ms);
        return;
    }
    if (message->type == VISION_LINK_TYPE_MODE_TRACK)
        link_diag.unexpected_frames++;
    else
        link_diag.unknown_type++;
}

static void RequestTrack(uint32_t now_ms)
{
    uint8_t frame[VISION_LINK_FRAME_SIZE];
    uint8_t next_sequence = (uint8_t)(link_diag.request_sequence + 1U);
    VisionLink_EncodeFrame(VISION_LINK_TYPE_MODE_TRACK, next_sequence, 0, frame);
    if (VisionUart_Send(frame, VISION_LINK_FRAME_SIZE)) {
        link_diag.request_sequence = next_sequence;
        last_request_ms = now_ms;
        request_sent = 1U;
    }
}

void VisionLink_Init(void)
{
    const VisionLinkDiagnostics empty = {0};
    link_diag = empty;
    rx_length = request_sent = sequence_initialized = message_pending = 0U;
    last_request_ms = 0U;
    VisionUart_Init();
}

void VisionLink_Service(uint32_t now_ms)
{
    uint8_t byte;
    VisionLinkMessage message;

    VisionUart_Service();
    while (VisionUart_ReadByte(&byte)) {
        if (ParseByte(byte, &message)) Dispatch(&message, now_ms);
    }

    if (link_diag.link_active &&
        (uint32_t)(now_ms - link_diag.last_progress_ms) > VISION_LINK_FRESH_MS) {
        link_diag.link_timeouts++;
        EndSession();
    }
    if (!link_diag.link_active &&
        (!request_sent ||
         (uint32_t)(now_ms - last_request_ms) >= VISION_LINK_RETRY_MS)) {
        RequestTrack(now_ms);
    }
}

uint8_t VisionLink_TakeMessage(VisionLinkMessage *message)
{
    if (message == 0 || !message_pending) return 0U;
    *message = pending_message;
    message_pending = 0U;
    return 1U;
}

uint8_t VisionLink_IsActive(void)
{
    return link_diag.link_active;
}

void VisionLink_GetDiagnostics(VisionLinkDiagnostics *diagnostics)
{
    if (diagnostics != 0) *diagnostics = link_diag;
}

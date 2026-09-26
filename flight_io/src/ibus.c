#include "flight_io/ibus.h"
#include <string.h>

bool fio_ibus_decode(const uint8_t *bytes, size_t length, uint32_t received_at_ms,
                     fio_ibus_frame_t *output)
{
    if (bytes == NULL || output == NULL || length != FIO_IBUS_FRAME_SIZE ||
        bytes[0] != 0x20 || bytes[1] != 0x40) return false;
    uint16_t expected = 0xFFFFu;
    for (size_t i = 0; i < FIO_IBUS_FRAME_SIZE - 2; ++i)
        expected = (uint16_t)(expected - bytes[i]);
    const uint16_t received = (uint16_t)((uint16_t)bytes[30] | ((uint16_t)bytes[31] << 8));
    if (received != expected) return false;
    fio_ibus_frame_t result = {0};
    for (unsigned i = 0; i < FIO_IBUS_CHANNELS; ++i)
        result.channels[i] = (uint16_t)((uint16_t)bytes[2 + 2 * i] |
                                       ((uint16_t)(bytes[3 + 2 * i] & 0x0Fu) << 8));
    /* Recognize the channel-1/channel-4 high-nibble loss indications used by
     * existing iBUS decoders. Not all receiver firmware emits these flags. */
    result.failsafe = (bytes[3] & 0xF0u) != 0 || (bytes[9] & 0xF0u) != 0;
    result.received_at_ms = received_at_ms;
    *output = result;
    return true;
}

static void discard_first(fio_ibus_parser_t *parser)
{
    --parser->used;
    memmove(parser->bytes, parser->bytes + 1, parser->used);
}

bool fio_ibus_feed(fio_ibus_parser_t *parser, uint8_t byte, uint32_t now_ms,
                   fio_ibus_frame_t *output)
{
    if (parser == NULL || output == NULL) return false;
    if (parser->used >= FIO_IBUS_FRAME_SIZE ||
        (uint32_t)(now_ms - parser->last_byte_ms) > 5u) parser->used = 0;
    parser->last_byte_ms = now_ms;
    parser->bytes[parser->used++] = byte;
    while (parser->used > 0 && (parser->bytes[0] != 0x20 ||
           (parser->used > 1 && parser->bytes[1] != 0x40))) discard_first(parser);
    if (parser->used < FIO_IBUS_FRAME_SIZE) return false;
    if (fio_ibus_decode(parser->bytes, parser->used, now_ms, output)) {
        parser->used = 0;
        return true;
    }
    discard_first(parser);
    return false;
}

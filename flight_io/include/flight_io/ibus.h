#ifndef FLIGHT_IO_IBUS_H
#define FLIGHT_IO_IBUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FIO_IBUS_CHANNELS 14u
#define FIO_IBUS_FRAME_SIZE 32u

typedef struct {
    uint16_t channels[FIO_IBUS_CHANNELS];
    uint32_t received_at_ms;
    bool failsafe; /* Recognized channel flag bits; absence is not proof of RF health. */
} fio_ibus_frame_t;

/* Initialize to {0}. Firmware supplies UART bytes (115200 8N1) in order. */
typedef struct {
    uint8_t bytes[FIO_IBUS_FRAME_SIZE];
    size_t used;
    uint32_t last_byte_ms;
} fio_ibus_parser_t;

/* Check one exact channel frame, 0x20/0x40 header and subtractive checksum.
 * Returns false without changing output on malformed input. No RF assumption. */
bool fio_ibus_decode(const uint8_t *bytes, size_t length, uint32_t received_at_ms,
                     fio_ibus_frame_t *output);

/* Returns true only when a complete valid frame is produced. On false, output
 * is unchanged but parser state advances. Resynchronizes after noise/dropped
 * bytes; discards partial data after a gap >5 ms. Call with reception times. */
bool fio_ibus_feed(fio_ibus_parser_t *parser, uint8_t byte, uint32_t now_ms,
                   fio_ibus_frame_t *output);

#ifdef __cplusplus
}
#endif
#endif

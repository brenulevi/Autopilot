#ifndef FLIGHT_COMMON_CRC32_H
#define FLIGHT_COMMON_CRC32_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRC-32/ISO-HDLC: reflected polynomial 0xEDB88320, initial/final XOR
 * 0xFFFFFFFF. Compatible with zlib CRC32. data must contain length bytes;
 * NULL is permitted only for length == 0, whose checksum is zero. */
uint32_t flight_crc32(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif
#endif

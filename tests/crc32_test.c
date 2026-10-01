#include "flight_common/crc32.h"
#include <stdio.h>

int main(void)
{
    const uint8_t check[] = "123456789";
    const uint8_t zero[] = {0};
    if (flight_crc32(NULL, 0) != 0 ||
        flight_crc32(check, 9) != UINT32_C(0xCBF43926) ||
        flight_crc32(zero, 1) != UINT32_C(0xD202EF8D)) {
        fprintf(stderr, "CRC32 known-vector mismatch\n");
        return 1;
    }
    return 0;
}

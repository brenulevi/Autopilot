#include "flight_io/ibus.h"
#include <stdio.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); return 1; } } while (0)

static const uint8_t packet[32] = {
    0x20,0x40,0xE8,0x03,0xDC,0x05,0xD0,0x07,0xE2,0x04,0xD6,0x06,0xE8,0x03,0xDC,0x05,
    0xDC,0x05,0xDC,0x05,0xDC,0x05,0xDC,0x05,0xDC,0x05,0xDC,0x05,0xDC,0x05,0x47,0xF3
};

static void checksum(uint8_t bytes[32])
{
    unsigned sum = 0;
    for (unsigned i = 0; i < 30; ++i) sum += bytes[i];
    unsigned value = 65535u - sum;
    bytes[30] = (uint8_t)value;
    bytes[31] = (uint8_t)(value >> 8);
}

int main(void)
{
    fio_ibus_frame_t frame = {0};
    CHECK(fio_ibus_decode(packet, 32, 42, &frame));
    CHECK(frame.channels[0] == 1000 && frame.channels[1] == 1500 && frame.channels[2] == 2000);
    CHECK(frame.channels[3] == 1250 && frame.channels[4] == 1750 && frame.channels[13] == 1500);
    CHECK(frame.received_at_ms == 42 && !frame.failsafe);
    unsigned char snapshot[sizeof(frame)];
    memcpy(snapshot, &frame, sizeof(frame));
    for (unsigned length = 0; length < 32; ++length) CHECK(!fio_ibus_decode(packet, length, 0, &frame));
    CHECK(!fio_ibus_decode(packet, 33, 0, &frame));
    CHECK(!fio_ibus_decode(NULL, 32, 0, &frame));
    CHECK(!fio_ibus_decode(packet, 32, 0, NULL));
    for (unsigned i = 0; i < 32; ++i) {
        uint8_t bad[32]; memcpy(bad, packet, 32); bad[i] ^= 1;
        CHECK(!fio_ibus_decode(bad, 32, 0, &frame));
        CHECK(memcmp(snapshot, &frame, sizeof(frame)) == 0);
    }
    uint8_t flagged[32]; memcpy(flagged, packet, 32);
    flagged[3] |= 0x10; checksum(flagged);
    CHECK(fio_ibus_decode(flagged, 32, 1, &frame) && frame.failsafe && frame.channels[0] == 1000);
    memcpy(flagged, packet, 32); flagged[9] |= 0x80; checksum(flagged);
    CHECK(fio_ibus_decode(flagged, 32, 1, &frame) && frame.failsafe);

    fio_ibus_parser_t parser = {0};
    const uint8_t noise[] = {0x12, 0x20, 0x20, 0x40, 0x01, 0xFE};
    for (unsigned i = 0; i < sizeof(noise); ++i) CHECK(!fio_ibus_feed(&parser, noise[i], 100, &frame));
    unsigned produced = 0;
    for (unsigned repeat = 0; repeat < 2; ++repeat)
        for (unsigned i = 0; i < 32; ++i)
            produced += fio_ibus_feed(&parser, packet[i], 100 + repeat, &frame) ? 1u : 0u;
    CHECK(produced == 2 && frame.received_at_ms == 101);

    /* Recover after each possible dropped byte, with no timing gap to help. */
    for (unsigned dropped = 0; dropped < 32; ++dropped) {
        memset(&parser, 0, sizeof(parser)); produced = 0;
        for (unsigned i = 0; i < 32; ++i)
            if (i != dropped) CHECK(!fio_ibus_feed(&parser, packet[i], 0, &frame));
        for (unsigned i = 0; i < 32; ++i)
            produced += fio_ibus_feed(&parser, packet[i], 1, &frame) ? 1u : 0u;
        CHECK(produced == 1);
    }
    memset(&parser, 0, sizeof(parser));
    CHECK(!fio_ibus_feed(&parser, 0x20, 10, &frame));
    CHECK(!fio_ibus_feed(&parser, 0x40, 16, &frame)); /* Partial-frame timeout. */
    CHECK(parser.used == 0);
    CHECK(!fio_ibus_feed(&parser, 0x20, UINT32_MAX, &frame));
    CHECK(!fio_ibus_feed(&parser, 0x40, 0, &frame));
    CHECK(parser.used == 2); /* Millisecond wrap must not discard an in-flight frame. */
    CHECK(!fio_ibus_feed(NULL, 0, 0, &frame));
    return 0;
}

#include <stdint.h>
#include "bl_internal.h"

/* Reflected CRC-32/ISO-HDLC, compatible with Python zlib.crc32. Keep this
 * tableless and out of LTO cloning: protocol v3 needs the recovered 64 bytes. */
__attribute__((noinline, noipa))
uint32_t bl_crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;

    while (length-- != 0u) {
        uint8_t bit;

        crc ^= *bytes++;
        for (bit = 0u; bit < 8u; bit++)
            crc = (crc >> 1) ^
                  (0xEDB88320u & (0u - (crc & 1u)));
    }

    return crc ^ 0xFFFFFFFFu;
}

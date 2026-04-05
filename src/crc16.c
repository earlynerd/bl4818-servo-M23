#include "crc16.h"

uint16_t crc16_update(uint16_t crc, uint8_t data)
{
    uint8_t i;

    crc ^= (uint16_t)data << 8;
    for (i = 0; i < 8; i++) {
        if (crc & 0x8000u)
            crc = (crc << 1) ^ 0x1021u;
        else
            crc <<= 1;
    }
    return crc;
}

uint16_t crc16_ccitt(const uint8_t *data, uint8_t len)
{
    uint16_t crc = CRC16_INIT;
    uint8_t i;

    for (i = 0; i < len; i++)
        crc = crc16_update(crc, data[i]);
    return crc;
}

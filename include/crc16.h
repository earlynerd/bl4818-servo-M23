#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

#define CRC16_INIT 0xFFFFu

uint16_t crc16_update(uint16_t crc, uint8_t data);
uint16_t crc16_ccitt(const uint8_t *data, uint8_t len);

#endif /* CRC16_H */

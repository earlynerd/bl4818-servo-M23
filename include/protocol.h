/*
 * Binary Ring Protocol
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

void protocol_init(void);
void protocol_poll(void);
void protocol_tick_1khz(void);
uint8_t protocol_is_enumerated(void);

#endif /* PROTOCOL_H */

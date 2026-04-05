#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

void protocol_init(void);
void protocol_poll(void);
void protocol_tick(void);
uint8_t protocol_is_enumerated(void);
uint8_t protocol_get_address(void);

#endif /* PROTOCOL_H */

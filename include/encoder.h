#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);
void encoder_poll(void);
uint32_t encoder_get_raw(void);
uint16_t encoder_get_angle(void);

#endif /* ENCODER_H */
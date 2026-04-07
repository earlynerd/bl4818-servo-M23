#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);
void encoder_poll(void);
uint32_t encoder_get_raw(void);
uint16_t encoder_get_angle(void);    /* 0-16383, single-turn */
int32_t  encoder_get_position(void); /* continuous multi-turn (encoder counts) */
void     encoder_set_zero_reference(uint16_t angle);
uint8_t  encoder_has_zero_reference(void);
uint16_t encoder_get_zero_reference(void);
void     encoder_reset_position(void);

#endif /* ENCODER_H */

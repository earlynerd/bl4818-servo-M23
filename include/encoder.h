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

/* SSI CSn polarity: PB1 level that asserts CSn at the encoder.
 * 0 matches the FET-inverting PCB variant; 1 matches the modchip
 * variant (no FET).  Persisted in flash; auto-detected at boot via
 * MT6701 frame CRC if no record is stored. */
void    encoder_set_csn_polarity(uint8_t assert_level);
uint8_t encoder_get_csn_polarity(void);
uint8_t encoder_has_csn_polarity(void);
uint8_t encoder_autodetect_csn_polarity(void);  /* 1=success, 0=encoder didn't respond */

#endif /* ENCODER_H */

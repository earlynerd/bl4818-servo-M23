/*
 * Six-Step Commutation — Low-Side-Only PWM Chopping
 */
#ifndef COMMUTATION_H
#define COMMUTATION_H

#include <stdint.h>

void commutation_get_masks(uint8_t hall_state, int8_t direction,
                           uint8_t *pmen, uint8_t *pmd);
void commutation_update(int8_t direction);
void commutation_coast(void);

#endif /* COMMUTATION_H */

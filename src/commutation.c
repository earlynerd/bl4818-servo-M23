/*
 * Six-Step Commutation — Low-Side-Only PWM Chopping
 */

#include "m2003_config.h"
#include "M2003.h"
#include "commutation.h"
#include "hall.h"
#include "app_pwm.h"

/* Forward commutation tables */
static const uint8_t fwd_pmen[8] = {
    0x3F, 0x37, 0x2F, 0x2F, 0x3E, 0x37, 0x3E, 0x3F
};

static const uint8_t fwd_pmd[8] = {
    0x00, 0x02, 0x04, 0x02, 0x20, 0x20, 0x04, 0x00
};

/* Reverse commutation tables */
static const uint8_t rev_pmen[8] = {
    0x3F, 0x3E, 0x37, 0x3E, 0x2F, 0x2F, 0x37, 0x3F
};

static const uint8_t rev_pmd[8] = {
    0x00, 0x04, 0x20, 0x20, 0x02, 0x04, 0x02, 0x00
};

static uint8_t rotate_hall_state(uint8_t hall_state)
{
    static const uint8_t seq[6] = {1, 3, 2, 6, 4, 5};
    uint8_t i;

    if (COMMUTATION_OFFSET == 0 || hall_state == 0 || hall_state == 7)
        return hall_state;

    for (i = 0; i < 6; i++) {
        if (seq[i] == hall_state)
            return seq[(i + COMMUTATION_OFFSET) % 6];
    }

    return hall_state;
}

void commutation_get_masks(uint8_t hall_state, int8_t direction,
                           uint8_t *pmen, uint8_t *pmd)
{
    hall_state = rotate_hall_state(hall_state);

    if (hall_state == 0 || hall_state == 7) {
        *pmen = 0x3F;
        *pmd  = 0x00;
        return;
    }

    if (direction >= 0) {
        *pmen = fwd_pmen[hall_state];
        *pmd  = fwd_pmd[hall_state];
    } else {
        *pmen = rev_pmen[hall_state];
        *pmd  = rev_pmd[hall_state];
    }
}

void commutation_update(int8_t direction)
{
    uint8_t hall = hall_read();
    uint8_t pmen, pmd;

    commutation_get_masks(hall, direction, &pmen, &pmd);
    pwm_set_commutation(pmen, pmd);
}

void commutation_coast(void)
{
    pwm_set_commutation(0x3F, 0x00);
}

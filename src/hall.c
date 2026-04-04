/*
 * Hall Sensor Interface for Nuvoton M2003
 */

#include "m2003_config.h"
#include "M2003.h"
#include "hall.h"

static const uint8_t hall_decode[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
static const uint8_t hall_forward_seq[8] = { 0xFF, 3, 6, 2, 5, 1, 4, 0xFF };

static uint8_t prev_hall;
static int8_t  detected_direction;
static int32_t hall_position;

void hall_init(void)
{
    /* Configure Hall pins as Input (PC.14, PB.5, PB.4) */
    GPIO_SetMode(PC, BIT14, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT5 | BIT4, GPIO_MODE_INPUT);

    prev_hall = hall_read();
    detected_direction = 0;
    hall_position = 0;
}

uint8_t hall_read_raw(void)
{
    uint8_t state = 0;
    if (PC14) state |= 0x01; // Hall 1
    if (PB5)  state |= 0x02; // Hall 2
    if (PB4)  state |= 0x04; // Hall 3
    return state;
}

uint8_t hall_decode_state(uint8_t raw_state)
{
    return hall_decode[raw_state & 0x07];
}

uint8_t hall_read(void)
{
    return hall_decode_state(hall_read_raw());
}

uint8_t hall_poll(void)
{
    uint8_t current = hall_read();

    if (current == prev_hall)
        return HALL_POLL_NO_CHANGE;

    if (current == 0 || current == 7)
        return HALL_POLL_INVALID;

    if (hall_forward_seq[prev_hall] == current) {
        detected_direction = 1;
        hall_position++;
    } else if (hall_forward_seq[current] == prev_hall) {
        detected_direction = -1;
        hall_position--;
    }

    prev_hall = current;
    return HALL_POLL_TRANSITION;
}

int8_t hall_direction(void) { return detected_direction; }
int32_t hall_count(void)     { return hall_position; }
void hall_count_reset(void) { hall_position = 0; }
uint16_t hall_period(void)  { return 0; } // TODO: Implement timer period
uint8_t hall_sector(void)   { return 0; } // TODO: Implement sector map

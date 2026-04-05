/*
 * Motor Control for Nuvoton M2003
 *
 * Control loop runs from the main loop 2kHz tick, NOT from ISR.
 * ADC ISR only captures raw samples.
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "motor.h"
#include "app_pwm.h"
#include "app_adc.h"
#include "commutation.h"
#include "hall.h"

static motor_state_t state;
static fault_code_t  fault;
static int16_t       target_duty;
static uint16_t      torque_limit_ma;
static uint16_t      current_ma;

#define DEFAULT_TORQUE_LIMIT_MA 800
#define CURRENT_LIMIT_MA        5000

void motor_init(void)
{
    state = MOTOR_IDLE;
    fault = FAULT_NONE;
    target_duty = 0;
    torque_limit_ma = DEFAULT_TORQUE_LIMIT_MA;
    current_ma = 0;
}

void motor_set_duty(int16_t duty)
{
    target_duty = duty;
}

void motor_set_torque_limit(uint16_t ma)
{
    if (ma > CURRENT_LIMIT_MA)
        ma = CURRENT_LIMIT_MA;
    torque_limit_ma = ma;
}

void motor_start(void)
{
    if (state == MOTOR_FAULT)
        return;
    if (target_duty == 0)
        return;

    int8_t dir = (target_duty >= 0) ? 1 : -1;
    fault = FAULT_NONE;
    state = MOTOR_RUN;

    commutation_update(dir);
    pwm_enable();
}

void motor_stop(void)
{
    target_duty = 0;
    pwm_set_duty(0);
    pwm_disable();
    state = MOTOR_IDLE;
}

void motor_clear_fault(void)
{
    target_duty = 0;
    fault = FAULT_NONE;
    pwm_disable();
    state = MOTOR_IDLE;
}

motor_state_t motor_get_state(void) { return state; }
fault_code_t  motor_get_fault(void) { return fault; }
uint16_t motor_get_current(void)    { return current_ma; }

void motor_poll_fast(void)
{
    uint8_t result = hall_poll();

    if (result == HALL_POLL_INVALID) {
        pwm_fault_brake();
        state = MOTOR_FAULT;
        fault = FAULT_HALL_INVALID;
        return;
    }

    if (result != HALL_POLL_TRANSITION)
        return;

    if (state == MOTOR_RUN) {
        int8_t dir = (target_duty >= 0) ? 1 : -1;
        commutation_update(dir);
    }
}

void motor_tick_2khz(void)
{
    int16_t duty;
    int8_t dir;
    uint16_t abs_duty;

    /* Grab latest ADC sample (captured at 20kHz by ISR) */
    current_ma = adc_read_current_ma();

    /* Hard overcurrent fault */
    if (current_ma > CURRENT_LIMIT_MA) {
        pwm_fault_brake();
        state = MOTOR_FAULT;
        fault = FAULT_OVERCURRENT;
        return;
    }

    if (state != MOTOR_RUN)
        return;

    if (target_duty == 0) {
        motor_stop();
        return;
    }

    /* Torque limiting */
    duty = target_duty;
    if (current_ma > torque_limit_ma) {
        duty = (int16_t)(((int32_t)(duty) * (int32_t)(torque_limit_ma)) / (int32_t)(current_ma));
    }

    if (duty >= 0) {
        dir = 1;
        abs_duty = (uint16_t)duty;
    } else {
        dir = -1;
        abs_duty = (uint16_t)(-duty);
    }

    pwm_set_duty(abs_duty);
}

/*
 * PWM Driver — Hardware Complementary PWM
 */
#ifndef PWM_H
#define PWM_H

#include <stdint.h>

void pwm_init(void);
void pwm_set_duty(uint16_t duty);
uint16_t pwm_get_duty(void);
void pwm_set_commutation(uint8_t pmen, uint8_t pmd);
void pwm_enable(void);
void pwm_disable(void);
void pwm_fault_brake(void);

#endif /* PWM_H */

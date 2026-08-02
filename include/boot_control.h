#ifndef BOOT_CONTROL_H
#define BOOT_CONTROL_H

#include <stdint.h>

void boot_control_arm_loader(void);
void boot_control_request(uint8_t address);
uint8_t boot_control_pending(void);
void boot_control_enter(void) __attribute__((noreturn));

#endif /* BOOT_CONTROL_H */

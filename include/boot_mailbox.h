#ifndef BOOT_MAILBOX_H
#define BOOT_MAILBOX_H

#include <stdint.h>
#include "firmware_image.h"

#define BOOT_MAILBOX_MAGIC  0x424C3233UL /* "32LB" in memory */

typedef struct
{
    uint32_t magic;
    uint32_t magic_inverse;
    uint8_t address;
    uint8_t address_inverse;
    uint16_t reserved;
    uint32_t reserved2;
} boot_mailbox_t;

typedef char boot_mailbox_size_must_be_16[
    (sizeof(boot_mailbox_t) == FIRMWARE_BOOT_MAILBOX_SIZE) ? 1 : -1];

#endif /* BOOT_MAILBOX_H */

#ifndef FIRMWARE_IMAGE_H
#define FIRMWARE_IMAGE_H

#include <stdint.h>

/* M2003FC1AE APROM layout. Keep this synchronized with both linker scripts. */
#define FIRMWARE_APROM_BASE          0x00000000UL
#define FIRMWARE_APP_LIMIT           0x00007A00UL
#define FIRMWARE_MANIFEST_BASE       0x00007A00UL
#define FIRMWARE_MANIFEST_PAGE_SIZE  0x00000200UL
#define FIRMWARE_PERSIST_BASE        0x00007C00UL
#define FIRMWARE_APROM_LIMIT         0x00008000UL

#define FIRMWARE_LDROM_BASE          0x00100000UL
#define FIRMWARE_LDROM_SIZE          0x00001000UL
#define FIRMWARE_FLASH_PAGE_SIZE     0x00000200UL

#define FIRMWARE_SRAM_BASE           0x20000000UL
#define FIRMWARE_SRAM_LIMIT          0x20001000UL
#define FIRMWARE_BOOT_MAILBOX_BASE   0x20000FF0UL
#define FIRMWARE_BOOT_MAILBOX_SIZE   0x00000010UL

#define FIRMWARE_MANIFEST_MAGIC      0x4633324DUL /* "M23F" in memory */
#define FIRMWARE_MANIFEST_VERSION    1u
#define FIRMWARE_MANIFEST_SIZE       32u

typedef struct
{
    uint32_t magic;
    uint16_t format_version;
    uint16_t header_size;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t image_version;
    uint32_t flags;
    uint32_t reserved;
    uint32_t manifest_crc32;
} firmware_manifest_t;

typedef char firmware_manifest_size_must_be_32[
    (sizeof(firmware_manifest_t) == FIRMWARE_MANIFEST_SIZE) ? 1 : -1];

#endif /* FIRMWARE_IMAGE_H */

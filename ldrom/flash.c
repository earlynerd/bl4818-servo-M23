#include "M2003.h"
#include "bl_internal.h"
#include "firmware_image.h"

#define BL_FMC_TIMEOUT 1000000UL

static int flash_command(uint32_t command, uint32_t address, uint32_t data)
{
    uint32_t timeout = BL_FMC_TIMEOUT;

    FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
    FMC->ISPCMD = command;
    FMC->ISPADDR = address;
    FMC->ISPDAT = data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();
    while ((FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) != 0u) {
        if (--timeout == 0u)
            return -1;
    }

    return ((FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk) == 0u) ? 0 : -1;
}

static int flash_read_command(uint32_t command, uint32_t address,
                              uint32_t *value)
{
    if (flash_command(command, address, 0u) != 0)
        return -1;
    *value = FMC->ISPDAT;
    return 0;
}

void bl_flash_init(void)
{
    uint32_t control = FMC->ISPCTL;

    control |= FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk;
    control &= ~(FMC_ISPCTL_LDUEN_Msk | FMC_ISPCTL_CFGUEN_Msk |
                 FMC_ISPCTL_SPUEN_Msk);
    FMC->ISPCTL = control;
    FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
}

int bl_flash_map_vectors(uint32_t address)
{
    uint32_t vector_address = address & FMC_PAGE_ADDR_MASK;

    if (flash_command(FMC_ISPCMD_VECMAP, vector_address, 0u) != 0)
        return -1;
    return ((FMC->ISPSTS & FMC_ISPSTS_VECMAP_Msk) ==
            (vector_address & FMC_ISPSTS_VECMAP_Msk)) ? 0 : -1;
}

int bl_flash_erase_app_page(uint8_t page_index)
{
    uint32_t address = (uint32_t)page_index * FMC_FLASH_PAGE_SIZE;

    if (address >= FIRMWARE_APP_LIMIT)
        return -1;
    return flash_command(FMC_ISPCMD_PAGE_ERASE, address, 0u);
}

int bl_flash_erase_manifest(void)
{
    uint32_t first_word;

    if (flash_command(FMC_ISPCMD_PAGE_ERASE,
                      FIRMWARE_MANIFEST_BASE, 0u) != 0)
        return -1;
    if (bl_flash_read_word(FIRMWARE_MANIFEST_BASE, &first_word) != 0)
        return -1;
    return (first_word == 0xFFFFFFFFu) ? 0 : -1;
}

int bl_flash_read_word(uint32_t address, uint32_t *value)
{
    if ((address & 3u) != 0u || value == 0)
        return -1;
    return flash_read_command(FMC_ISPCMD_READ, address, value);
}

int bl_flash_program_word(uint32_t address, uint32_t value)
{
    uint32_t current;
    uint32_t verify;

    if ((address & 3u) != 0u || address >= FIRMWARE_APP_LIMIT)
        return -1;
    if (bl_flash_read_word(address, &current) != 0)
        return -1;
    if (current == value)
        return 0;
    if (current != 0xFFFFFFFFu)
        return -1;
    if (flash_command(FMC_ISPCMD_PROGRAM, address, value) != 0)
        return -1;
    if (bl_flash_read_word(address, &verify) != 0)
        return -1;
    return (verify == value) ? 0 : -1;
}

uint32_t bl_flash_read_uid(uint8_t index)
{
    uint32_t value = 0xFFFFFFFFu;

    if (index < 3u)
        (void)flash_read_command(FMC_ISPCMD_READ_UID,
                                 (uint32_t)index << 2, &value);
    return value;
}

int bl_flash_crc32_app(uint32_t length, uint32_t *crc)
{
    uint32_t primask;
    int result = -1;

    if (crc == 0 || length < 8u || length > FIRMWARE_APP_LIMIT)
        return -1;

    primask = __get_PRIMASK();
    __disable_irq();
    if (bl_flash_map_vectors(FIRMWARE_APROM_BASE) == 0) {
        *crc = bl_crc32((const void *)FIRMWARE_APROM_BASE, length);
        result = 0;
    }
    if (bl_flash_map_vectors(FIRMWARE_LDROM_BASE) == 0) {
        if (primask == 0u)
            __enable_irq();
    } else {
        result = -1;
    }
    return result;
}

int bl_flash_commit_manifest(const firmware_manifest_t *manifest)
{
    const uint32_t *words = (const uint32_t *)manifest;
    uint32_t current;
    uint32_t verify;
    uint8_t index;

    if (manifest == 0)
        return -1;

    /* Program magic last. Any interruption before the final word remains
     * unambiguously uncommitted even if the other fields are valid. */
    for (index = 1u; index < FIRMWARE_MANIFEST_SIZE / 4u; index++) {
        uint32_t address = FIRMWARE_MANIFEST_BASE + 4u * index;
        if (bl_flash_read_word(address, &current) != 0 ||
            current != 0xFFFFFFFFu ||
            flash_command(FMC_ISPCMD_PROGRAM, address, words[index]) != 0 ||
            bl_flash_read_word(address, &verify) != 0 ||
            verify != words[index])
            return -1;
    }

    if (bl_flash_read_word(FIRMWARE_MANIFEST_BASE, &current) != 0 ||
        current != 0xFFFFFFFFu ||
        flash_command(FMC_ISPCMD_PROGRAM, FIRMWARE_MANIFEST_BASE,
                      words[0]) != 0 ||
        bl_flash_read_word(FIRMWARE_MANIFEST_BASE, &verify) != 0 ||
        verify != words[0])
        return -1;
    return 0;
}

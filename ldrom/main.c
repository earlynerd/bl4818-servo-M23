#include <stdint.h>
#include "M2003.h"
#include "boot_mailbox.h"
#include "firmware_image.h"
#include "bl_internal.h"

static bl_boot_diagnostics_t boot_diagnostics;

static uint8_t take_boot_request(uint8_t *address)
{
    volatile boot_mailbox_t *mailbox =
        (volatile boot_mailbox_t *)FIRMWARE_BOOT_MAILBOX_BASE;
    uint32_t magic = mailbox->magic;
    uint32_t magic_inverse = mailbox->magic_inverse;
    uint8_t requested_address = mailbox->address;
    uint8_t requested_inverse = mailbox->address_inverse;

    mailbox->magic = 0u;
    mailbox->magic_inverse = 0u;
    __DSB();

    if (magic != BOOT_MAILBOX_MAGIC ||
        magic_inverse != ~BOOT_MAILBOX_MAGIC ||
        requested_address > 15u ||
        requested_inverse != (uint8_t)~requested_address)
        return 0u;

    *address = requested_address;
    return 1u;
}

static uint8_t application_valid(void)
{
    const firmware_manifest_t *manifest =
        (const firmware_manifest_t *)FIRMWARE_MANIFEST_BASE;
    uint32_t initial_stack;
    uint32_t reset_vector;
    uint32_t reset_address;

    if (manifest->magic != FIRMWARE_MANIFEST_MAGIC ||
        manifest->format_version != FIRMWARE_MANIFEST_VERSION ||
        manifest->header_size != FIRMWARE_MANIFEST_SIZE ||
        manifest->image_size < 8u ||
        manifest->image_size > FIRMWARE_APP_LIMIT ||
        (manifest->image_size & 3u) != 0u ||
        manifest->flags != 0u ||
        manifest->reserved != 0u)
        return 0u;

    if (bl_crc32(manifest, FIRMWARE_MANIFEST_SIZE - sizeof(uint32_t)) !=
        manifest->manifest_crc32)
        return 0u;

    if (bl_flash_read_word(FIRMWARE_APROM_BASE, &initial_stack) != 0 ||
        bl_flash_read_word(FIRMWARE_APROM_BASE + 4u, &reset_vector) != 0)
        return 0u;

    if (initial_stack < FIRMWARE_SRAM_BASE ||
        initial_stack > FIRMWARE_BOOT_MAILBOX_BASE ||
        (initial_stack & 3u) != 0u)
        return 0u;

    reset_address = reset_vector & ~1u;
    if ((reset_vector & 1u) == 0u || reset_address < 8u ||
        reset_address >= manifest->image_size)
        return 0u;

    return (bl_crc32((const void *)FIRMWARE_APROM_BASE,
                     manifest->image_size) == manifest->image_crc32) ? 1u : 0u;
}

static int reset_to_application(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (bl_flash_map_vectors(FIRMWARE_APROM_BASE) != 0)
        goto restore_loader;

    FMC_SET_APROM_BOOT();
    __DSB();

    if ((FMC->ISPCTL & FMC_ISPCTL_BS_Msk) != 0u)
        goto restore_loader;

    bl_uart_disable();
    __ISB();
    NVIC_SystemReset();
    for (;;) {
    }

restore_loader:
    if (bl_flash_map_vectors(FIRMWARE_LDROM_BASE) != 0) {
        for (;;) {
        }
    }
    FMC_SET_LDROM_BOOT();
    __DSB();
    __ISB();
    if (primask == 0u)
        __enable_irq();
    return -1;
}

static void restore_loader_vectors(void)
{
    if (bl_flash_map_vectors(FIRMWARE_LDROM_BASE) != 0) {
        __disable_irq();
        NVIC_SystemReset();
        for (;;) {
        }
    }
}

int main(void)
{
    uint8_t requested_address = 0xFFu;
    uint8_t requested;
    uint8_t valid;

    boot_diagnostics.reset_status = SYS->RSTSTS;
    bl_clock_init();
    boot_diagnostics.isp_control = FMC->ISPCTL;
    boot_diagnostics.isp_status = FMC->ISPSTS;
    bl_flash_init();
    requested = take_boot_request(&requested_address);

    if (bl_flash_map_vectors(FIRMWARE_APROM_BASE) != 0)
        valid = 0u;
    else
        valid = application_valid();

    restore_loader_vectors();
    bl_uart_init();
    bl_protocol_init(requested_address, valid, requested, &boot_diagnostics);

    if (requested == 0u && valid != 0u) {
        uint32_t elapsed_ms = 0u;

        SysTick->LOAD = 24000u - 1u;
        SysTick->VAL = 0u;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
        while (elapsed_ms < BL_BOOT_WINDOW_MS &&
               bl_protocol_hold_requested() == 0u) {
            bl_protocol_poll();
            bl_watchdog_service();
            if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u)
                elapsed_ms++;
        }
        SysTick->CTRL = 0u;
        if (bl_protocol_hold_requested() == 0u) {
            bl_uart_flush();
            (void)reset_to_application();
        }
    }

    for (;;) {
        bl_protocol_poll();
        bl_watchdog_service();
        if (bl_protocol_run_requested() != 0u) {
            bl_uart_flush();
            (void)reset_to_application();
        }
    }
}

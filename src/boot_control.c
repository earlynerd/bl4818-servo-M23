#include <stdint.h>
#include "M2003.h"
#include "app_uart.h"
#include "boot_control.h"
#include "boot_mailbox.h"

static volatile uint8_t enter_pending;
static volatile uint8_t pending_address;

#define BOOT_FMC_TIMEOUT 1000000UL

__attribute__((section(".boot_mailbox"), used))
static volatile boot_mailbox_t boot_mailbox;

static void boot_control_enable_isp(void)
{
    SYS_UnlockReg();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
}

static int boot_control_map_loader_vectors(void)
{
    uint32_t timeout = BOOT_FMC_TIMEOUT;

    FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADDR = FIRMWARE_LDROM_BASE & FMC_PAGE_ADDR_MASK;
    FMC->ISPDAT = 0u;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();
    while ((FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) != 0u) {
        if (--timeout == 0u)
            return -1;
    }

    if ((FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk) != 0u)
        return -1;
    return ((FMC->ISPSTS & FMC_ISPSTS_VECMAP_Msk) ==
            (FIRMWARE_LDROM_BASE & FMC_ISPSTS_VECMAP_Msk)) ? 0 : -1;
}

void boot_control_arm_loader(void)
{
    boot_control_enable_isp();
    FMC_SET_LDROM_BOOT();
    __DSB();

    /* System reset preserves the software-selected BS bit. Keep LDROM armed
     * while APROM runs so watchdog, fault, and requested resets recover there. */
    while ((FMC->ISPCTL & FMC_ISPCTL_BS_Msk) == 0u) {
    }
}

void boot_control_request(uint8_t address)
{
    if (address < 16u) {
        pending_address = address;
        enter_pending = 1u;
    }
}

uint8_t boot_control_pending(void)
{
    return enter_pending;
}

void boot_control_enter(void)
{
    uint8_t address = pending_address;

    uart_tx_flush();
    __disable_irq();

    boot_mailbox.address = address;
    boot_mailbox.address_inverse = (uint8_t)~address;
    boot_mailbox.reserved = 0u;
    boot_mailbox.reserved2 = 0u;
    boot_mailbox.magic_inverse = ~BOOT_MAILBOX_MAGIC;
    boot_mailbox.magic = BOOT_MAILBOX_MAGIC;
    __DSB();

    boot_control_enable_isp();
    if (boot_control_map_loader_vectors() != 0) {
        /* The application watchdog performs a hardware reset, which reloads
         * the configured LDROM-first boot state and preserves the mailbox. */
        for (;;) {
        }
    }
    FMC_SET_LDROM_BOOT();
    __DSB();
    while ((FMC->ISPCTL & FMC_ISPCTL_BS_Msk) == 0u) {
    }
    __ISB();
    NVIC_SystemReset();
    for (;;) {
    }
}

#ifndef BL_INTERNAL_H
#define BL_INTERNAL_H

#include <stdint.h>
#include "firmware_image.h"

#define BL_UART_BAUD 250000UL
#define BL_BOOT_WINDOW_MS 1000u

#define BL_ADDRESS_UNASSIGNED 0xFFu

typedef struct
{
    uint32_t reset_status;
    uint32_t isp_control;
    uint32_t isp_status;
} bl_boot_diagnostics_t;

void bl_early_safe_outputs(void);
void bl_clock_init(void);
void bl_watchdog_service(void);

void bl_flash_init(void);
int bl_flash_map_vectors(uint32_t address);
int bl_flash_erase_app_page(uint8_t page_index);
int bl_flash_erase_manifest(void);
int bl_flash_program_word(uint32_t address, uint32_t value);
int bl_flash_read_word(uint32_t address, uint32_t *value);
uint32_t bl_flash_read_uid(uint8_t index);
int bl_flash_crc32_app(uint32_t length, uint32_t *crc);
int bl_flash_commit_manifest(const firmware_manifest_t *manifest);

uint32_t bl_crc32(const void *data, uint32_t length);

void bl_uart_init(void);
void bl_uart_disable(void);
void bl_uart_echo_enable(void);
void bl_uart_echo_disable(void);
uint8_t bl_uart_available(void);
uint8_t bl_uart_getc(void);
uint8_t bl_uart_overflowed(void);
void bl_uart_clear_overflow(void);
void bl_uart_putc(uint8_t value);
void bl_uart_write(const uint8_t *data, uint32_t length);
void bl_uart_flush(void);

void bl_protocol_init(uint8_t address, uint8_t committed_valid,
                      uint8_t requested_entry,
                      const bl_boot_diagnostics_t *diagnostics);
void bl_protocol_poll(void);
uint8_t bl_protocol_hold_requested(void);
uint8_t bl_protocol_run_requested(void);

#endif /* BL_INTERNAL_H */

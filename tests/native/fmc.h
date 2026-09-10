#ifndef TEST_FMC_H
#define TEST_FMC_H
#include <stdint.h>

/* Flash I/O is deliberately not linked: these tests exercise record handling
 * in memory. Link-time garbage collection removes physical flash routines. */
#define FMC_APROM_BASE 0u
#define FMC_APROM_SIZE 32768u
#define FMC_FLASH_PAGE_SIZE 512u
void FMC_Open(void);
void FMC_Close(void);
void FMC_ENABLE_AP_UPDATE(void);
void FMC_DISABLE_AP_UPDATE(void);
int32_t FMC_Erase(uint32_t address);
int32_t FMC_Write(uint32_t address, uint32_t value);
uint32_t FMC_Read(uint32_t address);
#endif

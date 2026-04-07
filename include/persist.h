#ifndef PERSIST_H
#define PERSIST_H

#include <stdint.h>

void persist_init(void);
int32_t persist_save_runtime(void);
int32_t persist_clear(void);
uint8_t persist_is_valid(void);

#endif /* PERSIST_H */

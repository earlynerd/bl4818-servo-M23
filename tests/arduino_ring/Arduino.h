#pragma once
#include <stddef.h>
#include <stdint.h>
uint32_t millis();
void yield();
class Stream
{
public:
    virtual ~Stream() {}
    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(const uint8_t *, size_t) = 0;
};

#include "BL4818Ring.h"
#include <string.h>

namespace {
uint16_t crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xffff;
    while (length--)
    {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}
uint16_t u16(const uint8_t *p)
{
    return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}
int16_t s16(const uint8_t *p)
{
    uint16_t v = u16(p);
    return v < 0x8000 ? v : static_cast<int32_t>(v) - 65536;
}
}

BL4818Ring::BL4818Ring(Stream &serial, uint32_t timeout_ms)
    : serial_(serial), timeout_ms_(timeout_ms ? timeout_ms : 1),
      error_(Error::None), ack_{0xff, 0}, used_(0)
{
}

void BL4818Ring::reset_input()
{
    used_ = 0;
    // Bound the drain even if a faulty peer produces continuous traffic.
    int pending = serial_.available();
    while (pending-- > 0)
        serial_.read();
    error_ = Error::None;
    ack_ = {0xff, 0};
}

void BL4818Ring::discard(uint8_t count)
{
    used_ -= count;
    memmove(rx_, rx_ + count, used_);
}

bool BL4818Ring::send(const uint8_t *payload, uint8_t length)
{
    uint8_t frame[69] = {0xa5, 0x5a, length};
    memcpy(frame + 3, payload, length);
    uint16_t crc = crc16(frame + 2, length + 1);
    frame[3 + length] = crc >> 8;
    frame[4 + length] = crc & 0xff;
    if (serial_.write(frame, length + 5) != static_cast<size_t>(length + 5))
    {
        error_ = Error::WriteFailed;
        return false;
    }
    return true;
}

bool BL4818Ring::receive(uint8_t *payload, uint8_t &length, uint32_t start)
{
    while (static_cast<uint32_t>(millis() - start) < timeout_ms_)
    {
        // Single-byte slip preserves nested frames after invalid length/CRC.
        while (used_ >= 2)
        {
            if (rx_[0] != 0xa5 || rx_[1] != 0x5a)
            {
                discard(1);
                continue;
            }
            if (used_ < 3) break;
            uint8_t n = rx_[2];
            if (n == 0 || n > 64)
            {
                discard(1);
                continue;
            }
            if (used_ < n + 5) break;
            if (crc16(rx_ + 2, n + 1) != u16(rx_ + n + 3))
            {
                discard(1);
                continue;
            }
            memcpy(payload, rx_ + 3, n);
            length = n;
            discard(n + 5);
            return true;
        }
        int byte = serial_.read();
        if (byte >= 0)
            rx_[used_++] = static_cast<uint8_t>(byte);
        else
            yield();
    }
    error_ = Error::Timeout;
    return false;
}

bool BL4818Ring::transition(uint8_t type)
{
    reset_input();
    uint32_t start = millis();
    if (!send(&type, 1)) return false;
    uint8_t p[64], n;
    while (receive(p, n, start))
        if (n == 1 && p[0] == type) return true;
    return false;
}

bool BL4818Ring::enumerate(uint8_t &count)
{
    count = 0;
    if (!transition(0x01)) return false;
    reset_input();
    const uint8_t request[] = {0x03, 0};
    uint32_t start = millis();
    if (!send(request, sizeof(request))) return false;
    uint8_t p[64], n;
    while (receive(p, n, start))
    {
        if (n != 2 || p[0] != 0x03 || p[1] == 0) continue;
        if (p[1] > 16)
        {
            error_ = Error::InvalidArgument;
            return false;
        }
        uint8_t found = p[1];
        if (!transition(0x02)) return false;
        count = found;
        return true;
    }
    return false;
}

bool BL4818Ring::command(uint8_t address, uint8_t subcommand,
                         const uint8_t *data, uint8_t length)
{
    if (address > 15 || !subcommand || subcommand > 0x3f || length > 6 ||
        (length && !data) || subcommand == 0x10 || subcommand == 0x11 ||
        subcommand == 0x16 || subcommand == 0x19 || subcommand == 0x1c ||
        subcommand == 0x1b)
    {
        error_ = Error::InvalidArgument;
        return false;
    }
    reset_input();
    uint8_t p[64] = {static_cast<uint8_t>(0x20 + address),
                     static_cast<uint8_t>(subcommand | 0x40)};
    if (length) memcpy(p + 2, data, length);
    uint32_t start = millis();
    if (!send(p, length + 2)) return false;
    uint8_t n;
    while (receive(p, n, start))
    {
        if (n != 5 || p[0] != 0x50 + address || p[1] != subcommand) continue;
        ack_ = {p[2], u16(p + 3)};
        if (ack_.result <= 1) return true;
        error_ = Error::Rejected;
        return false;
    }
    return false;
}

bool BL4818Ring::command16(uint8_t address, uint8_t subcommand, uint16_t value)
{
    uint8_t data[] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
    return command(address, subcommand, data, sizeof(data));
}
bool BL4818Ring::stop(uint8_t address) { return command(address, 0x03); }
bool BL4818Ring::clear_fault(uint8_t address) { return command(address, 0x04); }
bool BL4818Ring::set_mode(uint8_t address, Mode mode)
{
    uint8_t value = static_cast<uint8_t>(mode);
    if (value > 3) { error_ = Error::InvalidArgument; return false; }
    return command(address, 0x05, &value, 1);
}
bool BL4818Ring::set_velocity(uint8_t address, int16_t motor_rpm)
{
    return command16(address, 0x06, static_cast<uint16_t>(motor_rpm));
}
bool BL4818Ring::set_duty(uint8_t address, int16_t duty)
{
    return command16(address, 0x01, static_cast<uint16_t>(duty));
}
bool BL4818Ring::set_torque_limit(uint8_t address, uint16_t milliamps)
{
    return command16(address, 0x02, milliamps);
}
bool BL4818Ring::set_position(uint8_t address, int32_t counts)
{
    uint32_t v = static_cast<uint32_t>(counts);
    uint8_t data[] = {static_cast<uint8_t>(v >> 24), static_cast<uint8_t>(v >> 16),
                      static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v)};
    return command(address, 0x09, data, sizeof(data));
}
bool BL4818Ring::query_status(uint8_t address, Status &status)
{
    if (address > 15) { error_ = Error::InvalidArgument; return false; }
    reset_input();
    uint8_t p[64] = {static_cast<uint8_t>(0x20 + address), 0x10};
    uint32_t start = millis();
    if (!send(p, 2)) return false;
    uint8_t n;
    while (receive(p, n, start))
    {
        if (n != 17 || p[0] != 0x40 + address) continue;
        uint32_t position = (static_cast<uint32_t>(u16(p + 13)) << 16) | u16(p + 15);
        status = {p[1], p[2], p[3], u16(p + 4), p[6], u16(p + 7),
                  s16(p + 9), s16(p + 11),
                  static_cast<int32_t>(position < 0x80000000UL ?
                      static_cast<int64_t>(position) : static_cast<int64_t>(position) - 4294967296LL)};
        return true;
    }
    return false;
}

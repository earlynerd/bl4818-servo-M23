#include "BL4818Ring.h"
#include <assert.h>
#include <stdio.h>
#include <deque>
#include <functional>
#include <vector>

using Bytes = std::vector<uint8_t>;
static uint32_t clock_ms;
uint32_t millis() { return clock_ms++; }
void yield() {}

// Independent bitwise reference with a standard CRC check vector below.
static uint16_t crc(const Bytes &bytes)
{
    unsigned value = 65535;
    for (uint8_t b : bytes)
    {
        value ^= unsigned(b) * 256;
        for (int i = 0; i != 8; ++i)
            value = ((value * 2) ^ ((value & 32768) ? 4129 : 0)) & 65535;
    }
    return static_cast<uint16_t>(value);
}
static Bytes frame(Bytes payload)
{
    Bytes checked{static_cast<uint8_t>(payload.size())};
    checked.insert(checked.end(), payload.begin(), payload.end());
    uint16_t value = crc(checked);
    Bytes out{0xa5, 0x5a};
    out.insert(out.end(), checked.begin(), checked.end());
    out.push_back(value >> 8);
    out.push_back(value & 255);
    return out;
}
class Fake : public Stream
{
public:
    std::deque<uint8_t> rx;
    std::vector<Bytes> sent;
    std::function<void(const Bytes &)> respond;
    bool short_write = false;
    int available() override { return static_cast<int>(rx.size()); }
    int read() override
    {
        if (rx.empty()) return -1;
        int b = rx.front(); rx.pop_front(); return b;
    }
    void push(const Bytes &b) { rx.insert(rx.end(), b.begin(), b.end()); }
    size_t write(const uint8_t *p, size_t n) override
    {
        Bytes b(p, p + n);
        assert(b.size() >= 6 && b[0] == 0xa5 && b[1] == 0x5a && b[2] == n - 5);
        assert(crc(Bytes(b.begin() + 2, b.end() - 2)) == ((b[n-2] << 8) | b[n-1]));
        Bytes payload(b.begin() + 3, b.end() - 2);
        sent.push_back(payload);
        if (respond) respond(payload);
        return short_write ? n - 1 : n;
    }
};

int main()
{
    assert(crc({'1','2','3','4','5','6','7','8','9'}) == 0x29b1);
    Fake serial;
    BL4818Ring ring(serial, 1000);
    serial.respond = [&](const Bytes &p) {
        serial.push(frame(p)); // Normal ring echo must not satisfy ACK wait.
        serial.push(frame({0x51, 6, 0, 0, 0})); // Wrong address.
        serial.push(frame({0x50, 3, 0, 0, 0})); // Wrong subcommand.
        serial.push(frame({0x50, 6, 0, 0, 0}));
    };
    assert(ring.set_velocity(0, -123));
    assert(serial.sent.back() == Bytes({0x20, 0x46, 0xff, 0x85}));
    assert(ring.last_ack().result == 0);
    serial.respond = [&](const Bytes &p) {
        serial.push(frame({static_cast<uint8_t>(0x50 + (p[0] & 15)),
                           static_cast<uint8_t>(p[1] & 63), 0, 0, 0}));
    };
    assert(ring.set_position(15, INT32_MIN));
    assert(serial.sent.back() == Bytes({0x2f, 0x49, 0x80, 0, 0, 0}));
    assert(ring.set_torque_limit(0, 500));
    assert(serial.sent.back() == Bytes({0x20, 0x42, 1, 244}));
    assert(ring.set_duty(0, -1200));
    assert(serial.sent.back() == Bytes({0x20, 0x41, 0xfb, 0x50}));
    assert(ring.set_mode(0, BL4818Ring::Mode::Velocity));
    assert(serial.sent.back() == Bytes({0x20, 0x45, 1}));
    size_t before = serial.sent.size();
    assert(!ring.stop(16) && ring.error() == BL4818Ring::Error::InvalidArgument);
    assert(!ring.command(0, 0x10));
    assert(!ring.command(0, 1, nullptr, 1));
    assert(serial.sent.size() == before);

    serial.respond = [&](const Bytes &) { serial.push(frame({0x50, 6, 3, 0x12, 0x34})); };
    assert(!ring.set_velocity(0, 20));
    assert(ring.error() == BL4818Ring::Error::Rejected && ring.last_ack().detail == 0x1234);

    serial.respond = [&](const Bytes &) {
        serial.push({0, 0xa5, 0xa5, 0x5a, 0, 0xa5, 0x5a, 65});
        Bytes bad = frame({0x50, 3, 0, 0, 0}); bad.back() ^= 1; serial.push(bad);
        // Invalid outer CRC encloses a valid ACK: single-byte rescan must find it.
        Bytes nested{0xa5, 0x5a, 10};
        Bytes good = frame({0x50, 3, 0, 0, 0});
        nested.insert(nested.end(), good.begin(), good.end());
        nested.insert(nested.end(), {0, 0}); serial.push(nested);
    };
    assert(ring.stop(0));

    serial.respond = [&](const Bytes &) {
        serial.push(frame({0x40, 1, 0, 1, 1, 244, 5, 0x3f, 0xff,
                           0xff, 0x85, 0x80, 0, 0xff, 0xff, 0xff, 0xfe}));
    };
    BL4818Ring::Status status{};
    assert(ring.query_status(0, status));
    assert(status.current_ma == 500 && status.angle == 16383);
    assert(status.velocity_rpm == -123 && status.target_rpm == -32768 && status.position == -2);

    serial.respond = [&](const Bytes &p) {
        if (p == Bytes({1})) serial.push(frame({1}));
        else if (p == Bytes({3, 0}))
        {
            serial.push(frame({1})); serial.push(frame({3, 0})); serial.push(frame({3, 16}));
        }
        else { assert(p == Bytes({2})); serial.push(frame({2})); }
    };
    uint8_t count = 99;
    assert(ring.enumerate(count) && count == 16);
    serial.respond = [&](const Bytes &p) {
        if (p == Bytes({1})) serial.push(frame({1}));
        else if (p == Bytes({3, 0})) serial.push(frame({3, 1}));
        // Missing cut-through return must fail enumeration.
    };
    assert(!ring.enumerate(count) && count == 0);
    serial.respond = [&](const Bytes &p) {
        serial.push(frame(p == Bytes({1}) ? p : Bytes({3, 17})));
    };
    assert(!ring.enumerate(count) && count == 0 &&
           ring.error() == BL4818Ring::Error::InvalidArgument);

    serial.respond = [&](const Bytes &) {
        serial.push(frame(Bytes(64, 0))); // Maximum-size unrelated frame.
        serial.push(frame({0x50, 3, 0, 0, 0}));
    };
    assert(ring.stop(0));
    serial.respond = [&](const Bytes &) { serial.push({0xa5, 0x5a, 17, 0x40}); };
    assert(!ring.query_status(0, status) && ring.error() == BL4818Ring::Error::Timeout);
    // Deadline is not extended by a stream of valid but unrelated frames.
    serial.respond = [&](const Bytes &) {
        for (int i = 0; i < 500; ++i) serial.push(frame({0x51, 3, 0, 0, 0}));
    };
    uint32_t started = clock_ms;
    assert(!ring.stop(0) && ring.error() == BL4818Ring::Error::Timeout);
    assert(static_cast<uint32_t>(clock_ms - started) < 1010);

    serial.respond = nullptr;
    serial.push(frame({0x50, 3, 0, 0, 0})); // Stale queued ACK is discarded.
    clock_ms = UINT32_MAX - 5;
    assert(!ring.stop(0) && ring.error() == BL4818Ring::Error::Timeout);
    assert(!ring.query_status(0, status) && status.position == -2);
    serial.short_write = true;
    assert(!ring.stop(0) && ring.error() == BL4818Ring::Error::WriteFailed);
    puts("BL4818Ring protocol tests passed");
}

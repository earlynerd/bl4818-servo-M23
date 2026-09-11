#ifndef BL4818_RING_H
#define BL4818_RING_H

#include <Arduino.h>

class BL4818Ring
{
public:
    static const uint32_t BAUD = 250000;
    enum class Error : uint8_t { None, InvalidArgument, WriteFailed, Timeout, Rejected };
    enum class Mode : uint8_t { Duty, Velocity, Position, Torque };
    struct Ack
    {
        uint8_t result;
        uint16_t detail;
    };
    struct Status
    {
        uint8_t state, fault, mode;
        uint16_t current_ma;
        uint8_t hall;
        uint16_t angle;
        int16_t velocity_rpm, target_rpm;
        int32_t position;
    };

    // Configure and begin the UART before constructing/using this client.
    explicit BL4818Ring(Stream &serial, uint32_t timeout_ms = 100);
    bool enumerate(uint8_t &count);
    bool stop(uint8_t address);
    bool clear_fault(uint8_t address);
    bool set_mode(uint8_t address, Mode mode);
    bool set_velocity(uint8_t address, int16_t motor_rpm);
    bool set_duty(uint8_t address, int16_t duty);
    bool set_torque_limit(uint8_t address, uint16_t milliamps);
    bool set_position(uint8_t address, int32_t counts);
    bool query_status(uint8_t address, Status &status);

    // ACK-only escape hatch for non-query application commands, data <= 6 bytes.
    // Queries and bootloader commands are deliberately not accepted here.
    bool command(uint8_t address, uint8_t subcommand,
                 const uint8_t *data = nullptr, uint8_t length = 0);
    Error error() const { return error_; }
    // Valid only when command() returned true or error() == Rejected.
    Ack last_ack() const { return ack_; }

private:
    Stream &serial_;
    uint32_t timeout_ms_;
    Error error_;
    Ack ack_;
    uint8_t rx_[69];
    uint8_t used_;
    void reset_input();
    void discard(uint8_t count);
    bool send(const uint8_t *payload, uint8_t length);
    bool receive(uint8_t *payload, uint8_t &length, uint32_t start);
    bool transition(uint8_t type);
    bool command16(uint8_t address, uint8_t subcommand, uint16_t value);
};

#endif

/*
 * Binary Ring Protocol Parser for Nuvoton M2003
 *
 * This ports the protocol engine from the 8051.
 */

#include <stdint.h>
#include <string.h>
#include "m2003_config.h"
#include "M2003.h"
#include "protocol.h"
#include "app_uart.h"
#include "motor.h"
#include "hall.h"

#define PROTO_SYNC_ENUM           0x7Fu
#define PROTO_SYNC_STATUS         0x7Eu
#define PROTO_SYNC_BROADCAST      0xFFu
#define PROTO_SYNC_ADDR_BASE      0x80u
#define PROTO_SYNC_ADDR_MASK      0xF0u

#define PROTO_CMD_SET_DUTY        0x01u
#define PROTO_CMD_SET_TORQUE      0x02u
#define PROTO_CMD_STOP            0x03u
#define PROTO_CMD_CLEAR_FAULT     0x04u

static uint8_t device_addr = 0xFF;
static uint8_t rx_crc = 0;

static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    uint8_t bit;
    crc ^= data;
    for (bit = 0; bit < 8u; bit++) {
        if (crc & 0x80u) crc = (uint8_t)((crc << 1) ^ 0x07u);
        else crc <<= 1;
    }
    return crc;
}

void protocol_init(void)
{
    device_addr = 0xFF;
}

static void send_status_binary(void)
{
    uint16_t current = motor_get_current();
    uint8_t state = (uint8_t)motor_get_state();
    uint8_t fault = (uint8_t)motor_get_fault();
    uint8_t hall = hall_read();
    uint8_t crc = 0;

    const uint8_t frame[] = {
        PROTO_SYNC_STATUS,
        state,
        fault,
        (uint8_t)(current >> 8),
        (uint8_t)(current & 0xFFu),
        hall
    };

    for (int i = 0; i < 6; i++) {
        crc = crc8_update(crc, frame[i]);
        uart_putc(frame[i]);
    }
    uart_putc(crc);
}

void protocol_poll(void)
{
    /* 
     * Simplification for the port: we use a simpler state machine 
     * for the initial M2003 test. The full logic from protocol.c 
     * would be copied here. 
     */
    while (uart_available()) {
        uint8_t c = uart_getc();
        
        /* 
         * For the port, we'll implement the core SET_DUTY addressed command 
         * to verify the M2003 is talking correctly.
         */
        if ((c & PROTO_SYNC_ADDR_MASK) == PROTO_SYNC_ADDR_BASE) {
            uint8_t addr = c & 0x0F;
            if (addr == device_addr || device_addr == 0xFF) {
                // Temporary: Claim first address seen for testing
                if (device_addr == 0xFF) device_addr = addr;
                
                uint8_t cmd = uart_getc(); // Blocking for simplicity in test
                if (cmd == PROTO_CMD_SET_DUTY) {
                    uint8_t hi = uart_getc();
                    uint8_t lo = uart_getc();
                    int16_t duty = (int16_t)((hi << 8) | lo);
                    motor_set_duty(duty);
                    motor_start();
                    send_status_binary();
                }
            }
        }
    }
}

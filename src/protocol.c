/*
 * Ring Bus Protocol v2 for Nuvoton M2003
 *
 * Framing: [0xA5] [0x5A] [LEN] [payload...] [CRC16_HI] [CRC16_LO]
 * See protocol.md for full specification.
 */

#include <stdint.h>
#include "m2003_config.h"
#include "M2003.h"
#include "protocol.h"
#include "app_uart.h"
#include "crc16.h"
#include "motor.h"
#include "hall.h"
#include "encoder.h"
/* Enable for parser state tracing: #define DBG(c) uart_putc(c) */
#define DBG(c) ((void)0)

/* ── Framing constants ────────────────────────────────────────────────── */
#define PREAMBLE_0              0xA5u
#define PREAMBLE_1              0x5Au
#define MAX_PAYLOAD             34u
#define FRAME_BUF_SIZE          (1u + MAX_PAYLOAD + 2u) /* LEN + payload + CRC */
#define ADDR_UNASSIGNED         0xFFu
#define MAX_DEVICES             16u
#define FRAME_TIMEOUT_TICKS     10u   /* 5 ms at 2 kHz */
#define POLL_BYTE_BUDGET        48u

/* ── Command types ────────────────────────────────────────────────────── */
#define CMD_ENTER_SF            0x01u
#define CMD_ENTER_CT            0x02u
#define CMD_SET_ADDRESS         0x03u
#define CMD_BROADCAST_DUTY      0x10u
#define CMD_ADDR_BASE           0x20u
#define CMD_ADDR_END            0x30u
#define CMD_STATUS_BASE         0x40u
#define CMD_STATUS_END          0x50u

/* ── Addressed sub-commands ───────────────────────────────────────────── */
#define SUBCMD_SET_DUTY         0x01u
#define SUBCMD_SET_TORQUE       0x02u
#define SUBCMD_STOP             0x03u
#define SUBCMD_CLEAR_FAULT      0x04u
#define SUBCMD_QUERY_STATUS     0x10u

/* ── Forwarding mode ──────────────────────────────────────────────────── */
typedef enum {
    FWD_CUT_THROUGH = 0,
    FWD_STORE_AND_FORWARD
} fwd_mode_t;

/* ── Receiver phase ───────────────────────────────────────────────────── */
typedef enum {
    RX_SCAN_0 = 0,     /* looking for 0xA5 */
    RX_SCAN_1,          /* got 0xA5, looking for 0x5A */
    RX_BUFFERING        /* reading LEN + payload + CRC */
} rx_phase_t;

/* ── State ────────────────────────────────────────────────────────────── */
static uint8_t device_addr;
static fwd_mode_t fwd_mode;
static fwd_mode_t frame_start_mode; /* mode when preamble was detected */

static uint8_t frame_buf[FRAME_BUF_SIZE];
static uint8_t frame_pos;
static uint8_t frame_expected;
static rx_phase_t rx_phase;
static uint8_t rx_timeout;

/* ── Helpers ──────────────────────────────────────────────────────────── */

static void send_raw(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
        uart_putc(data[i]);
}

static void send_frame(const uint8_t *frame_body, uint8_t body_len)
{
    uart_putc(PREAMBLE_0);
    uart_putc(PREAMBLE_1);
    send_raw(frame_body, body_len);
}

static void forward_frame_verbatim(void)
{
    uint8_t total = 1u + frame_buf[0] + 2u;
    send_frame(frame_buf, total);
}

static void reset_receiver(void)
{
    rx_phase = RX_SCAN_0;
    frame_pos = 0;
    frame_expected = 0;
    rx_timeout = 0;
}

static uint8_t prepare_set_duty(int16_t duty)
{
    if (duty < -(int16_t)PWM_MAX_DUTY || duty > (int16_t)PWM_MAX_DUTY)
        return 0u;

    if (duty == 0) {
        motor_stop();
        return 0u;
    }

    if (motor_get_state() == MOTOR_FAULT)
        return 0u;

    motor_set_duty(duty);
    return 1u;
}

static void send_status_reply(void)
{
    uint16_t current = motor_get_current();
    uint16_t angle = encoder_get_angle();
    uint8_t buf[11];
    uint16_t crc;

    buf[0] = 8u;
    buf[1] = CMD_STATUS_BASE | device_addr;
    buf[2] = (uint8_t)motor_get_state();
    buf[3] = (uint8_t)motor_get_fault();
    buf[4] = (uint8_t)(current >> 8);
    buf[5] = (uint8_t)(current & 0xFFu);
    buf[6] = hall_read();
    buf[7] = (uint8_t)(angle >> 8);
    buf[8] = (uint8_t)(angle & 0xFFu);

    crc = crc16_ccitt(buf, 9);
    buf[9] = (uint8_t)(crc >> 8);
    buf[10] = (uint8_t)(crc & 0xFFu);

    send_frame(buf, 11);
}

/* ── Command handlers ─────────────────────────────────────────────────── */

static void handle_set_address(const uint8_t *payload, uint8_t len)
{
    uint8_t counter, next;
    uint8_t out[5];
    uint16_t crc;

    if (len < 2u) return;

    counter = payload[1];
    if (counter >= MAX_DEVICES) return;

    if (device_addr == ADDR_UNASSIGNED)
        device_addr = counter;

    next = counter + 1u;

    out[0] = 2u;
    out[1] = CMD_SET_ADDRESS;
    out[2] = next;
    crc = crc16_ccitt(out, 3);
    out[3] = (uint8_t)(crc >> 8);
    out[4] = (uint8_t)(crc & 0xFFu);

    send_frame(out, 5);
}

static void handle_broadcast_duty(const uint8_t *payload, uint8_t len)
{
    uint8_t n_slots, offset;
    int16_t duty;
    uint8_t start_after;

    if (device_addr == ADDR_UNASSIGNED) return;

    n_slots = (uint8_t)((len - 1u) / 2u);
    if (device_addr >= n_slots) return;

    offset = 1u + 2u * device_addr;
    if (offset + 1u >= len) return;

    duty = (int16_t)(((uint16_t)payload[offset] << 8) | payload[offset + 1u]);

    start_after = prepare_set_duty(duty);

    if (fwd_mode == FWD_STORE_AND_FORWARD)
        forward_frame_verbatim();

    if (start_after)
        motor_start();
}

static void handle_addressed_cmd(uint8_t cmd_type, const uint8_t *payload, uint8_t len)
{
    uint8_t target = cmd_type & 0x0Fu;
    uint8_t subcmd;
    uint8_t start_after;

    if (device_addr == ADDR_UNASSIGNED || target != device_addr) {
        if (fwd_mode == FWD_STORE_AND_FORWARD)
            forward_frame_verbatim();
        return;
    }

    subcmd = (len >= 2u) ? payload[1] : 0u;

    switch (subcmd) {
    case SUBCMD_SET_DUTY:
        if (len >= 4u) {
            int16_t duty = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
            start_after = prepare_set_duty(duty);
            send_status_reply();
            if (start_after) motor_start();
        }
        break;
    case SUBCMD_SET_TORQUE:
        if (len >= 4u) {
            uint16_t ma = ((uint16_t)payload[2] << 8) | payload[3];
            motor_set_torque_limit(ma);
        }
        send_status_reply();
        break;
    case SUBCMD_STOP:
        motor_stop();
        send_status_reply();
        break;
    case SUBCMD_CLEAR_FAULT:
        motor_clear_fault();
        send_status_reply();
        break;
    case SUBCMD_QUERY_STATUS:
        send_status_reply();
        break;
    default:
        break;
    }
}

/* ── Frame dispatch ───────────────────────────────────────────────────── */

static void dispatch(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd) {

    case CMD_ENTER_SF:
        fwd_mode = FWD_STORE_AND_FORWARD;
        uart_echo_disable();
        if (frame_start_mode == FWD_STORE_AND_FORWARD)
            forward_frame_verbatim();
        /* if was CT when frame arrived, ISR already echoed it */
        break;

    case CMD_ENTER_CT:
        fwd_mode = FWD_CUT_THROUGH;
        uart_echo_enable();
        if (frame_start_mode == FWD_STORE_AND_FORWARD)
            forward_frame_verbatim();
        break;

    case CMD_SET_ADDRESS:
        handle_set_address(payload, len);
        break;

    case CMD_BROADCAST_DUTY:
        handle_broadcast_duty(payload, len);
        break;

    default:
        if (cmd >= CMD_ADDR_BASE && cmd < CMD_ADDR_END) {
            handle_addressed_cmd(cmd, payload, len);
        } else if (cmd >= CMD_STATUS_BASE && cmd < CMD_STATUS_END) {
            if (fwd_mode == FWD_STORE_AND_FORWARD)
                forward_frame_verbatim();
        }
        break;
    }
}

static void process_frame(void)
{
    uint8_t len = frame_buf[0];
    uint8_t *payload = &frame_buf[1];
    uint16_t rx_crc = ((uint16_t)frame_buf[1 + len] << 8) | frame_buf[2 + len];
    uint16_t calc_crc = crc16_ccitt(frame_buf, 1u + len);

    if (calc_crc != rx_crc)
        return;

    dispatch(payload[0], payload, len);
}

/* ── Public API ───────────────────────────────────────────────────────── */

void protocol_init(void)
{
    device_addr = ADDR_UNASSIGNED;
    fwd_mode = FWD_CUT_THROUGH;
    uart_echo_enable();
    uart_rx_flush();
    reset_receiver();
}

void protocol_poll(void)
{
    uint8_t budget = POLL_BYTE_BUDGET;

    if (uart_rx_overflowed()) {
        uart_rx_flush();
        uart_rx_clear_overflow();
        reset_receiver();
        return;
    }

    while (budget != 0u && uart_available()) {
        uint8_t c = uart_getc();
        budget--;

        switch (rx_phase) {

        case RX_SCAN_0:
            if (c == PREAMBLE_0) {
                rx_phase = RX_SCAN_1;
                rx_timeout = FRAME_TIMEOUT_TICKS;
                DBG('A');
            }
            break;

        case RX_SCAN_1:
            if (c == PREAMBLE_1) {
                rx_phase = RX_BUFFERING;
                frame_pos = 0;
                frame_expected = 0;
                frame_start_mode = fwd_mode;
                DBG('B');
            } else if (c == PREAMBLE_0) {
                /* could be start of a new preamble, stay in SCAN_1 */
            } else {
                rx_phase = RX_SCAN_0;
                rx_timeout = 0;
            }
            break;

        case RX_BUFFERING:
            if (frame_pos < FRAME_BUF_SIZE)
                frame_buf[frame_pos] = c;
            frame_pos++;

            if (frame_pos == 1u) {
                uint8_t len = frame_buf[0];
                if (len == 0u || len > MAX_PAYLOAD) {
                    reset_receiver();
                    break;
                }
                frame_expected = 1u + len + 2u;
                DBG('L');
            }

            if (frame_expected != 0u && frame_pos >= frame_expected) {
                DBG('F');
                process_frame();
                reset_receiver();
            }
            break;
        }

        if (uart_rx_overflowed()) {
            uart_rx_flush();
            uart_rx_clear_overflow();
            reset_receiver();
            return;
        }
    }
}

void protocol_tick(void)
{
    if (rx_phase != RX_SCAN_0 && rx_timeout != 0u) {
        rx_timeout--;
        if (rx_timeout == 0u)
            reset_receiver();
    }
}

uint8_t protocol_is_enumerated(void)
{
    return (device_addr != ADDR_UNASSIGNED) ? 1u : 0u;
}

uint8_t protocol_get_address(void)
{
    return device_addr;
}

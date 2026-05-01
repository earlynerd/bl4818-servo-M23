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
#include "persist.h"
#include "strike.h"
#include "timing.h"
#include "irq_util.h"
/* Enable for parser state tracing: #define DBG(c) uart_putc(c) */
#define DBG(c) ((void)0)

/* ── Framing constants ────────────────────────────────────────────────── */
#define PREAMBLE_0              0xA5u
#define PREAMBLE_1              0x5Au
#define MAX_PAYLOAD             64u
#define FRAME_BUF_SIZE          (1u + MAX_PAYLOAD + 2u) /* LEN + payload + CRC */
#define ADDR_UNASSIGNED         0xFFu
#define MAX_DEVICES             16u
#define FRAME_TIMEOUT_TICKS     HZ_TICKS_FROM_MS(PROTOCOL_TICK_HZ, PROTOCOL_FRAME_TIMEOUT_MS)
#define POLL_BYTE_BUDGET        128u

/* ── Command types ────────────────────────────────────────────────────── */
#define CMD_ENTER_SF            0x01u
#define CMD_ENTER_CT            0x02u
#define CMD_SET_ADDRESS         0x03u
#define CMD_BROADCAST_DUTY      0x10u
#define CMD_ADDR_BASE           0x20u
#define CMD_ADDR_END            0x30u
#define CMD_STATUS_BASE         0x40u
#define CMD_STATUS_END          0x50u
#define CMD_ACK_BASE            0x50u
#define CMD_ACK_END             0x60u

/* ── Addressed sub-commands ───────────────────────────────────────────── */
#define SUBCMD_SET_DUTY         0x01u
#define SUBCMD_SET_TORQUE       0x02u
#define SUBCMD_STOP             0x03u
#define SUBCMD_CLEAR_FAULT      0x04u
#define SUBCMD_SET_MODE         0x05u
#define SUBCMD_SET_VELOCITY     0x06u
#define SUBCMD_SET_PID          0x07u
#define SUBCMD_SET_FF           0x08u
#define SUBCMD_SET_POSITION     0x09u
#define SUBCMD_SET_POS_PID      0x0Au
#define SUBCMD_ZERO_POSITION    0x0Bu
#define SUBCMD_STRIKE           0x0Cu
#define SUBCMD_STRIKE_HOME      0x0Du
#define SUBCMD_STRIKE_CANCEL    0x0Eu
#define SUBCMD_SET_STRIKE_PARAM 0x0Fu
#define SUBCMD_QUERY_STATUS     0x10u
#define SUBCMD_QUERY_STRIKE     0x11u
#define SUBCMD_SAVE_SETTINGS    0x12u
#define SUBCMD_CLEAR_SETTINGS   0x13u
#define SUBCMD_SET_CUR_PID      0x14u
#define SUBCMD_SET_CURRENT      0x15u
#define SUBCMD_QUERY_TIMING     0x16u
#define SUBCMD_DETECT_CSN_POLARITY 0x17u
#define SUBCMD_SET_CSN_POLARITY    0x18u
#define SUBCMD_MASK             0x3Fu
#define SUBCMD_REPLY_MASK       0xC0u
#define SUBCMD_REPLY_FULL       0x00u
#define SUBCMD_REPLY_ACK        0x40u
#define SUBCMD_REPLY_NONE       0x80u

/* ── Minimal ACK result codes ─────────────────────────────────────────── */
#define ACK_RESULT_OK               0x00u
#define ACK_RESULT_OK_RETRIGGERED   0x01u
#define ACK_RESULT_REJECT_NOT_HOMED 0x02u
#define ACK_RESULT_REJECT_FAULT     0x03u
#define ACK_RESULT_REJECT_ZERO      0x04u
#define ACK_RESULT_REJECT_NOT_READY 0x05u
#define ACK_RESULT_INVALID_ARGUMENT 0x06u
#define ACK_RESULT_PERSIST_FAILED   0x07u

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

    motor_set_mode(CTRL_DUTY);
    motor_set_duty(duty);
    return 1u;
}

static uint8_t prepare_set_velocity(int16_t rpm)
{
    uint32_t irq_state;

    if (motor_get_state() == MOTOR_FAULT)
        return 0u;

    irq_state = irq_save();
    motor_set_velocity(rpm);
    motor_set_mode(CTRL_VELOCITY);
    irq_restore(irq_state);
    return 1u;
}

static uint8_t prepare_set_position(int32_t counts)
{
    if (motor_get_state() == MOTOR_FAULT)
        return 0u;

    motor_set_position(counts);
    motor_set_mode(CTRL_POSITION);
    return 1u;
}

static void send_status_reply(void)
{
    uint32_t irq_state;
    uint8_t motor_state;
    uint8_t motor_fault;
    uint8_t motor_mode;
    uint16_t current;
    uint16_t angle;
    int16_t velocity;
    int16_t target;
    int32_t position;
    uint8_t hall_state;
    uint8_t buf[20];
    uint16_t crc;

    irq_state = irq_save();
    motor_state = (uint8_t)motor_get_state();
    motor_fault = (uint8_t)motor_get_fault();
    motor_mode = (uint8_t)motor_get_mode();
    current = (uint16_t)motor_get_current();
    angle = encoder_get_angle();
    velocity = (int16_t)motor_get_velocity();
    target = (int16_t)motor_get_target();
    position = -encoder_get_position(); /* negate: forward = positive */
    hall_state = hall_read();
    irq_restore(irq_state);

    buf[0]  = 17u;
    buf[1]  = CMD_STATUS_BASE | device_addr;
    buf[2]  = motor_state;
    buf[3]  = motor_fault;
    buf[4]  = motor_mode;
    buf[5]  = (uint8_t)(current >> 8);
    buf[6]  = (uint8_t)(current & 0xFFu);
    buf[7]  = hall_state;
    buf[8]  = (uint8_t)(angle >> 8);
    buf[9]  = (uint8_t)(angle & 0xFFu);
    buf[10] = (uint8_t)((uint16_t)velocity >> 8);
    buf[11] = (uint8_t)((uint16_t)velocity & 0xFFu);
    buf[12] = (uint8_t)((uint16_t)target >> 8);
    buf[13] = (uint8_t)((uint16_t)target & 0xFFu);
    buf[14] = (uint8_t)((uint32_t)position >> 24);
    buf[15] = (uint8_t)((uint32_t)position >> 16);
    buf[16] = (uint8_t)((uint32_t)position >> 8);
    buf[17] = (uint8_t)((uint32_t)position & 0xFFu);

    crc = crc16_ccitt(buf, 18);
    buf[18] = (uint8_t)(crc >> 8);
    buf[19] = (uint8_t)(crc & 0xFFu);

    send_frame(buf, 20);
}

/* Strike parameter IDs */
#define STRIKE_PARAM_HOME_OFFSET     0x01u
#define STRIKE_PARAM_COAST_DISTANCE  0x02u
#define STRIKE_PARAM_HOMING_DUTY     0x03u

#define FULL_REPLY_STATUS            0u
#define FULL_REPLY_STRIKE_STATUS     1u
#define FULL_REPLY_TIMING_STATUS     2u

static void send_strike_status_reply(void)
{
    uint32_t irq_state;
    int32_t drum_pos;
    int32_t home_pos;
    uint8_t strike_state;
    uint8_t strike_homed;
    strike_metrics_t metrics;
    uint8_t buf[35];
    uint16_t crc;

    irq_state = irq_save();
    drum_pos = strike_get_drum_position();
    home_pos = strike_get_home_position();
    strike_state = (uint8_t)strike_get_state();
    strike_homed = strike_is_homed();
    strike_get_metrics(&metrics);
    irq_restore(irq_state);

    buf[0]  = 32u;                                  /* LEN */
    buf[1]  = CMD_STATUS_BASE | device_addr;
    buf[2]  = strike_state;
    buf[3]  = strike_homed;
    buf[4]  = metrics.flags;
    buf[5]  = (uint8_t)(metrics.sequence >> 8);
    buf[6]  = (uint8_t)(metrics.sequence & 0xFFu);
    buf[7]  = (uint8_t)((uint16_t)metrics.last_current_ma >> 8);
    buf[8]  = (uint8_t)((uint16_t)metrics.last_current_ma & 0xFFu);
    buf[9]  = (uint8_t)(metrics.trigger_to_coast_ms >> 8);
    buf[10] = (uint8_t)(metrics.trigger_to_coast_ms & 0xFFu);
    buf[11] = (uint8_t)(metrics.trigger_to_rebound_ms >> 8);
    buf[12] = (uint8_t)(metrics.trigger_to_rebound_ms & 0xFFu);
    buf[13] = (uint8_t)(metrics.trigger_to_retrigger_ready_ms >> 8);
    buf[14] = (uint8_t)(metrics.trigger_to_retrigger_ready_ms & 0xFFu);
    buf[15] = (uint8_t)(metrics.trigger_to_ready_ms >> 8);
    buf[16] = (uint8_t)(metrics.trigger_to_ready_ms & 0xFFu);
    buf[17] = (uint8_t)(metrics.estimated_strike_velocity_dps >> 8);
    buf[18] = (uint8_t)(metrics.estimated_strike_velocity_dps & 0xFFu);
    buf[19] = (uint8_t)((uint32_t)drum_pos >> 24);
    buf[20] = (uint8_t)((uint32_t)drum_pos >> 16);
    buf[21] = (uint8_t)((uint32_t)drum_pos >> 8);
    buf[22] = (uint8_t)((uint32_t)drum_pos & 0xFFu);
    buf[23] = (uint8_t)((uint32_t)home_pos >> 24);
    buf[24] = (uint8_t)((uint32_t)home_pos >> 16);
    buf[25] = (uint8_t)((uint32_t)home_pos >> 8);
    buf[26] = (uint8_t)((uint32_t)home_pos & 0xFFu);
    buf[27] = (uint8_t)((uint16_t)metrics.home_offset >> 8);
    buf[28] = (uint8_t)((uint16_t)metrics.home_offset & 0xFFu);
    buf[29] = (uint8_t)((uint16_t)metrics.coast_distance >> 8);
    buf[30] = (uint8_t)((uint16_t)metrics.coast_distance & 0xFFu);
    buf[31] = (uint8_t)((uint16_t)metrics.homing_duty >> 8);
    buf[32] = (uint8_t)((uint16_t)metrics.homing_duty & 0xFFu);

    crc = crc16_ccitt(buf, 33);
    buf[33] = (uint8_t)(crc >> 8);
    buf[34] = (uint8_t)(crc & 0xFFu);

    send_frame(buf, 35);
}

static void send_timing_status_reply(void)
{
    timing_snapshot_t snapshot;
    uint8_t buf[60];
    uint16_t crc;

    timing_get_snapshot(&snapshot);

    buf[0]  = 57u;                                  /* LEN */
    buf[1]  = CMD_STATUS_BASE | device_addr;
    buf[2]  = (uint8_t)(snapshot.control_budget_us >> 8);
    buf[3]  = (uint8_t)(snapshot.control_budget_us & 0xFFu);
    buf[4]  = (uint8_t)(snapshot.control_last_us >> 8);
    buf[5]  = (uint8_t)(snapshot.control_last_us & 0xFFu);
    buf[6]  = (uint8_t)(snapshot.control_max_us >> 8);
    buf[7]  = (uint8_t)(snapshot.control_max_us & 0xFFu);
    buf[8]  = (uint8_t)(snapshot.control_overrun_count >> 24);
    buf[9]  = (uint8_t)(snapshot.control_overrun_count >> 16);
    buf[10] = (uint8_t)(snapshot.control_overrun_count >> 8);
    buf[11] = (uint8_t)(snapshot.control_overrun_count & 0xFFu);
    buf[12] = (uint8_t)(snapshot.velocity_drop_count >> 24);
    buf[13] = (uint8_t)(snapshot.velocity_drop_count >> 16);
    buf[14] = (uint8_t)(snapshot.velocity_drop_count >> 8);
    buf[15] = (uint8_t)(snapshot.velocity_drop_count & 0xFFu);
    buf[16] = (uint8_t)(snapshot.position_drop_count >> 24);
    buf[17] = (uint8_t)(snapshot.position_drop_count >> 16);
    buf[18] = (uint8_t)(snapshot.position_drop_count >> 8);
    buf[19] = (uint8_t)(snapshot.position_drop_count & 0xFFu);
    buf[20] = (uint8_t)(snapshot.strike_drop_count >> 24);
    buf[21] = (uint8_t)(snapshot.strike_drop_count >> 16);
    buf[22] = (uint8_t)(snapshot.strike_drop_count >> 8);
    buf[23] = (uint8_t)(snapshot.strike_drop_count & 0xFFu);
    buf[24] = (uint8_t)(snapshot.protocol_drop_count >> 24);
    buf[25] = (uint8_t)(snapshot.protocol_drop_count >> 16);
    buf[26] = (uint8_t)(snapshot.protocol_drop_count >> 8);
    buf[27] = (uint8_t)(snapshot.protocol_drop_count & 0xFFu);
    buf[28] = (uint8_t)(snapshot.hall_last_us >> 8);
    buf[29] = (uint8_t)(snapshot.hall_last_us & 0xFFu);
    buf[30] = (uint8_t)(snapshot.hall_max_us >> 8);
    buf[31] = (uint8_t)(snapshot.hall_max_us & 0xFFu);
    buf[32] = (uint8_t)(snapshot.uart_last_us >> 8);
    buf[33] = (uint8_t)(snapshot.uart_last_us & 0xFFu);
    buf[34] = (uint8_t)(snapshot.uart_max_us >> 8);
    buf[35] = (uint8_t)(snapshot.uart_max_us & 0xFFu);
    buf[36] = (uint8_t)(snapshot.adc_last_us >> 8);
    buf[37] = (uint8_t)(snapshot.adc_last_us & 0xFFu);
    buf[38] = (uint8_t)(snapshot.adc_max_us >> 8);
    buf[39] = (uint8_t)(snapshot.adc_max_us & 0xFFu);
    buf[40] = (uint8_t)(snapshot.protocol_poll_last_us >> 8);
    buf[41] = (uint8_t)(snapshot.protocol_poll_last_us & 0xFFu);
    buf[42] = (uint8_t)(snapshot.protocol_poll_max_us >> 8);
    buf[43] = (uint8_t)(snapshot.protocol_poll_max_us & 0xFFu);
    buf[44] = (uint8_t)(snapshot.protocol_backlog_max >> 8);
    buf[45] = (uint8_t)(snapshot.protocol_backlog_max & 0xFFu);
    buf[46] = (uint8_t)(snapshot.uptime_ms >> 24);
    buf[47] = (uint8_t)(snapshot.uptime_ms >> 16);
    buf[48] = (uint8_t)(snapshot.uptime_ms >> 8);
    buf[49] = (uint8_t)(snapshot.uptime_ms & 0xFFu);
    buf[50] = (uint8_t)(snapshot.uart_rx_overflow_count >> 24);
    buf[51] = (uint8_t)(snapshot.uart_rx_overflow_count >> 16);
    buf[52] = (uint8_t)(snapshot.uart_rx_overflow_count >> 8);
    buf[53] = (uint8_t)(snapshot.uart_rx_overflow_count & 0xFFu);
    buf[54] = (uint8_t)(snapshot.adc_overrun_count >> 24);
    buf[55] = (uint8_t)(snapshot.adc_overrun_count >> 16);
    buf[56] = (uint8_t)(snapshot.adc_overrun_count >> 8);
    buf[57] = (uint8_t)(snapshot.adc_overrun_count & 0xFFu);

    crc = crc16_ccitt(buf, 58);
    buf[58] = (uint8_t)(crc >> 8);
    buf[59] = (uint8_t)(crc & 0xFFu);

    send_frame(buf, 60);
}

static void send_ack_reply(uint8_t subcmd, uint8_t result, uint16_t detail)
{
    uint8_t buf[8];
    uint16_t crc;

    buf[0] = 5u;
    buf[1] = CMD_ACK_BASE | device_addr;
    buf[2] = subcmd;
    buf[3] = result;
    buf[4] = (uint8_t)(detail >> 8);
    buf[5] = (uint8_t)(detail & 0xFFu);

    crc = crc16_ccitt(buf, 6);
    buf[6] = (uint8_t)(crc >> 8);
    buf[7] = (uint8_t)(crc & 0xFFu);

    send_frame(buf, 8);
}

static uint8_t ack_result_from_strike_trigger(strike_trigger_result_t result)
{
    switch (result) {
    case STRIKE_TRIGGER_ACCEPTED:
        return ACK_RESULT_OK;
    case STRIKE_TRIGGER_RETRIGGERED:
        return ACK_RESULT_OK_RETRIGGERED;
    case STRIKE_TRIGGER_REJECT_NOT_HOMED:
        return ACK_RESULT_REJECT_NOT_HOMED;
    case STRIKE_TRIGGER_REJECT_FAULT:
        return ACK_RESULT_REJECT_FAULT;
    case STRIKE_TRIGGER_REJECT_ZERO:
        return ACK_RESULT_REJECT_ZERO;
    case STRIKE_TRIGGER_REJECT_NOT_READY:
        return ACK_RESULT_REJECT_NOT_READY;
    default:
        return ACK_RESULT_INVALID_ARGUMENT;
    }
}

static uint8_t ack_result_from_persist_status(int32_t status)
{
    return (status == 0) ? ACK_RESULT_OK : ACK_RESULT_PERSIST_FAILED;
}

static uint8_t sanitize_reply_mode(uint8_t reply_mode)
{
    if (reply_mode == SUBCMD_REPLY_ACK || reply_mode == SUBCMD_REPLY_NONE)
        return reply_mode;

    return SUBCMD_REPLY_FULL;
}

static void send_addressed_reply(
    uint8_t reply_mode,
    uint8_t subcmd,
    uint8_t ack_result,
    uint16_t ack_detail,
    uint8_t full_reply_kind
)
{
    if (reply_mode == SUBCMD_REPLY_NONE)
        return;

    if (reply_mode == SUBCMD_REPLY_ACK) {
        send_ack_reply(subcmd, ack_result, ack_detail);
        return;
    }

    if (full_reply_kind == FULL_REPLY_STRIKE_STATUS)
        send_strike_status_reply();
    else if (full_reply_kind == FULL_REPLY_TIMING_STATUS)
        send_timing_status_reply();
    else
        send_status_reply();
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
    uint8_t raw_subcmd;
    uint8_t subcmd;
    uint8_t reply_mode;
    uint8_t ack_result;
    uint16_t ack_detail;
    uint8_t start_after;

    if (device_addr == ADDR_UNASSIGNED || target != device_addr) {
        if (fwd_mode == FWD_STORE_AND_FORWARD)
            forward_frame_verbatim();
        return;
    }

    raw_subcmd = (len >= 2u) ? payload[1] : 0u;
    subcmd = raw_subcmd & SUBCMD_MASK;
    reply_mode = sanitize_reply_mode(raw_subcmd & SUBCMD_REPLY_MASK);

    if (subcmd == SUBCMD_QUERY_STATUS || subcmd == SUBCMD_QUERY_STRIKE || subcmd == SUBCMD_QUERY_TIMING)
        reply_mode = SUBCMD_REPLY_FULL;

    switch (subcmd) {
    case SUBCMD_SET_DUTY:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 4u) {
            int16_t duty = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
            if (duty != 0 && motor_get_state() == MOTOR_FAULT) {
                ack_result = ACK_RESULT_REJECT_FAULT;
            } else if (prepare_set_duty(duty)) {
                motor_start();
                ack_result = ACK_RESULT_OK;
            } else if (duty == 0) {
                /* prepare_set_duty already stopped the motor */
                ack_result = ACK_RESULT_OK;
            }
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_TORQUE:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 4u) {
            uint16_t ma = ((uint16_t)payload[2] << 8) | payload[3];
            motor_set_torque_limit(ma);
            ack_result = ACK_RESULT_OK;
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_STOP:
        motor_stop();
        send_addressed_reply(reply_mode, subcmd, ACK_RESULT_OK, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_CLEAR_FAULT:
        motor_clear_fault();
        send_addressed_reply(reply_mode, subcmd, ACK_RESULT_OK, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_MODE:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 3u && payload[2] <= CTRL_TORQUE) {
            motor_set_mode((ctrl_mode_t)payload[2]);
            ack_result = ACK_RESULT_OK;
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_VELOCITY:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 4u) {
            int16_t rpm = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
            if (motor_get_state() == MOTOR_FAULT) {
                ack_result = ACK_RESULT_REJECT_FAULT;
            } else {
                start_after = prepare_set_velocity(rpm);
                ack_result = ACK_RESULT_OK;
                if (start_after) motor_start();
            }
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_PID:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 8u) {
            int16_t kp = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
            int16_t ki = (int16_t)(((uint16_t)payload[4] << 8) | payload[5]);
            int16_t kd = (int16_t)(((uint16_t)payload[6] << 8) | payload[7]);
            motor_set_vel_pid(kp, ki, kd);
            ack_result = ACK_RESULT_OK;
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_FF:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 4u) {
            int16_t gain = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
            motor_set_vel_ff(gain);
            ack_result = ACK_RESULT_OK;
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_POSITION:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 6u) {
            int32_t pos = (int32_t)(
                ((uint32_t)payload[2] << 24) | ((uint32_t)payload[3] << 16) |
                ((uint32_t)payload[4] << 8)  |  (uint32_t)payload[5]);
            if (motor_get_state() == MOTOR_FAULT) {
                ack_result = ACK_RESULT_REJECT_FAULT;
            } else {
                start_after = prepare_set_position(pos);
                ack_result = ACK_RESULT_OK;
                if (start_after) motor_start();
            }
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_POS_PID:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 8u) {
            int16_t kp = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
            int16_t ki = (int16_t)(((uint16_t)payload[4] << 8) | payload[5]);
            int16_t kd = (int16_t)(((uint16_t)payload[6] << 8) | payload[7]);
            motor_set_pos_pid(kp, ki, kd);
            ack_result = ACK_RESULT_OK;
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_ZERO_POSITION:
    {
        uint32_t irq_state;
        int32_t current_pos;
        int32_t persist_status;

        /* Flash erase disables IRQs for ~20 ms — never do it under active motion. */
        if (motor_get_state() == MOTOR_RUN) {
            send_addressed_reply(reply_mode, subcmd, ACK_RESULT_REJECT_NOT_READY,
                                 0u, FULL_REPLY_STATUS);
            break;
        }

        irq_state = irq_save();
        current_pos = -encoder_get_position();
        motor_shift_position_reference(current_pos);
        strike_shift_position_reference(current_pos);
        encoder_reset_position();
        irq_restore(irq_state);
        /* Persist the logical zero point so absolute angle reconstructs it. */
        persist_status = persist_save_runtime();
        send_addressed_reply(reply_mode, subcmd,
                             ack_result_from_persist_status(persist_status),
                             0u, FULL_REPLY_STATUS);
        break;
    }
    case SUBCMD_STRIKE:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        ack_detail = strike_get_sequence();
        if (len >= 4u) {
            int16_t current_ma = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
            ack_result = ack_result_from_strike_trigger(strike_trigger((int32_t)current_ma));
            ack_detail = strike_get_sequence();
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, ack_detail, FULL_REPLY_STATUS);
        break;
    case SUBCMD_STRIKE_HOME:
        strike_home();
        send_addressed_reply(reply_mode, subcmd, ACK_RESULT_OK, strike_get_sequence(), FULL_REPLY_STATUS);
        break;
    case SUBCMD_STRIKE_CANCEL:
        strike_cancel();
        send_addressed_reply(reply_mode, subcmd, ACK_RESULT_OK, strike_get_sequence(), FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_STRIKE_PARAM:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 5u) {
            uint8_t param_id = payload[2];
            int16_t value = (int16_t)(((uint16_t)payload[3] << 8) | payload[4]);
            switch (param_id) {
            case STRIKE_PARAM_HOME_OFFSET:
                strike_set_home_offset((int32_t)value);
                ack_result = ACK_RESULT_OK;
                break;
            case STRIKE_PARAM_COAST_DISTANCE:
                strike_set_coast_distance((int32_t)value);
                ack_result = ACK_RESULT_OK;
                break;
            case STRIKE_PARAM_HOMING_DUTY:
                strike_set_homing_duty((int32_t)value);
                ack_result = ACK_RESULT_OK;
                break;
            default:
                ack_result = ACK_RESULT_INVALID_ARGUMENT;
                break;
            }
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, strike_get_sequence(), FULL_REPLY_STATUS);
        break;
    case SUBCMD_QUERY_STATUS:
        send_addressed_reply(reply_mode, subcmd, ACK_RESULT_OK, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_QUERY_STRIKE:
        send_addressed_reply(reply_mode, subcmd, ACK_RESULT_OK, strike_get_sequence(), FULL_REPLY_STRIKE_STATUS);
        break;
    case SUBCMD_QUERY_TIMING:
        send_addressed_reply(reply_mode, subcmd, ACK_RESULT_OK, 0u, FULL_REPLY_TIMING_STATUS);
        break;
    case SUBCMD_SAVE_SETTINGS:
        /* Flash erase disables IRQs for ~20 ms — never do it under active motion. */
        if (motor_get_state() == MOTOR_RUN)
            ack_result = ACK_RESULT_REJECT_NOT_READY;
        else
            ack_result = ack_result_from_persist_status(persist_save_runtime());
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_CLEAR_SETTINGS:
        if (motor_get_state() == MOTOR_RUN)
            ack_result = ACK_RESULT_REJECT_NOT_READY;
        else
            ack_result = ack_result_from_persist_status(persist_clear());
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_CUR_PID:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 6u) {
            int16_t kp = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
            int16_t ki = (int16_t)(((uint16_t)payload[4] << 8) | payload[5]);
            motor_set_cur_pid(kp, ki);
            ack_result = ACK_RESULT_OK;
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_CURRENT:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        if (len >= 4u) {
            int16_t ma = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
            if (motor_get_state() == MOTOR_FAULT) {
                ack_result = ACK_RESULT_REJECT_FAULT;
            } else {
                motor_set_current(ma);
                motor_set_mode(CTRL_TORQUE);
                motor_start();
                ack_result = ACK_RESULT_OK;
            }
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, 0u, FULL_REPLY_STATUS);
        break;
    case SUBCMD_DETECT_CSN_POLARITY:
        /* Probes both polarities and CRCs the result.  Hammers CSn during
         * the probe, so refuse while motion is happening — the wrong-
         * polarity probe would otherwise corrupt mid-loop encoder reads. */
        if (motor_get_state() == MOTOR_RUN) {
            ack_result = ACK_RESULT_REJECT_NOT_READY;
            ack_detail = 0u;
        } else if (encoder_autodetect_csn_polarity()) {
            ack_result = ACK_RESULT_OK;
            ack_detail = encoder_get_csn_polarity();
            (void)persist_save_runtime();
        } else {
            ack_result = ACK_RESULT_INVALID_ARGUMENT;
            ack_detail = 0u;
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, ack_detail, FULL_REPLY_STATUS);
        break;
    case SUBCMD_SET_CSN_POLARITY:
        ack_result = ACK_RESULT_INVALID_ARGUMENT;
        ack_detail = 0u;
        if (len >= 3u && (payload[2] == 0u || payload[2] == 1u)) {
            if (motor_get_state() == MOTOR_RUN) {
                ack_result = ACK_RESULT_REJECT_NOT_READY;
            } else {
                encoder_set_csn_polarity(payload[2]);
                ack_detail = encoder_get_csn_polarity();
                ack_result = ack_result_from_persist_status(persist_save_runtime());
            }
        }
        send_addressed_reply(reply_mode, subcmd, ack_result, ack_detail, FULL_REPLY_STATUS);
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
        } else if ((cmd >= CMD_STATUS_BASE && cmd < CMD_STATUS_END) ||
                   (cmd >= CMD_ACK_BASE && cmd < CMD_ACK_END)) {
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
                rx_timeout = FRAME_TIMEOUT_TICKS;
                DBG('B');
            } else if (c == PREAMBLE_0) {
                /* could be start of a new preamble, stay in SCAN_1 */
                rx_timeout = FRAME_TIMEOUT_TICKS;
            } else {
                rx_phase = RX_SCAN_0;
                rx_timeout = 0;
            }
            break;

        case RX_BUFFERING:
            if (frame_pos < FRAME_BUF_SIZE)
                frame_buf[frame_pos] = c;
            frame_pos++;
            rx_timeout = FRAME_TIMEOUT_TICKS;

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

void protocol_tick(uint32_t elapsed_ticks)
{
    if (rx_phase != RX_SCAN_0 && rx_timeout != 0u) {
        if (elapsed_ticks >= rx_timeout)
            reset_receiver();
        else
            rx_timeout = (uint8_t)(rx_timeout - elapsed_ticks);
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

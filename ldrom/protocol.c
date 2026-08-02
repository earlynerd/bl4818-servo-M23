#include <stdint.h>
#include "boot_mailbox.h"
#include "firmware_image.h"
#include "bl_internal.h"

#define PREAMBLE_0 0xA5u
#define PREAMBLE_1 0x5Au
#define MAX_PAYLOAD 64u
#define FRAME_BODY_SIZE (1u + MAX_PAYLOAD + 2u)

#define CMD_ENTER_SF 0x01u
#define CMD_ENTER_CT 0x02u
#define CMD_SET_ADDRESS 0x03u
#define CMD_STAY_BOOT 0x61u
#define CMD_BOOT_BASE 0x70u
#define CMD_BOOT_END 0x80u
#define CMD_BOOT_REPLY_BASE 0x80u

#define BOOT_PROTOCOL_VERSION 2u
#define BOOT_LOADER_VERSION 2u

#define BOOT_SUB_GET_INFO 0x01u
#define BOOT_SUB_BEGIN_IMAGE 0x10u
#define BOOT_SUB_ERASE_PAGE 0x11u
#define BOOT_SUB_WRITE_CHUNK 0x12u
#define BOOT_SUB_VERIFY_IMAGE 0x13u
#define BOOT_SUB_COMMIT_IMAGE 0x14u
#define BOOT_SUB_RUN_APROM 0x15u

#define BOOT_RESULT_OK 0x00u
#define BOOT_RESULT_BAD_COMMAND 0x01u
#define BOOT_RESULT_BAD_STATE 0x02u
#define BOOT_RESULT_BAD_RANGE 0x03u
#define BOOT_RESULT_FLASH_FAILED 0x04u
#define BOOT_RESULT_CRC_MISMATCH 0x05u

#define BOOT_FLAG_COMMITTED_VALID 0x01u
#define BOOT_FLAG_UPDATE_ACTIVE 0x02u

typedef enum
{
    FORWARD_CUT_THROUGH = 0,
    FORWARD_STORE
} forward_mode_t;

typedef enum
{
    RX_SCAN_0 = 0,
    RX_SCAN_1,
    RX_BODY
} rx_state_t;

typedef struct
{
    uint8_t active;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t image_version;
    uint32_t flags;
} update_state_t;

static uint8_t device_address;
static uint8_t hold_requested;
static uint8_t run_requested;
static uint8_t committed_valid;
static forward_mode_t forward_mode;
static forward_mode_t frame_start_mode;
static rx_state_t rx_state;
static uint8_t frame_body[FRAME_BODY_SIZE];
static uint8_t frame_position;
static uint8_t frame_expected;
static update_state_t update_state;
static const bl_boot_diagnostics_t *boot_diagnostics;

static uint16_t crc16_update(uint16_t crc, uint8_t value)
{
    uint8_t bit;

    crc ^= (uint16_t)value << 8;
    for (bit = 0u; bit < 8u; bit++) {
        if ((crc & 0x8000u) != 0u)
            crc = (uint16_t)((crc << 1) ^ 0x1021u);
        else
            crc <<= 1;
    }
    return crc;
}

static uint16_t crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFu;

    while (length-- != 0u)
        crc = crc16_update(crc, *data++);
    return crc;
}

static uint16_t read_be16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t read_be32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | data[3];
}

static void append_be16(uint8_t *data, uint8_t *position, uint16_t value)
{
    data[(*position)++] = (uint8_t)(value >> 8);
    data[(*position)++] = (uint8_t)value;
}

static void append_be32(uint8_t *data, uint8_t *position, uint32_t value)
{
    data[(*position)++] = (uint8_t)(value >> 24);
    data[(*position)++] = (uint8_t)(value >> 16);
    data[(*position)++] = (uint8_t)(value >> 8);
    data[(*position)++] = (uint8_t)value;
}

static void send_frame(const uint8_t *payload, uint8_t length)
{
    uint8_t header[3] = {PREAMBLE_0, PREAMBLE_1, length};
    uint16_t crc = crc16(&header[2], 1u);
    uint8_t i;

    bl_uart_write(header, sizeof(header));
    for (i = 0u; i < length; i++) {
        bl_uart_putc(payload[i]);
        crc = crc16_update(crc, payload[i]);
    }
    bl_uart_putc((uint8_t)(crc >> 8));
    bl_uart_putc((uint8_t)crc);
}

static void forward_current_frame(void)
{
    static const uint8_t preamble[2] = {PREAMBLE_0, PREAMBLE_1};

    bl_uart_write(preamble, sizeof(preamble));
    bl_uart_write(frame_body, frame_expected);
}

static void send_reply(uint8_t subcommand, uint8_t result,
                       const uint8_t *detail, uint8_t detail_length)
{
    uint8_t payload[MAX_PAYLOAD];
    uint8_t i;

    if (device_address == BL_ADDRESS_UNASSIGNED)
        return;
    payload[0] = (uint8_t)(CMD_BOOT_REPLY_BASE + device_address);
    payload[1] = subcommand;
    payload[2] = result;
    for (i = 0u; i < detail_length; i++)
        payload[3u + i] = detail[i];
    send_frame(payload, (uint8_t)(3u + detail_length));
}

static void reply_get_info(void)
{
    const firmware_manifest_t *manifest =
        (const firmware_manifest_t *)FIRMWARE_MANIFEST_BASE;
    uint8_t detail[45];
    uint8_t position = 0u;
    uint8_t flags = 0u;
    uint8_t index;

    if (committed_valid != 0u)
        flags |= BOOT_FLAG_COMMITTED_VALID;
    if (update_state.active != 0u)
        flags |= BOOT_FLAG_UPDATE_ACTIVE;

    detail[position++] = BOOT_PROTOCOL_VERSION;
    append_be16(detail, &position, BOOT_LOADER_VERSION);
    detail[position++] = flags;
    detail[position++] = device_address;
    append_be16(detail, &position, FIRMWARE_FLASH_PAGE_SIZE);
    append_be16(detail, &position, FIRMWARE_APP_LIMIT);
    append_be32(detail, &position, manifest->image_size);
    append_be32(detail, &position, manifest->image_crc32);
    append_be32(detail, &position, manifest->image_version);
    for (index = 0u; index < 3u; index++)
        append_be32(detail, &position, bl_flash_read_uid(index));
    append_be32(detail, &position, boot_diagnostics->reset_status);
    append_be32(detail, &position, boot_diagnostics->isp_control);
    append_be32(detail, &position, boot_diagnostics->isp_status);
    send_reply(BOOT_SUB_GET_INFO, BOOT_RESULT_OK, detail, position);
}

static void handle_begin_image(const uint8_t *payload, uint8_t length)
{
    uint32_t image_size;
    uint32_t flags;

    if (length != 18u) {
        send_reply(BOOT_SUB_BEGIN_IMAGE, BOOT_RESULT_BAD_COMMAND, 0, 0u);
        return;
    }

    image_size = read_be32(&payload[2]);
    flags = read_be32(&payload[14]);
    if (image_size < 8u || image_size > FIRMWARE_APP_LIMIT ||
        (image_size & 3u) != 0u || flags != 0u) {
        send_reply(BOOT_SUB_BEGIN_IMAGE, BOOT_RESULT_BAD_RANGE, 0, 0u);
        return;
    }

    update_state.active = 0u;
    committed_valid = 0u;
    if (bl_flash_erase_manifest() != 0) {
        send_reply(BOOT_SUB_BEGIN_IMAGE, BOOT_RESULT_FLASH_FAILED, 0, 0u);
        return;
    }

    update_state.image_size = image_size;
    update_state.image_crc32 = read_be32(&payload[6]);
    update_state.image_version = read_be32(&payload[10]);
    update_state.flags = flags;
    update_state.active = 1u;
    send_reply(BOOT_SUB_BEGIN_IMAGE, BOOT_RESULT_OK, 0, 0u);
}

static void handle_erase_page(const uint8_t *payload, uint8_t length)
{
    uint8_t page;
    uint32_t page_address;

    if (length != 3u) {
        send_reply(BOOT_SUB_ERASE_PAGE, BOOT_RESULT_BAD_COMMAND, 0, 0u);
        return;
    }
    if (update_state.active == 0u) {
        send_reply(BOOT_SUB_ERASE_PAGE, BOOT_RESULT_BAD_STATE, 0, 0u);
        return;
    }

    page = payload[2];
    page_address = (uint32_t)page * FIRMWARE_FLASH_PAGE_SIZE;
    if (page_address >= update_state.image_size ||
        page_address >= FIRMWARE_APP_LIMIT) {
        send_reply(BOOT_SUB_ERASE_PAGE, BOOT_RESULT_BAD_RANGE, 0, 0u);
        return;
    }

    if (bl_flash_erase_app_page(page) != 0)
        send_reply(BOOT_SUB_ERASE_PAGE, BOOT_RESULT_FLASH_FAILED, &page, 1u);
    else
        send_reply(BOOT_SUB_ERASE_PAGE, BOOT_RESULT_OK, &page, 1u);
}

static void handle_write_chunk(const uint8_t *payload, uint8_t length)
{
    uint16_t offset;
    uint8_t data_length;
    uint8_t position;

    if (length < 8u) {
        send_reply(BOOT_SUB_WRITE_CHUNK, BOOT_RESULT_BAD_COMMAND, 0, 0u);
        return;
    }
    if (update_state.active == 0u) {
        send_reply(BOOT_SUB_WRITE_CHUNK, BOOT_RESULT_BAD_STATE, 0, 0u);
        return;
    }

    offset = read_be16(&payload[2]);
    data_length = (uint8_t)(length - 4u);
    if ((offset & 3u) != 0u || data_length > 32u ||
        (data_length & 3u) != 0u ||
        (uint32_t)offset + data_length > update_state.image_size) {
        send_reply(BOOT_SUB_WRITE_CHUNK, BOOT_RESULT_BAD_RANGE, 0, 0u);
        return;
    }

    for (position = 0u; position < data_length; position += 4u) {
        uint32_t word = (uint32_t)payload[4u + position] |
                        ((uint32_t)payload[5u + position] << 8) |
                        ((uint32_t)payload[6u + position] << 16) |
                        ((uint32_t)payload[7u + position] << 24);
        if (bl_flash_program_word((uint32_t)offset + position, word) != 0) {
            send_reply(BOOT_SUB_WRITE_CHUNK, BOOT_RESULT_FLASH_FAILED,
                       &payload[2], 2u);
            return;
        }
    }

    send_reply(BOOT_SUB_WRITE_CHUNK, BOOT_RESULT_OK, &payload[2], 2u);
}

static uint8_t verify_proposed_image(uint32_t *calculated_crc)
{
    if (update_state.active == 0u)
        return BOOT_RESULT_BAD_STATE;
    if (bl_flash_crc32_app(update_state.image_size, calculated_crc) != 0)
        return BOOT_RESULT_FLASH_FAILED;
    if (*calculated_crc != update_state.image_crc32)
        return BOOT_RESULT_CRC_MISMATCH;
    return BOOT_RESULT_OK;
}

static void handle_verify_image(void)
{
    uint8_t detail[4];
    uint8_t position = 0u;
    uint32_t calculated_crc = 0u;
    uint8_t result = verify_proposed_image(&calculated_crc);

    append_be32(detail, &position, calculated_crc);
    send_reply(BOOT_SUB_VERIFY_IMAGE, result, detail, position);
}

static int commit_manifest(void)
{
    firmware_manifest_t manifest;

    manifest.magic = FIRMWARE_MANIFEST_MAGIC;
    manifest.format_version = FIRMWARE_MANIFEST_VERSION;
    manifest.header_size = FIRMWARE_MANIFEST_SIZE;
    manifest.image_size = update_state.image_size;
    manifest.image_crc32 = update_state.image_crc32;
    manifest.image_version = update_state.image_version;
    manifest.flags = update_state.flags;
    manifest.reserved = 0u;
    manifest.manifest_crc32 = bl_crc32(&manifest,
        FIRMWARE_MANIFEST_SIZE - sizeof(uint32_t));

    return bl_flash_commit_manifest(&manifest);
}

static void handle_commit_image(void)
{
    uint32_t calculated_crc;
    uint8_t result = verify_proposed_image(&calculated_crc);

    if (result == BOOT_RESULT_OK && commit_manifest() != 0)
        result = BOOT_RESULT_FLASH_FAILED;
    if (result == BOOT_RESULT_OK) {
        update_state.active = 0u;
        committed_valid = 1u;
    }
    send_reply(BOOT_SUB_COMMIT_IMAGE, result, 0, 0u);
}

static void handle_boot_command(const uint8_t *payload, uint8_t length)
{
    uint8_t target = (uint8_t)(payload[0] - CMD_BOOT_BASE);
    uint8_t subcommand = (length >= 2u) ? payload[1] : 0u;

    if (device_address == BL_ADDRESS_UNASSIGNED || target != device_address) {
        if (forward_mode == FORWARD_STORE)
            forward_current_frame();
        return;
    }

    switch (subcommand) {
    case BOOT_SUB_GET_INFO:
        if (length == 2u)
            reply_get_info();
        else
            send_reply(subcommand, BOOT_RESULT_BAD_COMMAND, 0, 0u);
        break;
    case BOOT_SUB_BEGIN_IMAGE:
        handle_begin_image(payload, length);
        break;
    case BOOT_SUB_ERASE_PAGE:
        handle_erase_page(payload, length);
        break;
    case BOOT_SUB_WRITE_CHUNK:
        handle_write_chunk(payload, length);
        break;
    case BOOT_SUB_VERIFY_IMAGE:
        if (length == 2u)
            handle_verify_image();
        else
            send_reply(subcommand, BOOT_RESULT_BAD_COMMAND, 0, 0u);
        break;
    case BOOT_SUB_COMMIT_IMAGE:
        if (length == 2u)
            handle_commit_image();
        else
            send_reply(subcommand, BOOT_RESULT_BAD_COMMAND, 0, 0u);
        break;
    case BOOT_SUB_RUN_APROM:
        if (length == 2u && committed_valid != 0u) {
            send_reply(subcommand, BOOT_RESULT_OK, 0, 0u);
            run_requested = 1u;
        } else {
            send_reply(subcommand,
                       (length == 2u) ? BOOT_RESULT_BAD_STATE :
                                        BOOT_RESULT_BAD_COMMAND,
                       0, 0u);
        }
        break;
    default:
        send_reply(subcommand, BOOT_RESULT_BAD_COMMAND, 0, 0u);
        break;
    }
}

static void dispatch(const uint8_t *payload, uint8_t length)
{
    uint8_t command = payload[0];

    if (command == CMD_ENTER_SF) {
        forward_mode_t start = frame_start_mode;
        forward_mode = FORWARD_STORE;
        bl_uart_echo_disable();
        if (start == FORWARD_STORE)
            forward_current_frame();
    } else if (command == CMD_ENTER_CT) {
        forward_mode_t start = frame_start_mode;
        forward_mode = FORWARD_CUT_THROUGH;
        bl_uart_echo_enable();
        if (start == FORWARD_STORE)
            forward_current_frame();
    } else if (command == CMD_SET_ADDRESS && length == 2u) {
        uint8_t counter = payload[1];
        uint8_t next_payload[2] = {CMD_SET_ADDRESS, (uint8_t)(counter + 1u)};
        if (counter < 16u) {
            if (device_address == BL_ADDRESS_UNASSIGNED)
                device_address = counter;
            send_frame(next_payload, sizeof(next_payload));
        }
    } else if (command == CMD_STAY_BOOT) {
        hold_requested = 1u;
        if (frame_start_mode == FORWARD_STORE)
            forward_current_frame();
    } else if (command >= CMD_BOOT_BASE && command < CMD_BOOT_END) {
        handle_boot_command(payload, length);
    } else if (forward_mode == FORWARD_STORE) {
        forward_current_frame();
    }
}

static void reset_receiver(void)
{
    rx_state = RX_SCAN_0;
    frame_position = 0u;
    frame_expected = 0u;
}

static void consume_byte(uint8_t value)
{
    if (rx_state == RX_SCAN_0) {
        if (value == PREAMBLE_0)
            rx_state = RX_SCAN_1;
        return;
    }
    if (rx_state == RX_SCAN_1) {
        if (value == PREAMBLE_1) {
            rx_state = RX_BODY;
            frame_position = 0u;
            frame_expected = 0u;
            frame_start_mode = forward_mode;
        } else if (value != PREAMBLE_0) {
            rx_state = RX_SCAN_0;
        }
        return;
    }

    frame_body[frame_position++] = value;
    if (frame_position == 1u) {
        if (value == 0u || value > MAX_PAYLOAD) {
            reset_receiver();
            return;
        }
        frame_expected = (uint8_t)(1u + value + 2u);
    }
    if (frame_expected != 0u && frame_position == frame_expected) {
        uint8_t length = frame_body[0];
        uint16_t received_crc = read_be16(&frame_body[1u + length]);
        if (crc16(frame_body, (uint8_t)(1u + length)) == received_crc)
            dispatch(&frame_body[1], length);
        reset_receiver();
    }
}

void bl_protocol_init(uint8_t address, uint8_t image_valid,
                      uint8_t was_requested,
                      const bl_boot_diagnostics_t *diagnostics)
{
    device_address = (address < 16u) ? address : BL_ADDRESS_UNASSIGNED;
    committed_valid = image_valid;
    hold_requested = was_requested;
    run_requested = 0u;
    boot_diagnostics = diagnostics;
    forward_mode = FORWARD_CUT_THROUGH;
    update_state.active = 0u;
    bl_uart_echo_enable();
    reset_receiver();
}

void bl_protocol_poll(void)
{
    if (bl_uart_overflowed() != 0u) {
        bl_uart_clear_overflow();
        reset_receiver();
        return;
    }
    while (bl_uart_available() != 0u)
        consume_byte(bl_uart_getc());
}

uint8_t bl_protocol_hold_requested(void)
{
    return hold_requested;
}

uint8_t bl_protocol_run_requested(void)
{
    uint8_t requested = run_requested;

    run_requested = 0u;
    return requested;
}

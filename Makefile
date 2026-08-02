# Makefile for Nuvoton M2003 (Cortex-M23) Motor Firmware

# ── Toolchain ────────────────────────────────────────────────────────────────
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size
PYTHON  = py

# ── Project ──────────────────────────────────────────────────────────────────
PROJECT = m2003-motor
LDROM_PROJECT = m2003-ldrom
BUILD_DIR = build
FIRMWARE_IMAGE_VERSION ?= 0

# CPU flags for Cortex-M23
CPU_FLAGS = -mcpu=cortex-m23 -mthumb -mfloat-abi=soft

# Compilation flags
CFLAGS = $(CPU_FLAGS) -O2 -Wall -g -ffunction-sections -fdata-sections
CFLAGS += -Iinclude -ICMSIS -ILibrary/StdDriver/inc
CFLAGS += -MMD -MP

# Linker flags
LDFLAGS = $(CPU_FLAGS) -Wl,--gc-sections -T m2003.ld

LDROM_CFLAGS = $(CPU_FLAGS) -Os -flto -Wall -Wextra -g -ffreestanding
LDROM_CFLAGS += -ffunction-sections -fdata-sections -fno-unwind-tables
LDROM_CFLAGS += -fno-asynchronous-unwind-tables -fno-delete-null-pointer-checks
LDROM_CFLAGS += -fno-jump-tables
LDROM_CFLAGS += -Iinclude -ICMSIS -ILibrary/StdDriver/inc
LDROM_CFLAGS += -MMD -MP
LDROM_LDFLAGS = $(CPU_FLAGS) -flto -nostdlib -Wl,--gc-sections
LDROM_LDFLAGS += -Wl,-Map,$(BUILD_DIR)/$(LDROM_PROJECT).map -T ldrom/m2003_ldrom.ld

# ── Files ────────────────────────────────────────────────────────────────────
SRCS = src/main.c \
       src/app_pwm.c \
       src/app_uart.c \
       src/app_adc.c \
       src/motor.c \
       src/protocol.c \
       src/pid.c \
       src/crc16.c \
       src/commutation.c \
       src/hall.c \
       src/encoder.c \
       src/indicator.c \
       src/timing.c \
       src/persist.c \
       src/strike.c \
       src/boot_control.c \
       src/startup_m2003.c \
       src/system_M2003.c

BSP_SRCS = $(wildcard Library/StdDriver/src/*.c)

OBJS = $(addprefix $(BUILD_DIR)/, $(SRCS:.c=.o)) $(addprefix $(BUILD_DIR)/, $(BSP_SRCS:.c=.o))
DEPS = $(OBJS:.o=.d)

LDROM_SRCS = ldrom/startup.c \
             ldrom/platform.c \
             ldrom/main.c \
             ldrom/uart.c \
             ldrom/flash.c \
             ldrom/crc32.c \
             ldrom/protocol.c
LDROM_OBJS = $(addprefix $(BUILD_DIR)/, $(LDROM_SRCS:.c=.o))
LDROM_DEPS = $(LDROM_OBJS:.o=.d)

# ── Targets ──────────────────────────────────────────────────────────────────
all: $(BUILD_DIR)/$(PROJECT).bin size

images: all ldrom package

ldrom: $(BUILD_DIR)/$(LDROM_PROJECT).bin ldrom-size

package: $(BUILD_DIR)/$(PROJECT).bin
	$(PYTHON) scripts/firmware_image.py $< \
		--image-version $(FIRMWARE_IMAGE_VERSION) \
		--image-out $(BUILD_DIR)/$(PROJECT).update.bin \
		--manifest-out $(BUILD_DIR)/$(PROJECT).manifest.bin

build-jlink: images
	$(PYTHON) scripts/make_provision_jlink.py \
		--app $(BUILD_DIR)/$(PROJECT).update.bin \
		--manifest $(BUILD_DIR)/$(PROJECT).manifest.bin \
		--ldrom $(BUILD_DIR)/$(LDROM_PROJECT).bin \
		--output $(BUILD_DIR)/$(PROJECT).jlink \
		--verify-output $(BUILD_DIR)/m2003-firmware-verify.jlink \
		--config-read-output $(BUILD_DIR)/m2003-config-read.jlink

$(BUILD_DIR)/%.o: %.c
	@if not exist $(subst /,\,$(dir $@)) mkdir $(subst /,\,$(dir $@))
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/ldrom/%.o: ldrom/%.c
	@if not exist $(subst /,\,$(dir $@)) mkdir $(subst /,\,$(dir $@))
	$(CC) $(LDROM_CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(PROJECT).elf: $(OBJS) m2003.ld
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(LDROM_PROJECT).elf: $(LDROM_OBJS) ldrom/m2003_ldrom.ld
	$(CC) $(LDROM_OBJS) $(LDROM_LDFLAGS) -o $@

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(LDROM_PROJECT).bin: $(BUILD_DIR)/$(LDROM_PROJECT).elf
	$(OBJCOPY) -O binary $< $@

size: $(BUILD_DIR)/$(PROJECT).elf
	$(SIZE) $<

ldrom-size: $(BUILD_DIR)/$(LDROM_PROJECT).elf
	$(SIZE) $<

clean:
	powershell -NoProfile -Command "if (Test-Path '$(BUILD_DIR)') { Remove-Item -LiteralPath '$(BUILD_DIR)' -Recurse -Force }"

.PHONY: all images ldrom package clean size ldrom-size build-jlink

-include $(DEPS) $(LDROM_DEPS)

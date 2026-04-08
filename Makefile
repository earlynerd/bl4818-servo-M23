# Makefile for Nuvoton M2003 (Cortex-M23) Motor Firmware

# ── Toolchain ────────────────────────────────────────────────────────────────
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size
PYTHON  = py

# ── Project ──────────────────────────────────────────────────────────────────
PROJECT = m2003-motor
BUILD_DIR = build

# CPU flags for Cortex-M23
CPU_FLAGS = -mcpu=cortex-m23 -mthumb -mfloat-abi=soft

# Compilation flags
CFLAGS = $(CPU_FLAGS) -O2 -Wall -g -ffunction-sections -fdata-sections
CFLAGS += -Iinclude -ICMSIS -ILibrary/StdDriver/inc
CFLAGS += -MMD -MP

# Linker flags
LDFLAGS = $(CPU_FLAGS) -Wl,--gc-sections -T m2003.ld

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
       src/persist.c \
       src/strike.c \
       src/startup_m2003.c \
       src/system_M2003.c

BSP_SRCS = $(wildcard Library/StdDriver/src/*.c)

OBJS = $(addprefix $(BUILD_DIR)/, $(SRCS:.c=.o)) $(addprefix $(BUILD_DIR)/, $(BSP_SRCS:.c=.o))
DEPS = $(OBJS:.o=.d)

# ── Targets ──────────────────────────────────────────────────────────────────
all: $(BUILD_DIR)/$(PROJECT).bin size

jlink-script: $(BUILD_DIR)/$(PROJECT).bin
	$(PYTHON) jlink_flash_m2003.py $(BUILD_DIR)\$(PROJECT).bin $(BUILD_DIR)\$(PROJECT).jlink

build-jlink: all jlink-script

$(BUILD_DIR)/%.o: %.c
	@if not exist $(subst /,\,$(dir $@)) mkdir $(subst /,\,$(dir $@))
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(PROJECT).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

size: $(BUILD_DIR)/$(PROJECT).elf
	$(SIZE) $<

clean:
	powershell -NoProfile -Command "if (Test-Path '$(BUILD_DIR)') { Remove-Item -LiteralPath '$(BUILD_DIR)' -Recurse -Force }"

.PHONY: all clean size jlink-script build-jlink

-include $(DEPS)

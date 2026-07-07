# Bootloader build. Self-contained — invoke as:
#   make -C boot
# from the project root. Produces boot/build/boot.elf/.bin/.hex/.map/.list.
#
# Uses the same toolchain paths as the CubeIDE app build (Debug/makefile);
# override TOOLCHAIN_BIN on the command line if your install layout differs.

TARGET      := boot
BUILD_DIR   := build

TOOLCHAIN_BIN ?= C:/ST/STM32CubeIDE_1.12.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712/tools/bin
CC          := $(TOOLCHAIN_BIN)/arm-none-eabi-gcc
OBJCOPY     := $(TOOLCHAIN_BIN)/arm-none-eabi-objcopy
OBJDUMP     := $(TOOLCHAIN_BIN)/arm-none-eabi-objdump
SIZE        := $(TOOLCHAIN_BIN)/arm-none-eabi-size

# Project root — one level up from boot/.
ROOT := ..

# -- source lists ----------------------------------------------------------
BOOT_SRCS := \
  main.c \
  boot_flag.c \
  boot_net_cfg.c \
  boot_od.c \
  boot_seg_sdo.c \
  boot_flash.c \
  boot_spi.c \
  boot_stubs.c \
  boot_log_stub.c

BSP_SRCS := \
  $(ROOT)/bsp/net/net.c \
  $(ROOT)/bsp/net/wizchip_glue.c \
  $(ROOT)/bsp/flash/flash.c \
  $(ROOT)/bsp/time/time.c \
  $(ROOT)/bsp/identity/identity.c

DRV_SRCS := \
  $(ROOT)/Drivers/w6100/W6100/w6100.c \
  $(ROOT)/Drivers/w6100/socket.c \
  $(ROOT)/Drivers/w6100/wizchip_conf.c

# Cube-generated files reused by the bootloader. system_stm32g4xx.c is
# compiled with VECT_TAB_OFFSET=0U (see CFLAGS_BOOT_ONLY) so the bootloader's
# VTOR stays at 0x08000000. spi.c and stm32g4xx_it.c are DELIBERATELY not
# included — the bootloader provides its own boot_spi.c (no DMA — see
# comment there) and boot_stubs.c (minimal interrupt handlers; Cube's
# stm32g4xx_it.c references peripheral handles like hi2c2 / hdma_* that
# the bootloader doesn't own).
CORE_SRCS := \
  $(ROOT)/Core/Src/system_stm32g4xx.c \
  $(ROOT)/Core/Src/gpio.c \
  $(ROOT)/Core/Src/stm32g4xx_hal_msp.c

# HAL sources — every one the bootloader could plausibly call. gc-sections
# in the linker drops unused ones from the final .elf.
HAL_DIR  := $(ROOT)/Drivers/STM32G4xx_HAL_Driver/Src
HAL_SRCS := \
  $(HAL_DIR)/stm32g4xx_hal.c \
  $(HAL_DIR)/stm32g4xx_hal_cortex.c \
  $(HAL_DIR)/stm32g4xx_hal_flash.c \
  $(HAL_DIR)/stm32g4xx_hal_flash_ex.c \
  $(HAL_DIR)/stm32g4xx_hal_flash_ramfunc.c \
  $(HAL_DIR)/stm32g4xx_hal_gpio.c \
  $(HAL_DIR)/stm32g4xx_hal_pwr.c \
  $(HAL_DIR)/stm32g4xx_hal_pwr_ex.c \
  $(HAL_DIR)/stm32g4xx_hal_rcc.c \
  $(HAL_DIR)/stm32g4xx_hal_rcc_ex.c \
  $(HAL_DIR)/stm32g4xx_hal_spi.c \
  $(HAL_DIR)/stm32g4xx_hal_spi_ex.c \
  $(HAL_DIR)/stm32g4xx_hal_tim.c \
  $(HAL_DIR)/stm32g4xx_hal_tim_ex.c \
  $(HAL_DIR)/stm32g4xx_hal_exti.c

ASM_SRCS := $(ROOT)/Core/Startup/startup_stm32g474retx.s

ALL_C_SRCS := $(BOOT_SRCS) $(BSP_SRCS) $(DRV_SRCS) $(CORE_SRCS) $(HAL_SRCS)

# -- output files ----------------------------------------------------------
C_OBJS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(BOOT_SRCS))) \
          $(patsubst $(ROOT)/%,$(BUILD_DIR)/%.o,$(basename $(BSP_SRCS) $(DRV_SRCS) $(CORE_SRCS) $(HAL_SRCS)))

ASM_OBJS := $(patsubst $(ROOT)/%,$(BUILD_DIR)/%.o,$(basename $(ASM_SRCS)))

ALL_OBJS := $(C_OBJS) $(ASM_OBJS)

# -- flags -----------------------------------------------------------------
INCLUDES := \
  -I$(ROOT)/Core/Inc \
  -I$(ROOT)/Drivers/STM32G4xx_HAL_Driver/Inc \
  -I$(ROOT)/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy \
  -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
  -I$(ROOT)/Drivers/CMSIS/Include \
  -I$(ROOT)/Drivers/w6100 \
  -I$(ROOT) \
  -I.

MCU_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS      := -DSTM32G474xx -DUSE_HAL_DRIVER -DDEBUG
CFLAGS    := $(MCU_FLAGS) $(DEFS) $(INCLUDES) \
             -std=gnu11 -g3 -O0 \
             -ffunction-sections -fdata-sections \
             -Wall -fstack-usage -fcyclomatic-complexity \
             --specs=nano.specs

# The bootloader's VTOR must stay at 0x08000000 — override the app-default
# VECT_TAB_OFFSET=0x8000 that Core/Src/system_stm32g4xx.c uses.
CFLAGS_BOOT_ONLY := -DUSER_VECT_TAB_ADDRESS -DVECT_TAB_OFFSET=0U

ASFLAGS := $(MCU_FLAGS) -g3 -x assembler-with-cpp $(DEFS)

LDFLAGS := $(MCU_FLAGS) -T$(CURDIR)/boot_link.ld \
           --specs=nano.specs \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
           -Wl,--gc-sections \
           -static -Wl,--start-group -lc -lm -Wl,--end-group

# -- default target --------------------------------------------------------
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@$(SIZE) $(BUILD_DIR)/$(TARGET).elf

# -- generic rules ---------------------------------------------------------
# Bootloader-local .c files (main.c, boot_*.c).
$(BUILD_DIR)/main.o: main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CFLAGS_BOOT_ONLY) -c $< -o $@
$(BUILD_DIR)/boot_%.o: boot_%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CFLAGS_BOOT_ONLY) -c $< -o $@

# Everything from the project tree lands under $(BUILD_DIR)/<mirrored path>.o.
# Two patterns cover Core/Src/*, Drivers/**, bsp/**.
$(BUILD_DIR)/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CFLAGS_BOOT_ONLY) -c $< -o $@

$(BUILD_DIR)/%.o: $(ROOT)/%.s
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

# -- link + convert --------------------------------------------------------
$(BUILD_DIR)/$(TARGET).elf: $(ALL_OBJS) boot_link.ld | $(BUILD_DIR)
	$(CC) $(ALL_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean

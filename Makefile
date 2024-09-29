# Makefile to build firmware for STM32F429 Discovery Board
#
# Call with "make Q=" to enable full path names
#
# TOOLCHAIN_ROOT path must be set.
# Create a config.mk with the following path to the cross compiler GCC:
#
# 	TOOLCHAIN_ROOT=/path/to/gcc-arm-none-eabi-XX.XX-XX/bin/
#

-include config.mk

ifeq ($(TOOLCHAIN_ROOT),)
	$(error TOOLCHAIN_ROOT is not defined. Please set TOOLCHAIN_ROOT in config.mk)
endif

Q ?= @

TARGET_COMPILER ?= gcc
# default compiler optimization level:
export OPTIMIZE_LEVEL ?= 3
# APP_CPP_FLAGS   += -g

APPNAME         := firmware
OBJDIR          := build

APP_CPP_FLAGS   += -fno-strict-aliasing -fno-math-errno
ODFLAGS         := -x --syms
#
CROSS_COMPILE   ?= arm-none-eabi-
ARCH_FLAGS      := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian --specs=nosys.specs
LINKER_SCRIPT   := STM32F429ZITX_FLASH.ld
APP_CPP_FLAGS   += -DUSE_HAL_DRIVER -DUSE_FULL_ASSERT -DSTM32F4xx -DARM_MATH_CM4
APP_CPP_FLAGS   += -DSTM32F429xx

# -MMD: to autogenerate dependencies for make
# -MP: These dummy rules work around errors make gives if you remove header
#      files without updating the Makefile to match.
# -MF: When used with the driver options -MD or -MMD, -MF overrides the default
#      dependency output file.
# -fno-common: This has the effect that if the same variable is declared
#              (without extern) in two different compilations, you get a
#              multiple-definition error when you link them.
# -fmessage-length=n: If n is zero, then no line-wrapping is done; each error
#                     message appears on a single line.
# -fno-math-errno: Otherwise the default sqrt() function is used and we won't
#                  get the ARM vsqrt.f32 (14 cycle) instruction (but not in O0)
# --specs=nano.specs: Use newlib nano libc
# --specs=nosys.specs: semihosting disabled

# GCC compiler warnings
GCC_STACK_WARNING_BYTES := 4096
WARNING_CHECKS  := -Wall
WARNING_CHECKS  += -Wframe-larger-than=$(GCC_STACK_WARNING_BYTES)
WARNING_CHECKS  += -Wstack-usage=$(GCC_STACK_WARNING_BYTES)
WARNING_CHECKS  += -Wdouble-promotion
WARNING_CHECKS  += -Wpointer-arith
WARNING_CHECKS  += -Wformat=2
WARNING_CHECKS  += -Wmissing-include-dirs
WARNING_CHECKS  += -Wwrite-strings
WARNING_CHECKS  += -Wlogical-op
WARNING_CHECKS  += -Winline
WARNING_CHECKS  += -Wunreachable-code
WARNING_CHECKS  += -Wno-unknown-pragmas
WARNING_CHECKS  += -Wvla
WARNING_CHECKS  += -Wdate-time
# WARNING_CHECKS  += -Wstrict-prototypes
# WARNING_CHECKS  += -Wredundant-decls
# WARNING_CHECKS  += -Wold-style-definition
# WARNING_CHECKS  += -Wswitch-enum
# WARNING_CHECKS  += -Wshadow

default: all

# App source directories
APP_SUBDIRS += \
	./Core/Src \
	./src/components/vote \
	./Drivers/BSP/STM32F429I-Discovery \
	./Drivers/STM32F4xx_HAL_Driver/Src \
	./Drivers/BSP/Components/l3gd20 \
	./Drivers/BSP/Components/stmpe811 \
	./Drivers/BSP/Components/ili9341 \


# Include directories
APP_INCLUDE_PATH += \
	-I"Core/Inc/" \
	-I"Core/Src/" \
	-I"Drivers/BSP/Components/Common" \
	-I"Drivers/BSP/STM32F429I-Discovery/" \
	-I"Drivers/STM32F4xx_HAL_Driver/Inc/" \
	-I"Drivers/CMSIS/Include/" \
	-I"Drivers/CMSIS/Device/ST/STM32F4xx/Include/" \
	-I"Drivers/BSP/Components/l3gd20/" \
	-I"Drivers/BSP/Components/stmpe811/" \
	-I"Drivers/BSP/Components/ili9341/" \


S_STARTUP := startup_stm32f429zitx
S_SRC += Core/Startup/$(S_STARTUP).s

LIBRARIES := -lm

# =======================================================================

COMPILER = $(CROSS_COMPILE)$(TARGET_COMPILER)

CPP_FLAGS = $(APP_CPP_FLAGS)
COMPILER_FLAGS  = -O$(OPTIMIZE_LEVEL)
# -c: Compile without linking:
COMPILER_FLAGS  += ${WARNING_CHECKS} -c
COMPILER_FLAGS  += -flto -MMD ${CPP_FLAGS} ${ARCH_FLAGS}
ASM_FLAGS       = -x assembler-with-cpp ${COMPILER_FLAGS}
LINK_FLAGS      = -flto -Wl,--gc-sections -T $(LINKER_SCRIPT) ${ARCH_FLAGS} -Wl,-Map=${APPNAME}.map

OC              := ${TOOLCHAIN_ROOT}${CROSS_COMPILE}objcopy
OD              := ${TOOLCHAIN_ROOT}${CROSS_COMPILE}objdump
HEX             := ${OC} -O ihex   # intel .hex file output
BIN             := ${OC} -O binary # binary output
SIZEINFO        := ${TOOLCHAIN_ROOT}${CROSS_COMPILE}size
RM              := rm -f
CP              := cp
CC              := ${TOOLCHAIN_ROOT}${COMPILER}
LINK            := ${TOOLCHAIN_ROOT}${COMPILER}

APP_SRCS         = $(foreach dir, $(APP_SUBDIRS), $(wildcard $(dir)/*.c))
C_SRCS           = $(APP_SRCS)
VPATH            = $(APP_SUBDIRS)
OBJ_NAMES        = $(notdir $(C_SRCS))
OBJS             = $(addprefix $(OBJDIR)/,$(OBJ_NAMES:%.c=%.o))
OBJS             += $(OBJDIR)/$(S_STARTUP).o
C_DEPS           = $(OBJS:%.o=%.d)
C_INCLUDES       = $(APP_INCLUDE_PATH)

COMPILER_CMDLINE = -std=c99 $(COMPILER_FLAGS) $(C_INCLUDES)

$(OBJDIR)/$(S_STARTUP).o: $(S_SRC)
	@echo 'ASM: $<'
	$(Q)$(CC) $(ASM_FLAGS) $(C_INCLUDES) -o "$@" "$<"

$(OBJDIR)/%.o: %.c
	@echo 'CC: $<'
	$(Q)$(CC) $(COMPILER_CMDLINE) -o "$@" "$<"

EXECUTABLES += \
	${APPNAME}.elf \
	${APPNAME}.list \
	${APPNAME}.dmp \

# All Target
all: $(EXECUTABLES)

# Make sure that we recompile if a header file was changed
-include $(C_DEPS)

# App
${APPNAME}.elf: $(OBJS) ${LINKER_SCRIPT}
	@echo 'Object files: '$(OBJS)
	@echo 'Building target: $@ with '$(CC)
	@echo 'Building machine: '$(HOSTNAME)
	@echo 'Optimize level:' $(OPTIMIZE_LEVEL)
	$(Q)$(LINK) ${LINK_FLAGS} -o "$@" $(OBJS) $(LIBRARIES)
	@echo 'Finished building target: $@'
	$(Q)$(MAKE) --no-print-directory post-build

clean:
	$(RM) $(APPNAME).bin $(APPNAME).hex
	$(RM) $(EXECUTABLES) ${APPNAME}.map
	$(RM) $(OBJDIR)/*.d
	$(RM) $(OBJDIR)/*.o
	$(RM) $(OBJDIR)/*.dbo
	$(RM) $(OBJDIR)/*.lst

post-build:
	@echo "Creating ${APPNAME}.bin"
	$(Q)$(BIN) "${APPNAME}.elf" "${APPNAME}.bin"
	@echo "Creating ${APPNAME}.hex"
	$(Q)$(HEX) "${APPNAME}.elf" "${APPNAME}.hex"
	$(Q)$(SIZEINFO) "${APPNAME}.elf"

%.list: %.elf
	@echo "Assembler output $@"
	$(Q)$(OD) -S $< > $@

%.dmp: %.elf
	@echo "Creating dump file $@"
	$(Q)$(OD) $(ODFLAGS) $< > $@

flash: all
ifeq ($(OS),Windows_NT)
	@./flash.bat
else
	@./flash.sh
endif

.FORCE:

.PHONY: all clean flash post-build

.SECONDARY: post-build

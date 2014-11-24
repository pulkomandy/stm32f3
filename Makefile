# Minimal makefile for STM32F3 Discovery board.
TARGET := arm-none-eabi

# Find the devtools
CC := $(shell { which $(TARGET)-gcc || which $(TARGET)-gcc-x86; } 2>/dev/null)
OBJCOPY := $(shell { which $(TARGET)-objcopy || which $(TARGET)-objcopy-x86; } 2>/dev/null)
STM32FLASH := $(shell { which stm32flash || which stm32flash-x86; } 2>/dev/null)
STFLASH := $(shell { which st-flash || which st-flash-x86; } 2>/dev/null)
PORT := $(shell { ls /dev/ttyS0 || ls /dev/ports/usb0; } 2>/dev/null)

# If hey is not available, this resolves to #, and makes the hey commands
# invisible to the shell. Dirty, I know.
HEY := $(shell { which hey || echo \\\# ; } 2>/dev/null)


a.bin: a.out
	$(OBJCOPY) a.out -I ihex -O binary a.bin

a.out: src/main.c src/startup.s stm32f303.ld
	$(CC) -g -mcpu=cortex-m3 -mthumb -nostdlib -Isrc -Os -ffunction-sections -fdata-sections -Wl,--gc-sections -std=c99 src/main.c src/startup.s -T stm32f303.ld

.PHONY: clean flash

stm32flash: a.bin
	$(HEY) SerialConnect delete port
	$(STM32FLASH) -v -w $^ $(PORT)
	$(HEY) SerialConnect set port to $(basename $(PORT))

st-flash: a.bin
	$(STFLASH) write a.bin 0x8000000

clean:
	rm a.out a.bin

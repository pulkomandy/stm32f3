# Minimal makefile for STM32F3 Discovery board.
TARGET := arm-none-eabi

# Find the devtools
CC := $(shell { which $(TARGET)-gcc || which $(TARGET)-gcc-x86; } 2>/dev/null)
OBJCOPY := $(shell { which $(TARGET)-objcopy || which $(TARGET)-objcopy-x86; } 2>/dev/null)
STM32FLASH := $(shell { which stm32flash || which stm32flash-x86; } 2>/dev/null)
PORT := $(shell { ls /dev/ttyS0 || ls /dev/ports/usb0; } 2>/dev/null)


a.bin: a.out
	$(OBJCOPY) a.out -I ihex -O binary a.bin

a.out: src/main.c src/startup.s stm32f303.ld
	$(CC) -mcpu=cortex-m3 -mthumb -Isrc -Os -ffunction-sections -fdata-sections -Wl,--gc-sections -nostdlib -std=c99 src/main.c src/startup.s -T stm32f303.ld

.PHONY: clean flash

flash: a.bin
	$(STM32FLASH) -v -w $^ $(PORT)

clean:
	rm a.out a.bin

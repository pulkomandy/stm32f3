# Minimal makefile for STM32F3 Discovery board.

a.bin: a.out
	arm-none-eabi-objcopy a.out -I ihex -O binary a.bin

a.out: src/main.c src/startup.s stm32f303.ld
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Isrc -Os -ffunction-sections -fdata-sections  -Wl,--gc-sections -nostdlib -std=c99 src/main.c src/startup.s -T stm32f303.ld

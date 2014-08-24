stm32f3 Discovery template project.
===================================

Some of you may remember my article from 2 years ago about how to get started
with development on the STM32F3 discovery board:
	http://pulkomandy.tk/_/_Electronique/_Discovering%20the%20STM32F3%20Discovery

2 years have passed now and things have become a little easier. I also found
some things that were wrong in the original project. So let's go for an updated
version of this!

Features
--------

 * No complex setup with OpenOCD and GDB and drivers to install. We will use
 the DFU bootloader which is built in the STM32F3 ROM.
 * Several issues in the original project fixed
 * This time, it actually blinks some LEDs!

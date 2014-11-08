#include <stm32f30x.h>

static inline void ConfigureLEDs(void)
{
	// Enable the GPIO clock for ports E (LEDs) and A (USART1)
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN | RCC_AHBENR_GPIOAEN;

	// Configure the GPIOs for the on-board LEDs as outputs
	GPIOE->MODER  = (GPIOE->MODER & 0x0000FFFF) | 0x55550000;
}

static inline void ConfigureUSART1(void)
{
	// Distribute clock to USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// Select AF7 (USART1) for PA9 and PA10
	GPIOA->AFR[1] = (GPIOA->AFR[1] & 0xFFFFF00F) | 0x770;

	// PA9 and PA10 as AF
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;

	// Configure baudrate
	USART1->BRR  = 8000000L/115200L;

	// Eable TX, RX and enable USART
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}

int main(void)
{
	ConfigureLEDs();
	ConfigureUSART1();

	// Loop forever
	for(;;)
	for(int i = 0x100; i <= 0x8000; i <<= 1) {
		// All leds off except current one
		GPIOE->ODR = (GPIOE->ODR & 0x00FF) | i;

		// Let's also send some garbabe to the UART
		// (assuming it was setup by the bootloader)
		USART1->TDR = 'U';

		// Sleep for a while so it doesn't blink too fast
		for(int j = 0; j < 0x3ffff; j++);
	};
	return 0;
}

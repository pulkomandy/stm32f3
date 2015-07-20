#include <stm32f30x.h>
#include <stdio.h>

static inline void ConfigureLEDs(void)
{
	// Enable the GPIO clock for ports E (LEDs) and A (USART1, SPI1)
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN | RCC_AHBENR_GPIOAEN;

	// Configure the GPIOs for the on-board LEDs as outputs
	// LEDs are on PE8-PE15
	GPIOE->MODER  = (GPIOE->MODER & 0x0000FFFF) | 0x55550000;
}


// USART ----------------------------------------------------------------------


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


static inline char ReadUSART1(void)
{
	// Wait for a char on the UART
	while (!(USART1->ISR & USART_ISR_RXNE));
	return USART1->RDR;
}


static inline void WriteUSART1(char c)
{
	// Wait for a char on the UART
	while (!(USART1->ISR & USART_ISR_TXE));
	USART1->TDR = c;
}


static inline void WriteX16Usart1(int v)
{
	char c;
	c = (v >> 12) & 0xF;
	if (c > 9) c+= ('A' - 10); else c+= '0';
	WriteUSART1(c);

	c = (v >> 8) & 0xF;
	if (c > 9) c+= ('A' - 10); else c+= '0';
	WriteUSART1(c);

	c = (v >> 4) & 0xF;
	if (c > 9) c+= ('A' - 10); else c+= '0';
	WriteUSART1(c);

	c = (v) & 0xF;
	if (c > 9) c+= ('A' - 10); else c+= '0';
	WriteUSART1(c);

	WriteUSART1(0x20);
}


// Gravis GRiP ----------------------------------------------------------------

static volatile int GravisState;

static inline void ConfigureGravis()
{
	// This is connected on GPIO port C (PC0 and PC1)
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// PC0: clock
	// PC1: data
	// They are inputs by default, nothing to do
	
	// Configure interrupts on PC0
	SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PC;

	EXTI->FTSR |= 1;
	EXTI->IMR |= 1;

	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_IRQn);

	//EXTI->PR |= 1;
}


void EXTI0_IRQHandler()
{

	static int bits;
	static int state;
	int bit = (GPIOC->IDR & 2) != 0;

	switch(state) {
		default:
			bits = 0;
			state = 0;

		case 0: //SYNC
			if (bit)
				bits++;
			else
				bits=0;
			if (bits >= 5) {
				state = 1;
				bits = 0;
			}
			break;

		case 1: // DATA
			if (bit)
				GravisState |= 1 << bits;
			else
				GravisState &= ~(1 << bits);
			bits++;
			if (bits >= 18) {
				bits = 0;
				state = 0;
			}
			break;
	}

	GPIOE->ODR = (GPIOC->ODR & 0xFFFF00FF) | (1 << (state + 8));

	EXTI->PR |= 1;
}


// Main -----------------------------------------------------------------------


int main(void)
{
	ConfigureLEDs();
	ConfigureUSART1();
	ConfigureGravis();

	int oldck = 0;
	int newck = 0;

	// Loop forever
	for(;;)
	{
		newck = GPIOC->IDR & 1;
		if (!newck && oldck) {
			// new bit
			//EXTI->SWIER |= 1;
		}

		oldck = newck;

	//	GPIOE->ODR = (GPIOC->ODR & 0xFFFF00FF) | (GravisState << 8) | GravisState | ((GravisState >> 8) & 0x300);
		GPIOE->ODR = (GPIOC->ODR & 0xFFFF00FF) | 0x800;

#if 0
		if (c == 'U')
			return 1;
#endif
	};
	return 0;
}

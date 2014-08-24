#include <stm32f30x.h>

int main(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;

	GPIOE->MODER  = (GPIOE->MODER & 0x0000FFFF) | 0x55550000;


	for(;;)
	for(int i = 0x100; i <= 0x8000; i <<= 1) {
		GPIOE->ODR = (GPIOE->ODR & 0x00FF) | i;
		for(int j = 0; j < 0xfffff; j++);
	};
	return 0;
}

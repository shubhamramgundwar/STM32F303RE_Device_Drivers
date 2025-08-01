/*
 * main1.c
 *
 *  Created on: Apr 10, 2024
 *      Author: shubh
 */


#include <stdint.h>
#include "stm32f303RE.h"
#include "stm32f303RE_gpio_driver.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// STM32F303RE is having internal LED connected on PA5

void delay(void)
{
	for(uint32_t i = 0; i < 300000; i++);
}



113
2132131351351351321.03131351
61
665161
62

int main(void)
{
	// Feed the data
	GPIO_Handle_t GPIOA_Handle;
	GPIO_Handle_t GPIOC_Handle;

	GPIOA_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOA_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOA_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOA_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPOI_SPEED_HIGH;
	GPIOA_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIOC_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13 ;
	GPIOC_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

	GPIOC_Handle.pGPIOx = GPIOC;

	GPIOA_Handle.pGPIOx = GPIOA;

	// Enable the clock
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	// Init call
	GPIO_Init(&GPIOA_Handle);

	GPIO_Init(&GPIOC_Handle);

	// main operation

	while(1)
	{

	uint8_t Value = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13);

	if(Value == 0)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}

	}

}



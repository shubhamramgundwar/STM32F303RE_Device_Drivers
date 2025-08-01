/*
 * stm32f407xx_gpio_driver.c
 *
 */


#include "stm32f303RE_gpio_driver.h"
#include <stdint.h>


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PClk_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PClk_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PClk_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PClk_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PClk_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PClk_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PClk_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PClk_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PClk_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PClk_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PClk_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PClk_DI();
		}
	}

}




/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	 //enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	 uint32_t temp=0; //temp. register

	//1 . configure the mode of gpio pin

	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= 3)
	 {
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	 }
	 else
	 {
		// 1. F/R/RF trigger configure
		 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		 {
			 // SET falling trigger mechanism
			 EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 // Clear Raising trigger mechanism
			 EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }
		 else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		 {
			 // SET Raising  trigger mechanism
			 EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 // Clear Falling trigger mechanism
			 EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }
		 else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		 {
			 // SET falling trigger mechanism
			 EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 // SET Raising  trigger mechanism
			 EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }

		// 2. Configre port selection in SYSCFG_EXTICRx
		 uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		 uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
		 uint8_t PortCode = PORT_ADDRESS_TO_CODE(pGPIOHandle->pGPIOx);
		 SYSCFG_PClk_EN();
		 SYSCFG->EXTICR[temp1] = (PortCode << (temp2 * 4));

		// 3. Configure exit interrupt EXTI->IMR
		 EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	 }

    temp=0; //temp. register

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	  temp=0; //temp. register

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	  temp=0; //temp. register


	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	  temp=0; //temp. register

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}
	  temp=0; //temp. register

}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}

}



/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;

   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 ) ;

   return value;
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR  = Value;
}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		// Use ISER register for Enabling the IRQ
		if(IRQNumber >= 0 && IRQNumber <= 31)
		{
			//ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber <= 63)
		{
			//ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber <= 95)
		{
			//ISER2
			*NVIC_ISER1 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		//Use ICER register for Disabling the IRQ
		if(IRQNumber >= 0 && IRQNumber <= 31)
		{
			//ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber <= 63)
		{
			//ICER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber <= 95)
		{
			//ICER2
			*NVIC_ICER1 |= (1 << (IRQNumber % 64));
		}
	}
}


/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t temp1 = IRQNumber / 4;
	uint8_t temp2 = IRQNumber % 4;

	uint8_t ShiftValue = ((8 * temp2) + NO_PR_IMPLEMENTED_BIT);

	*(NVIC_IPRx + temp1) |= (IRQPriority << ShiftValue);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQ_Handle
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_IRQ_Handle(uint8_t PinNumber)
{
 // clear PR bit inside PR register for corresponding pin number
	if(EXTI->PR1 & (1 << PinNumber))
	{
		//Clear
		EXTI->PR1 |= (1 << PinNumber);
	}
}

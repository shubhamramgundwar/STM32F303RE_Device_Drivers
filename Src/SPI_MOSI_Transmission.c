/*
 * main_SPI2.c
 *
 *  Created on: Apr 21, 2024
 *      Author: Ram
 */

#include <stdint.h>
#include "stm32f303RE.h"
#include<string.h>

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 * PC13 --> Button
 */


void delay()
{
	for(uint32_t i; i<300000; i++);
}

void GPIO_Button_Init()
{

	GPIO_Handle_t GPIOC_Handle;
	GPIOC_Handle.pGPIOx = GPIOC;
	GPIOC_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13 ;
	GPIOC_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOC_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOC_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPOI_SPEED_HIGH;
	GPIOC_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_Init(&GPIOC_Handle);
}

void SPI_GPIO_Init()
{
	GPIO_Handle_t GPIOB_Handle;
	GPIOB_Handle.pGPIOx = GPIOB;
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPOI_SPEED_HIGH;
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	//SS pin init
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&GPIOB_Handle);

	//SCK pin init
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GPIOB_Handle);

//	//MISO pin Init
//	GPIOB_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&GPIOB_Handle);

	//MOSI pin Init
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&GPIOB_Handle);
}


void SPI_Initialize()
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_Speed = SPI_SCLK_SPEED_DIV64;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; // Disabling software SS to enable HW

	SPI_Init(&SPI2handle);
}

int main()
{
	char Data[] = "Hello Slave!\n";

	//Initialize button GPIO
	GPIO_Button_Init();

	//Initialize SPI GPIO pins
	SPI_GPIO_Init();

	//Initialize SPI
	SPI_Initialize();

	//SSOE enable
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{

		//hanging until button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		//SPI pheripheral enable
		SPI_PeripheralControl(SPI2, ENABLE);

		//Data send call
		SPI_TransmitData(SPI2, (uint8_t *)Data, sizeof(Data));

		//Hang till SPI is free
		while(SPI_GetFlagStatus(SPI2 , SPI_BUSY_FLAG));

		//SPI pheripheral Disable
		SPI_PeripheralControl(SPI2, DISABLE);

	}
	return 0;
}

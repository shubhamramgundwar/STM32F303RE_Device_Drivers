/*
 * stm32f303RE_SPI_driver.c
 *
 *  Created on: Apr 18, 2024
 *      Author: Ram
 */


#include "stm32f303RE_SPI_driver.h"
#include<stdint.h>



/*
 * Peripheral Clock setup
 */

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PClk_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PClk_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PClk_EN();
		}else if (pSPIx == SPI4)
		{
			SPI4_PClk_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PClk_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PClk_DI();
		}else if (pSPIx == SPI3)
		{
			SPI3_PClk_DI();
		}else if (pSPIx == SPI4)
		{
			SPI4_PClk_DI();
		}
	}

}



/*
 * Init and De-init
 */

/*********************************************************************
 * @fn      		  - SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure SPI_CR1 register
	uint32_t TempReg = 0;
	//Configure Device mode
	TempReg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	//Configure SPI BusConfig
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_FD)
	{
		//configure FD
		TempReg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_HD)
	{
		//configure HD
		TempReg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_Simplex_RXONLY)
	{
		//configure Simplex in Rx only mode
		TempReg &= ~(1 << SPI_CR1_BIDIMODE);

		TempReg |= (1 << SPI_CR1_RXONLY);
	}

	//configure SPI_CPHA
	TempReg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	//configure SPI_CPOL
	TempReg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//configure SPI Speed
	TempReg |= pSPIHandle->SPI_Config.SPI_Speed << SPI_CR1_BR;

	//configure SPI SSM
	TempReg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	//Copy all configuration of Handle to Main CR register
	pSPIHandle->pSPIx->CR1 = TempReg;

}


/*********************************************************************
 * @fn      		  - SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/*
 * Transmit data
 */

/*********************************************************************
 * @fn      		  - SPI_TransmitData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call

 */
void SPI_TransmitData(SPI_RegDef_t *pSPIx, uint8_t * pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(!(pSPIx->SR & (1<<SPI_SR_TXE)));

		pSPIx->DR = (uint16_t)*pTxBuffer;

		Len--;
		pTxBuffer++;
	}
}


/*
 * Receive Data
 */


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t * pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(!((pSPIx->SR) & (1 <<SPI_SR_RXNE)));

		 *pRxBuffer = (uint8_t)(pSPIx->DR);

		Len--;
		pRxBuffer++;

	}
}


/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
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
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}


/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


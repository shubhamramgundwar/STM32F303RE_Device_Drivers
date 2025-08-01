#ifndef DRIVER_INC_STM32F303RE_H_
#define DRIVER_INC_STM32F303RE_H_

#include <stdint.h>



//Controller specific addresses
#define NVIC_ISER0           ((volatile uint32_t *)(0xE000E100U))
#define NVIC_ISER1           ((volatile uint32_t *)(0xE000E104U))
#define NVIC_ISER2           ((volatile uint32_t *)(0xE000E108U))
#define NVIC_ICER0           ((volatile uint32_t *)(0XE000E180U))
#define NVIC_ICER1           ((volatile uint32_t *)(0XE000E184U))
#define NVIC_ICER2           ((volatile uint32_t *)(0XE000E188U))
#define NVIC_IPRx            ((volatile uint32_t *)(0xE000E400U))


#define NO_PR_IMPLEMENTED_BIT    4

//MEMORY BASEADDRESS
#define FLASH_BASEADDRESS	0x08000000U		/*main Flash memory base address */
#define SRAM_BASEADDRESS	0x20000000U		/*main SRAM memory base address */
#define ROM_BASEADDRESS		0x1FFFD800U		/*main memory (ROM) base address */


//PERIPHERAL BASEADDRESS
#define PERIPHERAL_BASEADDRESS		0x40000000U

#define APB1PERIPH_OFFSET			0x0000U
#define APB2PERIPH_OFFSET			0x10000U
#define AHB1PERIPH_OFFSET			0x20000U
#define AHB2PERIPH_OFFSET			0x8000000U

#define APB1PERIPH_BASEADDRESS	    (PERIPHERAL_BASEADDRESS +APB1PERIPH_OFFSET )
#define APB2PERIPH_BASEADDRESS		(PERIPHERAL_BASEADDRESS +APB2PERIPH_OFFSET )
#define AHB1PERIPH_BASEADDRESS		(PERIPHERAL_BASEADDRESS +AHB1PERIPH_OFFSET )
#define AHB2PERIPH_BASEADDRESS		(PERIPHERAL_BASEADDRESS +AHB2PERIPH_OFFSET )


//APB1 PERIPHERAL BASE
#define SPI2_BASEADDRESS   (APB1PERIPH_BASEADDRESS + 0x3800U)
#define SPI3_BASEADDRESS   (APB1PERIPH_BASEADDRESS + 0x3C00U)
#define USART2_BASEADDRESS (APB1PERIPH_BASEADDRESS + 0x4400U)
#define USART3_BASEADDRESS (APB1PERIPH_BASEADDRESS + 0x4800U)
#define UART4_BASEADDRESS  (APB1PERIPH_BASEADDRESS + 0x4C00U)
#define UART5_BASEADDRESS  (APB1PERIPH_BASEADDRESS + 0x5000U)
#define I2C1_BASEADDRESS   (APB1PERIPH_BASEADDRESS + 0x5400U)
#define I2C2_BASEADDRESS   (APB1PERIPH_BASEADDRESS + 0x5800U)
#define I2C3_BASEADDRESS   (APB1PERIPH_BASEADDRESS + 0x7800U)


//APB2 PERIPHERAL BASE
#define EXTI_BASEADDRESS     (APB2PERIPH_BASEADDRESS + 0x0400U)
#define SYSCFG_BASEADDRESS   (APB2PERIPH_BASEADDRESS + 0x0000U)
#define SPI1_BASEADDRESS     (APB2PERIPH_BASEADDRESS + 0x3000U)
#define USART1_BASEADDRESS   (APB2PERIPH_BASEADDRESS + 0x3800U)
#define SPI4_BASEADDRESS     (APB2PERIPH_BASEADDRESS + 0x3C00U)

//AHB1 PERIPHERAL BASE
#define RCC_BASEADDRESS      (AHB1PERIPH_BASEADDRESS + 0x1000U)


//AHB2 PERIPHERAL BASE
#define GPIOA_BASEADDRESS (AHB2PERIPH_BASEADDRESS +0x0000U)
#define GPIOB_BASEADDRESS (AHB2PERIPH_BASEADDRESS +0x0400U)
#define GPIOC_BASEADDRESS (AHB2PERIPH_BASEADDRESS +0x0800U)
#define GPIOD_BASEADDRESS (AHB2PERIPH_BASEADDRESS +0x0C00U)
#define GPIOE_BASEADDRESS (AHB2PERIPH_BASEADDRESS +0x1000U)
#define GPIOF_BASEADDRESS (AHB2PERIPH_BASEADDRESS +0x1400U)




//GPIOx REGISTER STRUCTURE
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_RegDef_t;



//SPIx REGISTER STRUCTURE
typedef struct
{
	volatile uint32_t CR1;   // offset is 00
	volatile uint32_t CR2;    // offset is 04
	volatile uint32_t SR;   // offset is 08
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;




// RCC RegDef
/* periferal definion (periferal base address typecast to xxx_RegDef ) */

typedef struct
{

	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t CFGR2;
	volatile uint32_t CFGR3;

}RCC_RegDef_t;



//EXTI REGISTER STRUCTURE
typedef struct
{
	volatile uint32_t IMR1;
	volatile uint32_t EMR1;
	volatile uint32_t RTSR1;
	volatile uint32_t FTSR1;
	volatile uint32_t SWIER1;
	volatile uint32_t PR1;
	volatile uint32_t IMR2;
	volatile uint32_t EMR2;
	volatile uint32_t RTSR2;
	volatile uint32_t FTSR2;
	volatile uint32_t SWIER2;
	volatile uint32_t PR2;
}EXTI_RegDef_t;


//SYSCFG REGISTER STRUCTURE
typedef struct
{
	volatile uint32_t CFGR1;
	volatile uint32_t RCR;
	volatile uint32_t EXTICR[4];
}SYSCFG_RegDef_t;

// GPIO Address Macros
#define GPIOA  ((GPIO_RegDef_t*) GPIOA_BASEADDRESS)
#define GPIOB  ((GPIO_RegDef_t*) GPIOB_BASEADDRESS)
#define GPIOC  ((GPIO_RegDef_t*) GPIOC_BASEADDRESS)
#define GPIOD  ((GPIO_RegDef_t*) GPIOD_BASEADDRESS)
#define GPIOE  ((GPIO_RegDef_t*) GPIOE_BASEADDRESS)
#define GPIOF  ((GPIO_RegDef_t*) GPIOF_BASEADDRESS)


// SPI Address Macros
#define SPI1   ((SPI_RegDef_t*) SPI1_BASEADDRESS)
#define SPI2   ((SPI_RegDef_t*) SPI2_BASEADDRESS)
#define SPI3   ((SPI_RegDef_t*) SPI3_BASEADDRESS)
#define SPI4   ((SPI_RegDef_t*) SPI4_BASEADDRESS)


#define RCC     ((RCC_RegDef_t*)RCC_BASEADDRESS)
#define EXTI 	((EXTI_RegDef_t*)EXTI_BASEADDRESS)
#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDRESS)

/*
 * CLOCK ENABLE MACROS GPIOx PERIPHERAL
 */
#define GPIOA_PClk_EN()  (RCC->AHBENR|=(1<<17))
#define GPIOB_PClk_EN()  (RCC->AHBENR|=(1<<18))
#define GPIOC_PClk_EN()  (RCC->AHBENR|=(1<<19))
#define GPIOD_PClk_EN()  (RCC->AHBENR|=(1<<20))
#define GPIOE_PClk_EN()  (RCC->AHBENR|=(1<<21))
#define GPIOF_PClk_EN()  (RCC->AHBENR|=(1<<22))


/*
 * CLOCK ENABLE MACROS SPIx PERIPHERAL
 */
#define SPI1_PClk_EN()  (RCC->APB2ENR|=(1<<12))
#define SPI2_PClk_EN()  (RCC->APB1ENR|=(1<<14))
#define SPI3_PClk_EN()  (RCC->APB1ENR|=(1<<15))
#define SPI4_PClk_EN()  (RCC->APB2ENR|=(1<<15))


/*
 * CLOCK ENABLE MACROS SYSCFG PERIPHERAL
 */
#define SYSCFG_PClk_EN()  (RCC->APB2ENR|=(1<<0))

/*
 * CLOCK DISABLE MACROS GPIOx PERIPHERAL
 */

#define GPIOA_PClk_DI()  (RCC->AHBENR &= ~(1<<17))
#define GPIOB_PClk_DI()  (RCC->AHBENR &= ~(1<<18))
#define GPIOC_PClk_DI()  (RCC->AHBENR &= ~(1<<19))
#define GPIOD_PClk_DI()  (RCC->AHBENR &= ~(1<<20))
#define GPIOE_PClk_DI()  (RCC->AHBENR &= ~(1<<21))
#define GPIOF_PClk_DI()  (RCC->AHBENR &= ~(1<<22))

/*
 * CLOCK DISABLE MACROS SPIx PERIPHERAL
 */
#define SPI1_PClk_DI()  (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PClk_DI()  (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PClk_DI()  (RCC->APB1ENR &= ~(1<<15))
#define SPI4_PClk_DI()  (RCC->APB2ENR &= ~(1<<15))


/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 21)); (RCC->AHBRSTR &= ~(1 << 21)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); }while(0)


/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 15)); (RCC->APB2RSTR &= ~(1 << 15)); }while(0)

/*
 *  Macros function to select port code from port address
 */

#define PORT_ADDRESS_TO_CODE(x)     ((x == GPIOA)? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :0)


/*
 *  Macros for IRQ numbers (Position in Vector table)
 */

#define IRQ_NO_EXTI0 				 6
#define IRQ_NO_EXTI1 				 7
#define IRQ_NO_EXTI2 				 8
#define IRQ_NO_EXTI3 				 9
#define IRQ_NO_EXTI4 				10
#define IRQ_NO_EXTI0 				 6
#define IRQ_NO_EXTI9_5 			    23
#define IRQ_NO_EXTI15_10 			40


/*
 *  Macros for bit positions of SPI_CR1 register
 *
 */
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7



/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8




//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         RESET
#define FLAG_SET 			SET

#include "stm32f303RE_gpio_driver.h"
#include "stm32f303RE_SPI_driver.h"

#endif /* DRIVER_INC_STM32F303RE_H_ */

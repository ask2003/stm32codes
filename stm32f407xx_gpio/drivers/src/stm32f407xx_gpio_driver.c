/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 19, 2023
 *      Author: sumit
 */

#include "stm32f407xx_gpio_driver.h"

#include <stdint.h>

/*
 * Peripheral Clock control
 */

void GPIO_PeriClockControl (GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx==GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx==GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else{
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx==GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx==GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx==GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx==GPIOH){
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx==GPIOI){
			GPIOI_PCLK_DI();
		}
	}
} //First is Base address of which GPIO
//Second is Enable the clock or Disable the clock variable


/*
 * GPIO Initialization and De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	//1. CONFIGURE MODE OF GPIO PIN
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp=0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//EACH PIN TAKE 2 BITFIELDS IN THE MODE REGISTER, SO THE PIN NUMBER * 2 GIVES THE APPROPRIATE BITFIELD
		//NOW STORE THE ACTUAL VALUE IN THE REGISTER
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER|=temp;
	}

	else{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//CLEAR THE CORRESPONDING RTSR BIT IF IT HAS BEEN CONFIGURED PREVIOUSLY
			EXTI->EXTI_RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//CONFIGURE THE FTSR
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//CLEAR THE CORRESPONDING FTSR BIT IF IT HAS BEEN CONFIGURED PREVIOUSLY
			EXTI->EXTI_FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//CONFIGURE THE RTSR
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//CONFIGURE THE FTSR AND RTSR - SET BOTH RTSR AND FTSR
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. CONFIGURE THE GPIO PORT SELECTION IN SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->SYSCFG_EXTICR[temp1] = portcode<<(temp2 * 4);


		//3. ENABLE THE EXTI INTERRUPT DELIVERY USING IMR
		EXTI->EXTI_IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;

	//2. CONFIGURE THE SPEED

	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;

	temp=0;

	//3. CONFIGURE THE PULLUP & PULLDOWN SETTINGS

	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<=(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR|=temp;

	temp=0;

	//4. CONFIGURE THE OPTYPE

	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<=(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER|=temp;

	temp=0;

	//5. CONFIGURE THE ALTERNATE FUNCTIONALITY
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		//CONFIGURE THE ALTERNATE FUNCTION REGISTERS
		 uint8_t temp1,temp2;
		 temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		 temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;

		 pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<<(4*temp2));
		 pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4*temp2);
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	//THERE IS A REGISTER TO RESET THE GPIO USING RCC_AHB1RSTR
	//WE CAN ACCESS ITS BITFIELDS TO RESET A PARTICULAR GPIOx
	if(pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx==GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx==GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx==GPIOH){
		GPIOH_REG_RESET();
	}
	else if(pGPIOx==GPIOI){
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */
uint8_t volatile GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t volatile value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}

uint16_t volatile GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t volatile value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t volatile Value){
	if(Value==GPIO_PIN_SET){
		//WRITE 1 TO THE OUTPUT DATA REGISTER AT THE BIT FIELD TO THE CORRESPONDING PIN NUMBER
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else{
		//WRITE 1 TO THE OUTPUT DATA REGISTER AT THE BIT FIELD TO THE CORRESPONDING PIN NUMBER
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t volatile Value){
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ - Interrupt management and Configuration
 * Configure the IRQ number, like enabling and disabling
 * Priority etc..
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
		//******WHATERVER WE ARE CONFIGURING IN THIS API IS PROCESSOR SPECIFIC******//
		//******SO REFERE ARM CORTEX M4 GENERIC USER GUIDE. NOT STM32 REF MANUAL******//
		//******REFER PAGE NO. 219 IN THE USER GUIDE******//
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			//program ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			//program ISER1 register
			*NVIC_ISER1 |= (1<<IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			//program ISER2 register
			*NVIC_ISER2 |= (1<<IRQNumber % 64);
		}
	}
	else{
		if(IRQNumber <= 31){
			//program ICER0 register
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			//program ICER1 register
			*NVIC_ICER1 |= (1<<IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			//program ICER2 register
			*NVIC_ICER2 |= (1<<IRQNumber % 64);
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	//1. FIRST FIND-OUT THE CORRECT IPR REGISTER
	//THERE ARE 60 IPR REGS. RANGING FROM 0 TO 50 WHERE EACH IPR REGISTER HAS 4 INTERRUPT PRIORITY NUMBERS
	//SO WE NEED TO FIND THE CORRECT IPR REGISTER FROM THE IRQ NUMBER
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amt = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amt);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->EXTI_PR & (1<<PinNumber)){
		//clear
		EXTI->EXTI_PR |= (1<<PinNumber);
	}
}




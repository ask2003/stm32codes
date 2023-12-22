/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Dec 19, 2023
 *      Author: sumit
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include <stdint.h>
#include "stm32f407xx.h"

/*
 * THIS IS A CONFIGURATION STRUCTURE FOR A GPIO PIN
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * THIS IS A HANDLE STRUCTURE FOR A GPIO PIN
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx; //THIS HOLDS THE BASE ADDRESS OF THE GPIO PORT TO WHICH THE PIN BELONGS
	GPIO_PinConfig_t GPIO_PinConfig; //THIS HOLDS GPIO PIN CONFIGURATION SETTINGS
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4 //INPUT FALLING EDGE INTERRUPT
#define GPIO_MODE_IT_RT     5//INPUT RISING EDGE INTERRUPT
#define GPIO_MODE_IT_RFT    6	////INPUT FALLING EDGE RISING EDGE TRIGGER

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   0		//OUTPUT TYPE PUSH PULL
#define GPIO_OP_TYPE_OD   1		//OUTPUT TYPE OPEN DRAIN

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3

/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*
 * **************************APIs SUPPORTED BY THIS DRIVER**************************
 */

/*
 * Peripheral Clock control
 */
void GPIO_PeriClockControl (GPIO_RegDef_t *pGPIOx, uint8_t EnorDi); //First is Base address of which GPIO
//Second is Enable the clock or Disable the clock variable

/*
 * GPIO Initialization and De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t volatile GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t volatile GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t volatile Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t volatile Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/*
 * IRQ - Interrupt management and Configuration
 * Configure the IRQ number, like enabling and disabling
 * Priority etc..
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);





#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

/*
 * 005ext_button_ISR.c
 *
 *  Created on: Dec 21, 2023
 *      Author: sumit
 */

//CONNECT AN EXTERNAL BUTTON TO PD5 PIN AND TOGGLE THE LED WHENEVER INTERRUPT IS TRIGGERED BY THE PRESS
//INTERRUPT SHOULD BE TRIGGERED DURING FALLING EDGE OF THE BUTTON PRESS

#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_Handle_t GpioLed, GPIOBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));


	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);


	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GPIOBtn);

	//IRQ CONFIGURATIONS
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 30);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5 , ENABLE);

	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}

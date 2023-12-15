
#include <stdio.h>
#include <stdint.h>

#define RCC_BASE_ADDR 0x40023800UL
#define RCC_CFGR_REG_OFFSET 0x08UL
#define RCC_CFGR_REG_ADDR (RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)

#define GPIOA_BASE_ADDR 0x40020000UL

int main(void)
{
	uint32_t *pRccfgr = (uint32_t*)RCC_CFGR_REG_ADDR;

	/*MCO1: Microcontroller clock output 1
Set and cleared by software. Clock source selection may generate glitches on MCO1. It is
highly recommended to configure these bits only after reset before enabling the external
oscillators and PLL.
00: HSI clock selected
01: LSE oscillator selected
10: HSE oscillator clock selected
11: PLL clock selected
	 */

	//1. CONFIGURE THE RCC_CFGR MC01 BIT FIELDS TO SELECT HSI AS CLOCK SOURCE
	(*pRccfgr) &= ~(0x3<<21);   	//CLEAR 21 AND 22 BIT POSITIONS

	//CONFIGURE MC01 PRESCALAR
	/*
	MCO1PRE: MCO1 prescaler
	Set and cleared by software to configure the prescaler of the MCO1. Modification of this
	prescaler may generate glitches on MCO1. It is highly recommended to change this
	prescaler only after reset before enabling the external oscillators and the PLL.
	0xx: no division
	100: division by 2
	101: division by 3
	110: division by 4
	111: division by 5
	*/
	//page 165/1751
	//MISTAKE: MCO1 PRESCALAR BITS 26:24

	*pRccfgr |= (1<<25);
	*pRccfgr |= (1<<26);

	//2. CONFIGURE PA8 TO AF0 MODE TO BEHAVE AS MC01 SIGNAL

	//a) ENABLE THE PERIPHERAL CLOCK FOR GPIO A PERIPHERAL
	uint32_t *pRCCAhb1Enr = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCCAhb1Enr |= (1<<0);

	//b) CONFIGURE THE MODE OF GPIO PIN 8 AS ALTERNATE FUNCTION MODE
	uint32_t *pGPIOAModeReg = (uint32_t*)(GPIOA_BASE_ADDR+0x00);
	*pGPIOAModeReg &= ~(0x3<<16);
	*pGPIOAModeReg |= (0x2<<16);

	/*MODERy[1:0]: Port x configuration bits (y = 0..15)
These bits are written by software to configure the I/O direction mode.
00: Input (reset state)
01: General purpose output mode
10: Alternate function mode
11: Analog mode
*/

	//c) CONFIGURE THE ALTERNATION FUNCTION REGISTER TO SET THE MODE 0 FOR PA8
	/*
	8.4.10 GPIO alternate function high register (GPIOx_AFRH)
	(x = A..I/J)
	Address offset: 0x24
	Reset value: 0x0000 0000
	*/
	uint32_t *pGPIOA_Altfun = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);

	/*
	 * AFRHy: Alternate function selection for port x bit y (y = 8..15)
These bits are written by software to configure alternate function I/Os
AFRHy selection:
0000: AF0
0001: AF1
0010: AF2
0011: AF3
0100: AF4
0101: AF5
0110: AF6
0111: AF7
1000: AF8
1001: AF9
1010: AF10
1011: AF11
1100: AF12
1101: AF13
1110: AF14
1111: AF15
	 */
	*pGPIOA_Altfun &= ~(0xf<<0);


    for(;;);
}

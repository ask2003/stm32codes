
#include <stdio.h>
#include <stdint.h>
#define ADC_BASE_ADDR 0x40012000UL
#define ADC_CR1_REG_OFFSET 0x04UL
#define ADC_CR1_REG_ADDR (ADC_BASE_ADDR + ADC_CR1_REG_OFFSET)

#define RCC_BASE_ADDR 0x40023800UL
#define RCC_APB2_ENR_OFFSET 0x44UL
#define RCC_APB2_ENR_ADDR (RCC_BASE_ADDR + RCC_APB2_ENR_OFFSET)

int main(void)
{
	//ADC_CR1 IS CONNECTED TO APB2 BUS, SO ENABLE THE APB2 CLOCK IN RCC.
	//RCC APB2_ENR
    uint32_t *pClock = (uint32_t*)RCC_APB2_ENR_ADDR;
    uint32_t *pAddr = (uint32_t*)ADC_CR1_REG_ADDR;

    *pClock |= (1<<8);
    *pAddr |= (1<<8);




    for(;;);
}

#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "misc.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);


// 주변장치 클럭 부여
// UART(블루투스) GPIO, ADC
void RCC_Configure(void)
{  
    // TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
	
	/* USART1, USART2 TX/RX port clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // USART1 (PA9, PA10)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // USART2 (PD5, PD6)

	/* USART1, USART2 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART1은 APB2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART2는 APB1

	/* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}
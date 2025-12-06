#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "misc.h"

/* ================================================================
 * [사용자 설정] HX711 핀 & 보정 계수
 * ================================================================ */


// 예: 현재 400.0f -> (400 * 측정된값 / 실제무게)로 수정 필요
float Calibration_Factor = 400.0f; 
long Zero_Offset = 0;

/* 전역 변수 (디버깅 확인용) */
volatile float weight = 0.0f;
volatile long raw_data = 0;

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);

/* HX711 관련 함수 */
void HX711_Init_State(void); // 초기 핀 상태 설정
long HX711_Read(void);
long HX711_Read_Average(unsigned char times);
void HX711_Tare(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);


// 주변장치 클럭 부여
void RCC_Configure(void)
{  
	/* USART1, USART2 TX/RX port clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // USART1 (PA9, PA10)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // USART2 (PD5, PD6)

	/* HX711 clock enable*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // DATA - PB6, SCK - PB6

	/* USART1, USART2 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART1은 APB2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART2는 APB1

	/* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    /* USART1 pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-pull
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	//RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Input Floating
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 pin setting */
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-pull
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	//RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Input Floating
    GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* HX711 pin setting */
    // SCK (PB6) -> Output Push-Pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // DT (PB7) -> Input Pull-Up
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

	// Enable the USART1 peripheral
	USART_Cmd(USART1, ENABLE);
	
	// TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
	USART1_InitStructure.USART_BaudRate = 9600;
	USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART1_InitStructure.USART_StopBits = USART_StopBits_1;
	USART1_InitStructure.USART_Parity = USART_Parity_No;
	USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART1_InitStructure);
	
	// TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

	// Enable the USART2 peripheral
	USART_Cmd(USART2, ENABLE);
	
	// TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
	USART2_InitStructure.USART_BaudRate = 9600; // 
	USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART2_InitStructure.USART_StopBits = USART_StopBits_1;
	USART2_InitStructure.USART_Parity = USART_Parity_No;
	USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART2_InitStructure);
	
	// TODO: Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
	
    // TODO: fill the arg you want
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // USART1
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ================================================================
 * HX711 드라이버 함수 구현
 * ================================================================ */

void HX711_Init_State(void)
{
    // 초기 상태: SCK Low로 시작
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
}

long HX711_Read(void)
{
    long count = 0;
    unsigned char i;
    
    // SCK Low 확인
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    
    // 데이터 준비 대기
    while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7));

    // 24비트 데이터 읽기
    for (i = 0; i < 24; i++)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_6);
        Delay_us(1);
        count = count << 1;
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);
        Delay_us(1);
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)) count++;
    }

    // Gain 128 설정 (추가 1펄스)
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    Delay_us(1);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    Delay_us(1);

    count = count ^ 0x800000; 
    return count;
}

long HX711_Read_Average(unsigned char times)
{
    long sum = 0;
    for (unsigned char i = 0; i < times; i++) sum += HX711_Read();
    return sum / times;
}

void HX711_Tare(void)
{
    Zero_Offset = HX711_Read_Average(30);
}

void Delay_us(uint32_t us) { us *= 12; while(us--); }
void Delay_ms(uint32_t ms) { while(ms--) Delay_us(1000); }

/* ================================================================
 * 메인 함수
 * ================================================================ */
int main(void)
{
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    USART1_Init();      // pc
    USART2_Init();      // bluetooth
    NVIC_Configure();

    // HX711 초기 상태 설정 (SCK Low)
    HX711_Init_State();

    // 안정화 대기 (전원 인가 후 센서 안정화)
    Delay_ms(2000);

    // 영점 잡기 (Tare)
    HX711_Tare();

    while (1)
    {
        // 10회 평균 측정
        raw_data = HX711_Read_Average(10);

        // 무게 계산
        weight = (float)(raw_data - Zero_Offset) / Calibration_Factor;

        Delay_ms(200);
    }
}
#include "stm32f10x.h"
#include "stdio.h"
#include "string.h"
#include "misc.h"

/* =========================================================================
   [사용자 설정] 보드 자체 LED 핀 설정 (보드 회로도 확인 필요)
   ========================================================================= */
// 보통 STM32F107 보드의 경우 LED2는 PD3 또는 PD13 등에 연결됨
// 만약 동작하지 않으면 보드 매뉴얼을 보고 이 핀 번호를 수정하세요.
#define LED2_PORT GPIOD
#define LED2_PIN  GPIO_Pin_3 
#define LED2_RCC  RCC_APB2Periph_GPIOD

/* =========================================================================
   Function Prototypes
   ========================================================================= */
void System_Setup(void);
void RCC_Configure(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void USART2_Init(void);
void UART_SendString(char *str);
uint16_t Read_ADC_Channel(uint8_t channel);
void Delay_ms(uint32_t ms);

/* =========================================================================
   Main Function
   ========================================================================= */
int main(void)
{
    char msgBuffer[100];
    
    // 센서 값 저장 변수
    uint8_t valShock = 0;
    uint8_t valDoor = 0;
    uint8_t valSmoke = 0;
    uint8_t valFlame = 0;
    uint16_t adcPressure = 0;
    uint16_t adcTemp = 0;
    float realTemp = 0.0;
    
    // 트리거 여부 (1: 감지됨, 0: 정상)
    uint8_t isTriggered = 0;

    System_Setup(); 

    // 부팅 메시지
    UART_SendString("\r\n[DEBUG MODE] Sensor Check Start...\r\n");
    UART_SendString("Testing sensors one by one.\r\n");

    while (1)
    {
        isTriggered = 0; // 초기화

        // -------------------------------------------------------------
        // 1. 센서 값 읽기 (Polling 방식)
        // -------------------------------------------------------------
        
        // [Digital] 진동 (PB0) - High 감지 시
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_SET) valShock = 1;
        else valShock = 0;

        // [Digital] 문 (PB1) - High(열림) 감지 시
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_SET) valDoor = 1;
        else valDoor = 0;

        // [Digital] 연기 (PB12) - Low 감지 시 (모듈 특성)
        // 만약 평소에 High이고 연기 감지 시 Low라면 !GPIO_Read... 로 조건 반전
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == Bit_RESET) valSmoke = 1; 
        else valSmoke = 0;

        // [Digital] 불꽃 (PB13) - Low 감지 시
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == Bit_RESET) valFlame = 1;
        else valFlame = 0;

        // [Analog] 온도 (PA1)
        adcTemp = Read_ADC_Channel(ADC_Channel_1);
        realTemp = (float)adcTemp * 3300.0 / 4095.0 / 10.0; // 섭씨 변환

        // [Analog] 압력 (PA2)
        adcPressure = Read_ADC_Channel(ADC_Channel_2);

        // -------------------------------------------------------------
        // 2. 트리거 판단 (하나라도 감지되면 LED ON)
        // -------------------------------------------------------------
        
        // 기준값 설정 (테스트 환경에 맞춰 조정하세요)
        // 예: 온도가 30도 이상이거나, 압력이 2000 이상이면 트리거
        if (valShock == 1) isTriggered = 1;
        if (valDoor == 1)  isTriggered = 1;
        if (valSmoke == 1) isTriggered = 1;
        if (valFlame == 1) isTriggered = 1;
        if (realTemp > 30.0) isTriggered = 1;    // 손으로 쥐어서 온도 올리기 테스트
        if (adcPressure > 2000) isTriggered = 1; // 손으로 꾹 눌러보기 테스트

        // -------------------------------------------------------------
        // 3. LED 제어 및 결과 전송
        // -------------------------------------------------------------
        
        if (isTriggered) {
            // 감지됨 -> LED2 켜기 (Low Active인지 High Active인지 보드마다 다름)
            // 보통 보드 LED는 Low를 주면 켜지는 경우가 많음 (GPIOD, Pin3 기준)
            // 안 켜지면 Bit_SET으로 변경해보세요.
            GPIO_ResetBits(LED2_PORT, LED2_PIN); // LED ON (Low Active 가정)
            
            // 디버깅 메시지 생성 (감지된 상태 알림)
            sprintf(msgBuffer, "[DETECTED] ");
            UART_SendString(msgBuffer);
        } else {
            // 정상 -> LED2 끄기
            GPIO_SetBits(LED2_PORT, LED2_PIN);   // LED OFF
        }

        // 전체 센서 값 출력 (블루투스로 확인)
        // S:Shock, D:Door, SM:Smoke, F:Flame, T:Temp, P:Press
        sprintf(msgBuffer, "S:%d D:%d SM:%d F:%d T:%.1f P:%d\r\n", 
                valShock, valDoor, valSmoke, valFlame, realTemp, adcPressure);
        UART_SendString(msgBuffer);

        Delay_ms(500); // 0.5초 간격 반복
    }
}

/* =========================================================================
   Configuration Functions
   ========================================================================= */
void System_Setup(void) {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    USART2_Init();
}

void RCC_Configure(void) {
    // GPIO A, B, D 및 AFIO, ADC1, USART2 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO | 
                           RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    // LED 포트 클럭 활성화 (매크로 사용)
    RCC_APB2PeriphClockCmd(LED2_RCC, ENABLE);
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. [Output] 보드 자체 LED2
    GPIO_InitStructure.GPIO_Pin = LED2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED2_PORT, &GPIO_InitStructure);
    
    // 초기 상태: LED 끄기 (High가 끄는 것인지 Low가 끄는 것인지 보드 확인)
    GPIO_SetBits(LED2_PORT, LED2_PIN); 

    // 2. [Input] 디지털 센서들 (PB0, PB1, PB12, PB13)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 풀업
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // PB1 (Door) - Floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. [Analog] 아날로그 센서들 (PA1, PA2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. [Comm] Bluetooth (USART2) - PD5(TX), PD6(RX) & Remap
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); 
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

void USART2_Init(void) {
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(USART2, ENABLE);
    USART_InitStructure.USART_BaudRate = 9600; 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    // 폴링 방식이므로 인터럽트(NVIC) 설정 불필요
}

/* =========================================================================
   Helper Functions
   ========================================================================= */

void UART_SendString(char *str) {
    while (*str) {
        USART_SendData(USART2, *str++);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
}

uint16_t Read_ADC_Channel(uint8_t channel) {
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

void Delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for(i=0; i<ms; i++) for(j=0; j<5000; j++);
}

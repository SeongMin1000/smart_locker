#include "stm32f10x.h"
#include "lcd.h"
#include "stdio.h"
#include "string.h"
#include "misc.h"

/* =========================================================================
   Global Flags & Variables
   ========================================================================= */
volatile uint8_t g_SmokeEvent = 0;        // 연기 감지 플래그
volatile uint8_t g_FlameEvent = 0;        // 불꽃 감지 플래그
volatile uint8_t g_ShockEvent = 0;        // 진동 감지 플래그
volatile uint8_t g_DoorStatusChanged = 0; // 문 상태 변화 감지 플래그
volatile char    g_BluetoothCmd = 0;      // 블루투스 수신 명령어

/* =========================================================================
   Function Prototypes
   ========================================================================= */
void System_Setup(void);
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void NVIC_Configure(void);
void ADC_Configure(void);
void USART2_Init(void);            // 블루투스 연결용 (USART2)
void TIM4_PWM_Init(void);
void Servo_Control(uint8_t state); // 0:Unlock(0도), 1:Lock(90도)
void UART_SendString(char *str);   // 앱으로 전송 함수
uint16_t Read_ADC_Channel(uint8_t channel);
float Get_Temperature(void);
void Delay_ms(uint32_t ms);

/* =========================================================================
   Main Function
   ========================================================================= */
int main(void)
{
    char msgBuffer[50];
    uint16_t pressureVal = 0;
    float currentTemp = 0.0;
    
    uint8_t currentDoorState = 0; // 0: 닫힘, 1: 열림
    uint8_t isLocked = 0;         // 0: Unlocked, 1: Locked (초기값 열림)

    // 1. 시스템 초기화
    System_Setup(); 

    // 2. LCD 초기 화면 설정
    LCD_Clear(WHITE);
    LCD_ShowString(20, 20, "SMART LOCKER SYSTEM", BLACK, WHITE);
    
    // 3. 초기 상태: 잠금 해제 (0도)
    Servo_Control(0); 
    isLocked = 0;
    LCD_ShowString(20, 50, "Status: UNLOCKED  ", BLUE, WHITE);
    
    // 4. 앱으로 부팅 완료 알림
    UART_SendString("SYSTEM:BOOT_COMPLETE\r\n"); 

    while (1)
    {
        // =============================================================
        // [1] 화재 감시 (최우선 순위 - 즉시 개방)
        // =============================================================
        if (g_FlameEvent) {
            Servo_Control(0); // 화재 시 무조건 잠금 해제
            LCD_Clear(RED);
            LCD_ShowString(40, 100, "FIRE! FLAME DETECTED", YELLOW, RED);
            UART_SendString("EVENT:FIRE_FLAME\r\n"); 
            
            GPIO_SetBits(GPIOA, GPIO_Pin_3); // 부저 ON (계속 울림)
            while(1); // 시스템 정지 (전원 리셋 필요)
        }

        if (g_SmokeEvent) {
            LCD_ShowString(20, 80, "SMOKE DETECTED... ", MAGENTA, WHITE);
            UART_SendString("WARNING:SMOKE_DETECTED\r\n");
            
            // 2차 확인: 온도 센서 값 읽기
            currentTemp = Get_Temperature();
            sprintf(msgBuffer, "TEMP:%.1f", currentTemp);
            LCD_ShowString(20, 100, (u8*)msgBuffer, BLACK, WHITE);
            
            // 기준 온도(예: 45도) 초과 시 화재 확정
            if (currentTemp > 45.0) { 
                 Servo_Control(0); 
                 LCD_Clear(RED);
                 LCD_ShowString(40, 100, "FIRE! HIGH TEMP", YELLOW, RED);
                 UART_SendString("EVENT:FIRE_SMOKE_TEMP\r\n");
                 GPIO_SetBits(GPIOA, GPIO_Pin_3); // 부저 ON
                 while(1);
            } else {
                 // 연기는 났지만 온도가 낮으면 단순 경고 후 복귀
                 Delay_ms(2000);
                 LCD_ShowString(20, 80, "                  ", WHITE, WHITE);
                 g_SmokeEvent = 0; 
            }
        }

        // =============================================================
        // [2] 앱(블루투스) 명령 처리
        // =============================================================
        if (g_BluetoothCmd != 0) {
            if (g_BluetoothCmd == 'L' || g_BluetoothCmd == 'l') {
                // [Lock 명령]
                LCD_ShowString(20, 50, "Status: LOCKED    ", RED, WHITE);
                Servo_Control(1); 
                isLocked = 1;
                UART_SendString("CMD_ACK:LOCKED\r\n");
            }
            else if (g_BluetoothCmd == 'O' || g_BluetoothCmd == 'o') {
                // [Unlock 명령]
                LCD_ShowString(20, 50, "Status: UNLOCKED  ", BLUE, WHITE);
                Servo_Control(0); 
                isLocked = 0;
                UART_SendString("CMD_ACK:UNLOCKED\r\n");
            }
            g_BluetoothCmd = 0; // 명령 처리 완료
        }

        // =============================================================
        // [3] 문 상태 감시 (리드 스위치)
        // =============================================================
        if (g_DoorStatusChanged) {
            // 리드 스위치 읽기 (자석 멀어짐=High=열림)
            if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_SET) {
                currentDoorState = 1; 
            } else {
                currentDoorState = 0; 
            }

            if (currentDoorState == 1) { 
                // [문 열림]
                LCD_ShowString(20, 140, "DOOR: OPEN        ", RED, WHITE);
                UART_SendString("STATE:DOOR_OPEN\r\n");
                
                // 잠금 상태인데 문이 열리면 '침입' 경고
                if (isLocked == 1) {
                    UART_SendString("ALARM:FORCED_ENTRY\r\n");
                    GPIO_SetBits(GPIOA, GPIO_Pin_3); // 부저 경고
                    Delay_ms(1000);
                    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
                }
            } 
            else { 
                // [문 닫힘]
                LCD_ShowString(20, 140, "DOOR: CLOSED      ", BLUE, WHITE);
                UART_SendString("STATE:DOOR_CLOSED\r\n");

                // 물품 감지 (압력 센서)
                pressureVal = Read_ADC_Channel(ADC_Channel_2);
                if (pressureVal > 1000) { 
                     UART_SendString("INFO:ITEM_INSIDE\r\n");
                     LCD_ShowString(20, 160, "ITEM: O      ", BLACK, WHITE);
                } else {
                     UART_SendString("INFO:EMPTY      \r\n");
                     LCD_ShowString(20, 160, "ITEM: X      ", BLACK, WHITE);
                }
            }
            g_DoorStatusChanged = 0; 
        }

        // =============================================================
        // [4] 진동(충격) 감지
        // =============================================================
        if (g_ShockEvent) {
            LCD_ShowString(20, 120, "SHOCK DETECTED!   ", RED, WHITE);
            UART_SendString("EVENT:SHOCK_DETECTED\r\n");
            
            // 경고음 3회
            for(int i=0; i<3; i++) {
                GPIO_SetBits(GPIOA, GPIO_Pin_3); Delay_ms(100);
                GPIO_ResetBits(GPIOA, GPIO_Pin_3); Delay_ms(100);
            }
            g_ShockEvent = 0;
            LCD_ShowString(20, 120, "                  ", WHITE, WHITE);
        }
    }
}

/* =========================================================================
   Configuration Functions
   ========================================================================= */
void System_Setup(void) {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    EXTI_Configure();
    NVIC_Configure();
    ADC_Configure();
    USART2_Init(); // 블루투스
    TIM4_PWM_Init();
    LCD_Init();
}

void RCC_Configure(void) {
    // GPIO A, B, D 및 AFIO, ADC1, TIM4, USART2 클럭 활성화
    // USART2는 APB1, 나머지는 APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM4, ENABLE);
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. Buzzer (PA3) - Output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 2. Sensors Input (PB0, PB12, PB13) - IPU
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 3. Door Input (PB1) - Floating (모듈 특성 반영)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 4. Analog Inputs (PA1: Temp, PA2: Pressure)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 5. Bluetooth (USART2) - PD5(TX), PD6(RX) & Remap
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); // [중요] Remap
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 6. Servo Motor (PB6) - AF_PP
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void EXTI_Configure(void) {
    EXTI_InitTypeDef EXTI_InitStructure;

    // PB0 (Shock)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // PB1 (Door)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
    EXTI_Init(&EXTI_InitStructure);

    // PB12 (Smoke)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_Init(&EXTI_InitStructure);
    
    // PB13 (Flame)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_Init(&EXTI_InitStructure);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 우선순위 설정 (화재 > 진동 > 통신 > 문)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; // Smoke & Flame
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; // Shock
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; // Bluetooth
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; // Door
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_Init(&NVIC_InitStructure);
}

void USART2_Init(void) { // Bluetooth 연결
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(USART2, ENABLE);
    USART_InitStructure.USART_BaudRate = 9600; 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
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

void TIM4_PWM_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_Cmd(TIM4, ENABLE);
}

/* =========================================================================
   Helper Functions & Interrupts
   ========================================================================= */
void Servo_Control(uint8_t state) {
    if(state == 1) TIM_SetCompare1(TIM4, 1500); // 90도 (잠금)
    else TIM_SetCompare1(TIM4, 600);            // 0도 (열림)
}

// 블루투스(App)로 전송 - USART2 사용
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

float Get_Temperature(void) {
    uint16_t adcValue = Read_ADC_Channel(ADC_Channel_1);
    float voltage_mv = (float)adcValue * 3300.0 / 4095.0;
    return voltage_mv / 10.0;
}

void Delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for(i=0; i<ms; i++) for(j=0; j<5000; j++);
}

// --- Interrupt Handlers ---

void EXTI0_IRQHandler(void) { // Shock
    if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
        g_ShockEvent = 1;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

void EXTI1_IRQHandler(void) { // Door
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
        g_DoorStatusChanged = 1;
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void EXTI15_10_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line12) != RESET) { // Smoke
        g_SmokeEvent = 1;
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
    if(EXTI_GetITStatus(EXTI_Line13) != RESET) { // Flame
        g_FlameEvent = 1;
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}

// [블루투스 수신 핸들러] - USART2
void USART2_IRQHandler(void) {
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint16_t rcv = USART_ReceiveData(USART2);
        g_BluetoothCmd = (char)rcv; // 메인 루프로 전달
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

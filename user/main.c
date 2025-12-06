#include "stm32f10x.h"
#include "lcd.h"
#include "stdio.h"
#include "string.h"

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
void USART1_Init(void);
void TIM4_PWM_Init(void);
void Servo_Control(uint8_t state); // 0:Unlock(0도), 1:Lock(90도)
void UART_SendString(char *str);
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
    
    // 현재 문 상태 (0: 닫힘, 1: 열림)
    uint8_t currentDoorState = 0; 
    
    // 현재 잠금 장치 상태 (0: Unlocked, 1: Locked) - 초기값 열림
    uint8_t isLocked = 0; 

    // 시스템 및 하드웨어 초기화
    System_Setup(); 

    // LCD 초기 화면
    LCD_Clear(WHITE);
    LCD_ShowString(20, 20, "SMART LOCKER SYSTEM", BLACK, WHITE);
    
    // 초기화: 서보모터 0도 (열림 상태)
    Servo_Control(0); 
    isLocked = 0;
    LCD_ShowString(20, 50, "Status: UNLOCKED  ", BLUE, WHITE);
    UART_SendString("SYSTEM:BOOT_UNLOCKED\r\n");

    while (1)
    {
        // -------------------------------------------------------------
        // 1. 화재 감시 시나리오 (최우선 순위 - 즉시 개방)
        // -------------------------------------------------------------
        if (g_FlameEvent) {
            Servo_Control(0); // 화재 시 무조건 잠금 해제(0도)
            LCD_Clear(RED);
            LCD_ShowString(40, 100, "FIRE! FLAME DETECTED", YELLOW, RED);
            UART_SendString("EVENT:FIRE_FLAME\r\n"); 
            
            GPIO_SetBits(GPIOA, GPIO_Pin_3); // 부저 ON (계속 울림)
            while(1); // 시스템 정지 (리셋 필요)
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
                 Servo_Control(0); // 잠금 해제
                 LCD_Clear(RED);
                 LCD_ShowString(40, 100, "FIRE! HIGH TEMP", YELLOW, RED);
                 UART_SendString("EVENT:FIRE_SMOKE_TEMP\r\n");
                 GPIO_SetBits(GPIOA, GPIO_Pin_3); 
                 while(1);
            } else {
                 // 연기는 감지되었으나 온도가 낮으면 단순 경고 후 복귀
                 Delay_ms(2000);
                 LCD_ShowString(20, 80, "                  ", WHITE, WHITE);
                 g_SmokeEvent = 0; 
            }
        }

        // -------------------------------------------------------------
        // 2. 앱(블루투스) 명령 처리 (수동 잠금/해제)
        // -------------------------------------------------------------
        if (g_BluetoothCmd != 0) {
            if (g_BluetoothCmd == 'L' || g_BluetoothCmd == 'l') {
                // [Lock 명령] -> 90도 회전
                LCD_ShowString(20, 50, "Status: LOCKED    ", RED, WHITE);
                Servo_Control(1); 
                isLocked = 1;
                UART_SendString("CMD_ACK:LOCKED\r\n");
            }
            else if (g_BluetoothCmd == 'O' || g_BluetoothCmd == 'o') {
                // [Unlock 명령] -> 0도 회전
                LCD_ShowString(20, 50, "Status: UNLOCKED  ", BLUE, WHITE);
                Servo_Control(0); 
                isLocked = 0;
                UART_SendString("CMD_ACK:UNLOCKED\r\n");
            }
            g_BluetoothCmd = 0; // 명령 처리 완료, 플래그 초기화
        }

        // -------------------------------------------------------------
        // 3. 문 상태 감시 (리드 스위치) - 알림 전용
        // -------------------------------------------------------------
        if (g_DoorStatusChanged) {
            // 핀 상태 읽기 (SZH-SSBH-040: 자석 멀어짐=High=열림)
            if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_SET) {
                currentDoorState = 1; // 열림
            } else {
                currentDoorState = 0; // 닫힘
            }

            if (currentDoorState == 1) { 
                // [문 열림]
                LCD_ShowString(20, 140, "DOOR: OPEN        ", RED, WHITE);
                UART_SendString("STATE:DOOR_OPEN\r\n"); // 앱에 알림
                
                // 만약 잠금(Locked) 상태인데 문이 열렸다면? (침입 상황)
                if (isLocked == 1) {
                    UART_SendString("ALARM:FORCED_ENTRY\r\n");
                    // 부저 경고 1초
                    GPIO_SetBits(GPIOA, GPIO_Pin_3); 
                    Delay_ms(1000);
                    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
                }
            } 
            else { 
                // [문 닫힘]
                LCD_ShowString(20, 140, "DOOR: CLOSED      ", BLUE, WHITE);
                UART_SendString("STATE:DOOR_CLOSED\r\n");

                // 물품 감지 (압력 센서 확인)
                pressureVal = Read_ADC_Channel(ADC_Channel_2);
                if (pressureVal > 1000) { // 기준값(실험 필요)
                     UART_SendString("INFO:ITEM_INSIDE\r\n");
                     LCD_ShowString(20, 160, "ITEM: O      ", BLACK, WHITE);
                } else {
                     UART_SendString("INFO:EMPTY      \r\n");
                     LCD_ShowString(20, 160, "ITEM: X      ", BLACK, WHITE);
                }
            }
            g_DoorStatusChanged = 0; // 처리 완료
        }

        // -------------------------------------------------------------
        // 4. 진동(충격) 감지
        // -------------------------------------------------------------
        if (g_ShockEvent) {
            LCD_ShowString(20, 120, "SHOCK DETECTED!   ", RED, WHITE);
            UART_SendString("EVENT:SHOCK_DETECTED\r\n");
            
            // 경고음 3회 삑-삑-삑
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
   Initialization Functions
   ========================================================================= */
void System_Setup(void) {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    EXTI_Configure();
    NVIC_Configure();
    ADC_Configure();
    USART1_Init();
    TIM4_PWM_Init();
    LCD_Init();
}

void RCC_Configure(void) {
    // GPIO A, B 및 AFIO, ADC1, USART1, TIM4 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1 |
                           RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. Output Pins (Buzzer: PA3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 2. Input Pins for Interrupts
    // PB0(Shock): IPU or IPD (센서 특성 확인, 보통 IPU 무난)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // PB1(Door - Reed Switch): IN_FLOATING (모듈이 0/1 출력함)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB12(Smoke), PB13(Flame): IPU
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. Analog Inputs (PA1: Temp, PA2: Pressure)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 4. USART1 (PA9: TX, PA10: RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 5. Servo Motor (PB6 - TIM4_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void EXTI_Configure(void) {
    EXTI_InitTypeDef EXTI_InitStructure;

    // PB0 (Shock) -> Line 0 (Rising Edge)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // PB1 (Door) -> Line 1 (Both Edges: 열림/닫힘 모두 감지)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
    EXTI_Init(&EXTI_InitStructure);

    // PB12 (Smoke) -> Line 12 (Falling Edge 가정)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_Init(&EXTI_InitStructure);
    
    // PB13 (Flame) -> Line 13 (Falling Edge 가정)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_Init(&EXTI_InitStructure);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 우선순위: 화재(0) > 진동(1) > 문(2) > 통신(3)
    
    // EXTI15_10 (Smoke & Flame)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // EXTI0 (Shock)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    // EXTI1 (Door)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
    
    // USART1 (Bluetooth)
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_Init(&NVIC_InitStructure);
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
    
    // Calibration
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

void USART1_Init(void) {
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600; // HC-06 Default
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 수신 인터럽트 활성화
    USART_Cmd(USART1, ENABLE);
}

void TIM4_PWM_Init(void) {
    // Servo Motor PWM (50Hz)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 72MHz / 72 = 1MHz count (1us)
    // Period 20000 -> 20ms (50Hz)
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500; // Init (Not critical here, controlled by main)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);

    TIM_Cmd(TIM4, ENABLE);
}

/* =========================================================================
   Helper Functions & Interrupt Handlers
   ========================================================================= */

// [Servo Control]
// state 0: Unlock (0도) -> Pulse 600 (약 0.6ms) - 막대기 원위치
// state 1: Lock (90도)  -> Pulse 1500 (1.5ms) - 막대기 회전하여 걸림
void Servo_Control(uint8_t state) {
    if(state == 1) {
        TIM_SetCompare1(TIM4, 1500); // 90도 (잠금)
    } else {
        TIM_SetCompare1(TIM4, 600);  // 0도 (열림) - 각도 미세조정 필요 시 이 값 변경
    }
}

// [UART Send]
void UART_SendString(char *str) {
    while (*str) {
        USART_SendData(USART1, *str++);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

// [ADC Read]
uint16_t Read_ADC_Channel(uint8_t channel) {
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

// [Temperature Calc]
float Get_Temperature(void) {
    uint16_t adcValue = Read_ADC_Channel(ADC_Channel_1); // PA1
    // 3.3V 기준, 12bit
    float voltage_mv = (float)adcValue * 3300.0 / 4095.0;
    return voltage_mv / 10.0; // 10mV per 1 Degree
}

void Delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for(i=0; i<ms; i++)
        for(j=0; j<5000; j++);
}

// ---------------- Interrupt Service Routines ----------------

// 1. 진동 감지 (PB0)
void EXTI0_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
        g_ShockEvent = 1;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

// 2. 문 상태 변화 (PB1)
void EXTI1_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
        g_DoorStatusChanged = 1;
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

// 3. 연기 & 불꽃 (PB12, PB13)
void EXTI15_10_IRQHandler(void) {
    // Smoke (PB12)
    if(EXTI_GetITStatus(EXTI_Line12) != RESET) {
        g_SmokeEvent = 1;
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
    // Flame (PB13)
    if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
        g_FlameEvent = 1;
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}

// 4. 블루투스 수신 (PA10)
void USART1_IRQHandler(void) {
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        // 받은 문자 하나를 전역 변수에 저장 (메인 루프에서 처리)
        g_BluetoothCmd = USART_ReceiveData(USART1);
    }
}

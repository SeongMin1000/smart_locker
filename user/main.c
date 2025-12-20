#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include <string.h>
#include <stdio.h>

#include "misc.h"

/* ================================================================
 * [전역 변수 선언부] - 모든 플래그 및 데이터 변수 통일
 * ================================================================ */

// 1. 통신 관련
#define MAX_RX_BUF 64
#define ABS(x) ((x) < 0 ? -(x) : (x))
#define VIB_THRESHOLD_5SEC  500

// [main 함수 밖 전역 변수 선언]
volatile uint8_t g_theft_msg_sent = 0; // 도난 메시지 보냈는지 체크
volatile uint8_t g_flame_msg_sent = 0; // 불꽃 메시지 보냈는지 체크
uint8_t g_msg_sent_moving = 0;

volatile char g_bt_rx_buffer[MAX_RX_BUF]; // 수신 버퍼
volatile uint8_t g_bt_rx_index = 0;       // 버퍼 인덱스
volatile uint8_t g_bt_data_ready = 0;     // 수신 완료 플래그

// 2. 타이머 및 제어 플래그
volatile uint8_t g_timer_sec_count = 0;   // 초 카운트
volatile uint8_t g_send_temp_flag = 1;    // 5초 주기 전송 플래그
volatile uint16_t g_vibration_count = 0;  // 5초간 진동 횟수 누적

// [전역 변수 추가]
volatile uint8_t g_flame_detected_flag = 0;

// 3. 센서 데이터 및 디버깅용 (Live Watch 확인용)
volatile uint8_t g_debug_temp = 0;        // 온도
volatile uint8_t g_debug_humi = 0;        // 습도
volatile uint8_t g_debug_flame = 1;       // 불꽃 (1:정상, 0:감지)
volatile uint8_t g_debug_vibration = 0;   // 진동 디버깅 (1:감지됨)
volatile uint8_t g_door_debug_val = 0;    // 리드 스위치 (0:닫힘, 1:열림)

volatile long g_debug_current_weight = 0; // 현재 로드셀 값
volatile long g_debug_ref_weight = 0;     // 기준 로드셀 값 (잠금 시점)
volatile long g_debug_weight_diff = 0;    // 무게 차이 (절대값)

// 4. 로드셀 설정값
const long EMPTY_WEIGHT = 826000; // 캘리브레이션 값
const long THRESHOLD = 50000;    // 도난 판단 임계값

// 5. 시스템 상태
typedef enum {
    STATE_UNLOCKED = 0,
    STATE_LOCKED = 1
} SafeState;

// volatile을 붙여 최적화 방지 (전역 변수 직접 제어)
volatile SafeState current_state = STATE_UNLOCKED;

/* ================================================================
 * 함수 프로토타입
 * ================================================================ */
void RCC_Configure(void);
void GPIO_Configure(void);
void TIM_Configure(void); 
void USART1_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);
void TIM4_Configure(void);
void EXTI_Configure(void);

void Servo_Write(uint16_t pulse); 
void HX711_Init_State(void); 
long HX711_Read(void);
long HX711_Read_Average(unsigned char times);
void HX711_Tare(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);

void DHT11_GPIO_Output(void);
void DHT11_GPIO_Input(void);
uint8_t DHT11_Read_Data(volatile uint8_t *temp, volatile uint8_t *humi); 
void DHT11_Delay_us(uint32_t us);

void USART2_SendString(char* str);


/* ================================================================
 * 하드웨어 설정 함수들
 * ================================================================ */

void RCC_Configure(void)
{  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                          RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                          RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
   
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM3 | 
                          RCC_APB1Periph_TIM4, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // USART1 TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // USART1 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 부저 (PA3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_3); // 초기값 OFF

    // USART2 TX (PD5 - Remap)
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    // USART2 RX (PD6 - Remap)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // HX711 SCK (PB6)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // HX711 DT (PB7)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 서보모터 (PB0)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
   
    // 불꽃 감지 (PD3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 리드 스위치 (PC3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 진동 센서 (PC1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 모듈 종류에 따라 IPU/IPD 변경 필요 가능성 있음
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void TIM_Configure(void) // 서보모터용 TIM3
{
    TIM_TimeBaseInitTypeDef TIM3_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    uint16_t prescale = (uint16_t) (SystemCoreClock / 1000000) - 1; 
    
    TIM3_InitStructure.TIM_Period = 20000 - 1; 
    TIM3_InitStructure.TIM_Prescaler = prescale;
    TIM3_InitStructure.TIM_ClockDivision = 0;
    TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500; 
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void TIM4_Configure(void) // 1초 타이머
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStructure.TIM_Period = 10000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
    TIM_Cmd(TIM4, ENABLE);
}

void USART1_Init(void) { /* PC 연결용 (생략가능하나 유지) */
    USART_InitTypeDef USART_InitStr;
    USART_Cmd(USART1, ENABLE);
    USART_InitStr.USART_BaudRate = 9600;
    USART_InitStr.USART_WordLength = USART_WordLength_8b;
    USART_InitStr.USART_StopBits = USART_StopBits_1;
    USART_InitStr.USART_Parity = USART_Parity_No;
    USART_InitStr.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStr.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStr);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void) { /* 블루투스용 */
    USART_InitTypeDef USART_InitStr;
    USART_Cmd(USART2, ENABLE);
    USART_InitStr.USART_BaudRate = 9600; 
    USART_InitStr.USART_WordLength = USART_WordLength_8b;
    USART_InitStr.USART_StopBits = USART_StopBits_1;
    USART_InitStr.USART_Parity = USART_Parity_No;
    USART_InitStr.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStr.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStr);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void EXTI_Configure(void) { // 불꽃(PD3), 진동(PC1) 인터럽트 설정
    EXTI_InitTypeDef EXTI_InitStructure;

    // 불꽃 (Line 3)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3); 
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 진동 (Line 1)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1); 
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 불꽃
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 진동
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ================================================================
 * 기타 드라이버 함수들
 * ================================================================ */
void Servo_Write(uint16_t pulse) {
    if(pulse < 500) pulse = 500;
    if(pulse > 2500) pulse = 2500;
    TIM_SetCompare3(TIM3, pulse);
}

void HX711_Init_State(void) {
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    GPIO_SetBits(GPIOB, GPIO_Pin_6);   
    Delay_ms(10);                      
    GPIO_ResetBits(GPIOB, GPIO_Pin_6); 
    Delay_ms(10);
}
long HX711_Read(void) {
    long count = 0;
    unsigned char i;
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    __disable_irq(); 
    for (i = 0; i < 24; i++) {
        GPIO_SetBits(GPIOB, GPIO_Pin_6); 
        Delay_us(1); 
        count = count << 1;
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)) count++;
        GPIO_ResetBits(GPIOB, GPIO_Pin_6); 
        Delay_us(1);
    }
    GPIO_SetBits(GPIOB, GPIO_Pin_6); Delay_us(1);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6); Delay_us(1);
    __enable_irq(); 
    count = count ^ 0x800000; 
    return count;
}
long HX711_Read_Average(unsigned char times) {
    long sum = 0;
    for (unsigned char i = 0; i < times; i++) sum += HX711_Read();
    return sum / times;
}
void HX711_Tare(void) { HX711_Read_Average(30); }

void DHT11_Delay_us(uint32_t us) { us *= 8; while(us--) { __NOP(); } }
void DHT11_GPIO_Output(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
void DHT11_GPIO_Input(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
uint8_t DHT11_Read_Data(volatile uint8_t *temp, volatile uint8_t *humi) {
    uint8_t data[5] = {0,};
    uint8_t i, j;
    DHT11_GPIO_Output();
    GPIO_ResetBits(GPIOC, GPIO_Pin_2); Delay_ms(20); 
    GPIO_SetBits(GPIOC, GPIO_Pin_2); DHT11_Delay_us(30);
    DHT11_GPIO_Input(); DHT11_Delay_us(10);
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1) return 0; 
    while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 0); 
    while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1); 
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 0); 
            DHT11_Delay_us(40); 
            if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1) {
                data[i] |= (1 << (7 - j));
                while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1); 
            }
        }
    }
    if ((data[0] + data[1] + data[2] + data[3]) == data[4]) {
        *humi = data[0]; *temp = data[2]; return 1;
    }
    return 0;
}

void USART2_SendString(char* str) {
    while (*str) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, *str++);
    }
}
void Delay_us(uint32_t us) { us *= 12; while(us--); }
void Delay_ms(uint32_t ms) { while(ms--) Delay_us(1000); }

/* ================================================================
 * 인터럽트 핸들러 (전역 플래그 제어)
 * ================================================================ */

void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        char ch = USART_ReceiveData(USART2);
        // ECHO
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
        USART_SendData(USART1, ch); 

        if (ch == '\n' || ch == '\r') {
            if (g_bt_rx_index > 0) {
                g_bt_rx_buffer[g_bt_rx_index] = '\0'; 
                g_bt_data_ready = 1;                  
                g_bt_rx_index = 0;                    
            }
        } else {
            if (g_bt_rx_index < MAX_RX_BUF - 1) g_bt_rx_buffer[g_bt_rx_index++] = ch;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

void TIM4_IRQHandler(void) { // 5초 주기
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        g_timer_sec_count++;
        if (g_timer_sec_count >= 5) {
            g_timer_sec_count = 0;
            g_send_temp_flag = 1; // 전역 플래그 ON
        }
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

void EXTI3_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        g_flame_detected_flag = 1; // 플래그만 세움
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

void EXTI1_IRQHandler(void) { // 진동 감지
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        
        // 잠겨있을 때만 진동 횟수 누적 (열고 닫을 땐 무시)
        if (current_state == STATE_LOCKED) {
            g_vibration_count++; 
        }
        EXTI_ClearITPendingBit(EXTI_Line1); 
    }
}

/* ================================================================
 * MAIN
 * ================================================================ */
int main(void)
{
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    EXTI_Configure(); // EXTI 설정 함수 호출
    
    USART1_Init();      
    USART2_Init();
    TIM4_Configure(); 
    NVIC_Configure();
    TIM_Configure();

    HX711_Init_State();
    Delay_ms(2000);
    HX711_Tare();

    Servo_Write(1500); // 0도 (열림)
    current_state = STATE_UNLOCKED;

    while (1)
    {
        // 1. 센서 값 읽어서 전역 변수에 상시 갱신
        g_door_debug_val = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
        g_debug_flame = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3); // 1:정상, 0:화재
        g_debug_current_weight = HX711_Read(); 

        // 2. 블루투스 명령 처리
        if (g_bt_data_ready)
        {
            g_bt_data_ready = 0; 

            if (strcmp((char*)g_bt_rx_buffer, "CMD:LOCK") == 0)
            {
                g_debug_ref_weight = g_debug_current_weight; 
                Servo_Write(2400); 
                current_state = STATE_LOCKED;
                
                // 잠금 시 모든 경고 플래그 초기화
                g_theft_msg_sent = 0;
                g_msg_sent_moving = 0; 
                
                char msg[64];
                sprintf(msg, "OK: LOCKED (Ref: %ld)\r\n", g_debug_ref_weight);
                USART2_SendString(msg);
            }
            else if (strcmp((char*)g_bt_rx_buffer, "CMD:UNLOCK") == 0)
            {
                Servo_Write(1500); 
                current_state = STATE_UNLOCKED;
                
                // 해제 시 경고 플래그 초기화
                g_theft_msg_sent = 0;
                g_msg_sent_moving = 0;

                USART2_SendString("OK: UNLOCKED\r\n");
            }
            memset((void*)g_bt_rx_buffer, 0, MAX_RX_BUF);
        }

        // 3. 5초마다 상태 전송 및 진동 감지
        if (g_send_temp_flag == 1)
        {
            DHT11_Read_Data(&g_debug_temp, &g_debug_humi);
            
            char combined_msg[64];
            sprintf(combined_msg, "T=%d\n", g_debug_temp);
            USART2_SendString(combined_msg);
            
            g_send_temp_flag = 0;

            // [수정] 진동 감지 (플래그 적용)
            if (current_state == STATE_LOCKED)
            {
                if (g_vibration_count > VIB_THRESHOLD_5SEC)
                {
                    // 아직 메시지를 안 보냈을 때만 실행
                    if (g_msg_sent_moving == 0) 
                    {
                        USART2_SendString("WARNING: MOVING DETECTED! (Carrying)\r\n");

                        for(int i=0; i<5; i++) {
                            GPIO_ResetBits(GPIOA, GPIO_Pin_3); 
                            Delay_ms(100);
                            GPIO_SetBits(GPIOA, GPIO_Pin_3);   
                            Delay_ms(100);
                        }
                        g_msg_sent_moving = 1; // 보냄 표시
                    }
                }
                else 
                {
                    // 진동이 멈추면(임계값 이하) 플래그 초기화하여 다시 경고 가능하게 함
                    g_msg_sent_moving = 0;
                }
            }

            g_vibration_count = 0; // 카운트 리셋
        }

        // 4. 불꽃 감지 처리 (플래그 적용)
        // 인터럽트가 발생했거나(g_flame_detected_flag), 현재 센서가 0(화재)인 경우
        if (g_flame_detected_flag == 1 || GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == 0) 
        {
            if (g_flame_msg_sent == 0) // 아직 메시지 안 보냄
            {
                USART2_SendString("WARNING: FLAME DETECTED!\r\n");
                
                for(int i = 0; i < 5; i++) {
                    GPIO_ResetBits(GPIOA, GPIO_Pin_3); 
                    Delay_ms(100);                     
                    GPIO_SetBits(GPIOA, GPIO_Pin_3);   
                    Delay_ms(100);       
                }
                g_flame_msg_sent = 1; // 보냄 표시 (잠금)
            }
            g_flame_detected_flag = 0;
        }
        else 
        {
            // 불꽃 센서가 다시 정상(1)으로 돌아오면 플래그 해제
            if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == 1) {
                g_flame_msg_sent = 0;
            }
        }

        // 5. 도난 감지 로직 (플래그 적용)
        if (current_state == STATE_LOCKED)
        {
            g_debug_weight_diff = ABS(g_debug_ref_weight - g_debug_current_weight);

            if (g_debug_weight_diff > THRESHOLD)
            {
                // 아직 메시지를 안 보냈을 때만 실행
                if (g_theft_msg_sent == 0)
                {
                    for(int i = 0; i < 5; i++) {
                        GPIO_ResetBits(GPIOA, GPIO_Pin_3); 
                        Delay_ms(100);                     
                        GPIO_SetBits(GPIOA, GPIO_Pin_3);   
                        Delay_ms(100);       
                    }
                    USART2_SendString("WARNING: THEFT DETECTED!\r\n");
                    
                    g_theft_msg_sent = 1; // 보냄 표시 (잠금)
                }
            }
            else
            {
                // 무게가 다시 정상 범위로 돌아오면(훔쳤다가 다시 놓으면) 재감지 가능하도록 리셋
                g_theft_msg_sent = 0;
            }
        }
        else
        {
            // 잠금 해제 상태면 플래그 리셋
            g_theft_msg_sent = 0;
        }
        
        Delay_ms(100); 
    }

}

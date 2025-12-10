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
 * UART2(블루투스) 관련 변수
 * ================================================================ */
#define MAX_RX_BUF 64
volatile char g_bt_rx_buffer[MAX_RX_BUF]; // 수신 버퍼
volatile uint8_t g_bt_rx_index = 0;       // 버퍼 인덱스
volatile uint8_t g_bt_data_ready = 0;     // 수신 완료 플래그 (1이면 데이터 있음)

// [추가] 5초 타이머 및 DMA 전송용 변수
volatile uint8_t g_timer_sec_count = 0; // 초 카운트
volatile uint8_t g_send_temp_flag = 1;  // 5초 전송 플래그

/* ================================================================
 * 전역 변수
 * ================================================================ */

// 상태 관리용 enum
typedef enum {
    STATE_UNLOCKED = 0,
    STATE_LOCKED = 1
} SafeState;
SafeState current_state = STATE_UNLOCKED;

// 로드셀 관련 변수
long reference_weight = 0; // 잠금 시점의 무게 (물건 있음)
long current_weight = 0;   // 실시간 무게
const long EMPTY_WEIGHT = 826000; // 아무것도 없을 때의 기본값 (보정 필요)
const long THRESHOLD = 10000;      // 오차 범위 (센서 노이즈 감안)

uint8_t servo_state = 0; // 0: 닫힘, 1: 열림

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void TIM_Configure(void); // [추가] PWM 타이머 설정 함수
void USART1_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);

/* 서보모터 제어 함수 [추가] */
void Servo_Write(uint16_t pulse); 

/* HX711 관련 함수 */
void HX711_Init_State(void); 
long HX711_Read(void);
long HX711_Read_Average(unsigned char times);
void HX711_Tare(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);

/* DHT11 관련 함수 */
void DHT11_GPIO_Output(void);
void DHT11_GPIO_Input(void);
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi);
void DHT11_Delay_us(uint32_t us);

/* 블루투스 함수*/
void USART2_SendString(char* str);
void USART2_IRQHandler(void);
void Process_Bluetooth_Command(void);


// 주변장치 클럭 부여
void RCC_Configure(void)
{  
   /* USART1, USART2 TX/RX port clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); 

   /* HX711, Servo(PB0) clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

   /* 리드 스위치 clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

   /* USART1, USART2 clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

   /* Alternate Function IO clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   /* [추가] 서보모터 PWM용 TIM3 클럭 (APB1) */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

   /* [추가] TIM4(시간 측정용) */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* USART1 pin setting */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 pin setting */
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); // <--- 이 부분!

   /* HX711 pin setting */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* [추가] 서보모터 핀 설정 (PB0 -> TIM3_CH3) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // PWM 출력을 위해 AF_PP 사용
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
   
    /* 불꽃 감지 센서 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);

   /* 리드 스위치 (PC3) - 문 감지 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* 진동 센서 (PC1) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* TIM3 설정 함수 (PWM for Servo) */
void TIM_Configure(void)
{
    TIM_TimeBaseInitTypeDef TIM3_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 72MHz / 72 = 1MHz (1us)
    uint16_t prescale = (uint16_t) (SystemCoreClock / 1000000) - 1; 
    
    // Period 20ms (20000us)
    TIM3_InitStructure.TIM_Period = 20000 - 1; 
    TIM3_InitStructure.TIM_Prescaler = prescale;
    TIM3_InitStructure.TIM_ClockDivision = 0;
    TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);

    // PWM Mode 1, Channel 3 (PB0)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500; // 초기 위치 (중간)
    
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;
    USART_Cmd(USART1, ENABLE);
    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART1_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;
    USART_Cmd(USART2, ENABLE);
    USART2_InitStructure.USART_BaudRate = 9600; 
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART2_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // USART1
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // [추가] TIM4 인터럽트 설정
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 우선순위 낮게 설정
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ================================================================
 * 서보모터 각도 조절 함수 (Pulse 폭으로 제어)
 * pulse: 500(0도) ~ 1500(90도) ~ 2500(180도)
 * ================================================================ */
void Servo_Write(uint16_t pulse) {
    if(pulse < 500) pulse = 500;
    if(pulse > 2500) pulse = 2500;
    TIM_SetCompare3(TIM3, pulse);
}

/* HX711 드라이버 함수 구현 */
void HX711_Init_State(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    GPIO_SetBits(GPIOB, GPIO_Pin_6);   
    Delay_ms(10);                      
    GPIO_ResetBits(GPIOB, GPIO_Pin_6); 
    Delay_ms(10);
}

long HX711_Read(void)
{
    long count = 0;
    unsigned char i;

    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    
    __disable_irq(); 
    for (i = 0; i < 24; i++)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_6); 
        Delay_us(1); 
        count = count << 1;
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)) count++;
        GPIO_ResetBits(GPIOB, GPIO_Pin_6); 
        Delay_us(1);
    }
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    Delay_us(1);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    Delay_us(1);
    __enable_irq(); 
    
    count = count ^ 0x800000; 
    return count;
}
long HX711_Read_Average(unsigned char times)
{
    long sum = 0;
    for (unsigned char i = 0; i < times; i++) sum += HX711_Read();
    return sum / times;
}
void HX711_Tare(void) { HX711_Read_Average(30); }

/* DHT11 드라이버 함수 구현 */
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
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint8_t i, j;
    DHT11_GPIO_Output();
    GPIO_ResetBits(GPIOC, GPIO_Pin_2);
    Delay_ms(20); 
    GPIO_SetBits(GPIOC, GPIO_Pin_2);
    DHT11_Delay_us(30);
    DHT11_GPIO_Input();
    DHT11_Delay_us(10);
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
        *humi = data[0];
        *temp = data[2];
        return 1;
    }
    return 0;
}

/* 문자열 송신 함수 */
void USART2_SendString(char* str)
{
    while (*str) // null 문자('\0') 만날 때까지 반복
    {
        // 데이터 레지스터가 비었는지 확인 (TXE 플래그)
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        
        // 한 글자 전송
        USART_SendData(USART2, *str++);
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        // 1. 블루투스로부터 문자 1개 수신
        char ch = USART_ReceiveData(USART2);

        // ==========================================================
        // [추가된 부분] PC 터미널로 즉시 전송 (에코/디버깅용)
        // ==========================================================
        // USART1의 송신 버퍼가 빌 때까지 기다렸다가 전송
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
        USART_SendData(USART1, ch); 
        // ==========================================================

        // 2. 기존 도어락 명령어 처리 로직 (버퍼에 저장)
        if (ch == '\n' || ch == '\r') 
        {
            if (g_bt_rx_index > 0) 
            {
                g_bt_rx_buffer[g_bt_rx_index] = '\0'; // 문자열 끝 표시
                g_bt_data_ready = 1;                  // 메인 루프에 알림
                g_bt_rx_index = 0;                    // 인덱스 초기화
            }
        }
        else
        {
            if (g_bt_rx_index < MAX_RX_BUF - 1)
            {
                g_bt_rx_buffer[g_bt_rx_index++] = ch;
            }
        }
        
        // 플래그 클리어 (보통 읽기만 해도 클리어되지만 안전하게)
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

void Process_Bluetooth_Command(void)
{
    if (g_bt_data_ready) // 수신된 명령어가 있다면
    {
        g_bt_data_ready = 0; // 플래그 초기화

        // 1. 잠금 해제 명령 (CMD: UNLOCK)
        if (strcmp((char*)g_bt_rx_buffer, "CMD: UNLOCK") == 0)
        {
            Servo_Write(2400); // 서보모터 열림 각도 (값은 캘리브레이션 필요)
            servo_state = 1;   // 상태 변수 업데이트
            //USART2_SendString("OK: Door Unlocked\r\n"); // 폰으로 응답 전송
        }
        // 2. 잠금 명령 (CMD: LOCK)
        else if (strcmp((char*)g_bt_rx_buffer, "CMD: LOCK") == 0)
        {
            Servo_Write(1500); // 서보모터 닫힘 각도 (값은 캘리브레이션 필요)
            servo_state = 0;   // 상태 변수 업데이트
            //USART2_SendString("OK: Door Locked\r\n"); // 폰으로 응답 전송
        }
        // 3. 알 수 없는 명령어
        //else 
        // {
        //     // 디버깅용: 받은 이상한 명령어를 다시 보내봄
        //     char msg[80];
        //     //sprintf(msg, "ERROR: Unknown Command [%s]\r\n", g_bt_rx_buffer);
        //     USART2_SendString(msg);
        // }
        
        // 버퍼 초기화 (잔여 데이터 방지)
        memset((void*)g_bt_rx_buffer, 0, MAX_RX_BUF);
    }
}

/* [추가] 5초 카운팅을 위한 TIM4 설정 (1초마다 인터럽트) */
void TIM4_Configure(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    // 72MHz / 7200 = 10kHz (0.1ms)
    // Period = 10000 -> 10000 * 0.1ms = 1000ms = 1초
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStructure.TIM_Period = 10000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); // 업데이트 인터럽트 허용
    TIM_Cmd(TIM4, ENABLE);
}

/* [추가] TIM4 인터럽트 핸들러 */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        g_timer_sec_count++;
        
        if (g_timer_sec_count >= 5) // 5초가 되면
        {
            g_timer_sec_count = 0;
            g_send_temp_flag = 1; // 메인 루프에 알림
        }
        
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

/* 딜레이 함수 */
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
    USART1_Init();      
    USART2_Init();
    TIM4_Configure(); // 5초 타이머 시작

    NVIC_Configure();
    TIM_Configure();

    // HX711 초기화
    HX711_Init_State();
    Delay_ms(2000);
    HX711_Tare();

    /* 리드 스위치 상태 관리용 변수 */
    uint8_t lastDoorState = 0;
    /* 온습도 센서 타이머용 변수 */
    int dht_timer = 0;
    /* 진동 센서 이전 상태 저장 변수 */
    uint8_t lastVibState = 0;
    
    lastVibState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
    lastDoorState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);

    // 초기 상태: 잠금 해제, 모터 0도(열림)
    Servo_Write(1500); // 0도 (열림)
    current_state = STATE_UNLOCKED;

    while (1)
    {
        // ---------------------------------------------------------
        // 1. 블루투스 명령어 처리
        // ---------------------------------------------------------
        if (g_bt_data_ready)
        {
            g_bt_data_ready = 0; // 플래그 초기화

            // [잠금 명령] CMD:LOCK
            if (strcmp((char*)g_bt_rx_buffer, "CMD:LOCK") == 0)
            {
                // 1) 현재 무게를 측정해서 '기준값'으로 잡음 (물건이 들어있는 상태)
                reference_weight = HX711_Read(); 
                
                // 2) 서보모터 잠금 (90도 회전)
                Servo_Write(2400); // 90도 (잠금 위치, 값 조절 필요)
                
                // 3) 상태 변경
                current_state = STATE_LOCKED;
                
                // 4) 앱에 확인 메시지 전송
                char msg[64];
                sprintf(msg, "OK: LOCKED (Weight: %ld)\r\n", reference_weight);
                USART2_SendString(msg);
            }
            // [해제 명령] CMD:UNLOCK
            else if (strcmp((char*)g_bt_rx_buffer, "CMD:UNLOCK") == 0)
            {
                Servo_Write(1500); // 0도 (열림 위치)
                current_state = STATE_UNLOCKED;
                USART2_SendString("OK: UNLOCKED\r\n");
            }
            
            // 버퍼 초기화
            memset((void*)g_bt_rx_buffer, 0, MAX_RX_BUF);
        }

        // =========================================================
        // [추가] 5초마다 온습도 전송
        // =========================================================
        if (g_send_temp_flag == 1)
        {
            uint8_t temp = 0, humi = 0;
            char temp_msg[32]; // 전송을 위한 임시 문자열 버퍼 생성
            
            // 1. DHT11 데이터 읽기
            if (DHT11_Read_Data(&temp, &humi)) 
            {
                // 2. 보낼 문자열 포맷팅
                sprintf(temp_msg, "T=%d\n", temp);
                
                // 3. 기존 문자열 송신 함수 사용 (Blocking 방식)
                USART2_SendString(temp_msg);
            }
            else
            {
                // (선택사항) 센서 읽기 실패 시 에러 메시지 전송
                // USART2_SendString("Err: DHT11 Fail\r\n");
            }
            
            g_send_temp_flag = 0; // 플래그 초기화
        }

        // ---------------------------------------------------------
        // 2. 도난 감지 로직 (LOCKED 상태일 때만 동작)
        // ---------------------------------------------------------
        if (current_state == STATE_LOCKED)
        {
            current_weight = HX711_Read(); // 실시간 무게 측정

            // 도난 판단 조건:
            // (기준 무게보다 현저히 가벼워짐) OR (빈 통 무게(820000) 근처로 돌아감)
            // reference_weight - current_weight > THRESHOLD : 기준보다 무게가 많이 빠짐
            
            if ((reference_weight - current_weight) > THRESHOLD)
            {
                // 도난 발생!!
                USART2_SendString("WARNING: THEFT DETECTED!\r\n");
                
                // PC 터미널에도 디버깅 출력
                // printf("THEFT! Ref: %ld, Curr: %ld\r\n", reference_weight, current_weight);
                
                // (선택사항) 너무 자주 보내지 않게 딜레이를 주거나,
                // 한 번 보내고 상태를 바꾸는 등의 처리가 필요할 수 있음
                Delay_ms(1000); 
            }
        }
        
        Delay_ms(100); // 루프 속도 조절
    }

}
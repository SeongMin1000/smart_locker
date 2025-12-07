#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h" // [추가] 타이머 헤더

#include "misc.h"

/* ================================================================
 * [사용자 설정] HX711 핀 & 보정 계수
 * ================================================================ */
float Calibration_Factor = 400.0f; 
long Zero_Offset = 0;

/* ================================================================
 * 전역 변수
 * ================================================================ */
// 1. 로드셀 관련
volatile float weight = 0.0f;
volatile long raw_data = 0;

// 2. 불꽃 감지 센서 상태 (0: 정상, 1: 화재감지)
volatile uint8_t flame_detected = 0; 

// 3. 리드 스위치 (문) 상태
volatile uint8_t g_IsDoorOpen = 0;
volatile uint32_t g_DoorOpenCount = 0;

// 4. 온습도 센서
volatile uint32_t g_Temperature = 0;
volatile uint32_t g_Humidity = 0;

// 5. 진동 센서
volatile uint32_t g_VibrationCount = 0;   
volatile uint8_t  g_VibrationDetected = 0; 

// 6. [NEW] 서보모터 동작 테스트용 변수
int servo_test_timer = 0;
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

/* [추가] TIM3 설정 함수 (PWM for Servo) */
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
}

/* ================================================================
 * [추가] 서보모터 각도 조절 함수 (Pulse 폭으로 제어)
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
    while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)); 
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
void HX711_Tare(void) { Zero_Offset = HX711_Read_Average(30); }
void Delay_us(uint32_t us) { us *= 12; while(us--); }
void Delay_ms(uint32_t ms) { while(ms--) Delay_us(1000); }

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
    NVIC_Configure();

    // [추가] 서보모터용 PWM 타이머 설정
    TIM_Configure();

    // HX711 초기화
    HX711_Init_State();
    Delay_ms(2000);
    HX711_Tare();

    /* 리드 스위치 상태 관리용 변수 (지역) */
    uint8_t lastDoorState = 0;
    /* 온습도 센서 타이머용 변수 */
    int dht_timer = 0;
    /* 진동 센서 이전 상태 저장 변수 */
    uint8_t lastVibState = 0;
    
    lastVibState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
    lastDoorState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);

    // 초기 서보 위치 (닫힘: 1500)
    Servo_Write(1500); 

    while (1)
    {
        /* 1. 로드셀 (무게 측정) */
        raw_data = HX711_Read_Average(10);
        weight = (float)(raw_data - Zero_Offset) / Calibration_Factor;
        
        /* 2. 불꽃 감지 센서 */
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == Bit_RESET) {
            if (flame_detected == 0) {
                 flame_detected = 1; 
                 // 화재 시 문 열기 (예시)
                 // Servo_Write(2400); 
            }
        } else {
            flame_detected = 0;
        }

        /* 3. 리드 스위치 (문 감지) */
        uint8_t currentDoorState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
        g_IsDoorOpen = currentDoorState;

        if (lastDoorState == 0 && currentDoorState == 1) {
            g_DoorOpenCount++; 
            Delay_ms(50);      
        } else if (lastDoorState == 1 && currentDoorState == 0) {
            Delay_ms(50);      
        }
        lastDoorState = currentDoorState;

        /* 4. 온습도 센서 */
        dht_timer++; 
        if (dht_timer >= 200) {
            dht_timer = 0; 
            uint8_t t = 0, h = 0;
            if (DHT11_Read_Data(&t, &h) == 1) {
                g_Temperature = t;
                g_Humidity = h;
            }
        }

        /* 5. 진동 센서 */
        uint8_t currentVibState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
        g_VibrationDetected = currentVibState;

        if (lastVibState == 0 && currentVibState == 1) {
            g_VibrationCount++; 
            Delay_ms(50); 
        }
        lastVibState = currentVibState;


        /* =======================================================
         * [NEW] 서보모터 동작 테스트 코드
         * 2초(200 loop * 10ms)마다 열렸다 닫혔다 반복
         * ======================================================= */
        servo_test_timer++;
        if (servo_test_timer > 200) { // 약 2초마다 실행
            servo_test_timer = 0;
            if (servo_state == 0) {
                Servo_Write(2400); // 열림 (각도 조절 필요: 500~2500)
                servo_state = 1;
            } else {
                Servo_Write(1500); // 닫힘
                servo_state = 0;
            }
        }

        /* 루프 딜레이 */
        Delay_ms(10);
    }
}
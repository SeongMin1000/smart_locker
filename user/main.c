#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "misc.h"

/* ================================================================
 * [사용자 설정] HX711 핀 & 보정 계수
 * ================================================================ */
float Calibration_Factor = 400.0f; 
long Zero_Offset = 0;

/* ================================================================
 * 전역 변수 (디버깅 확인용)
 * ================================================================ */
// 1. 로드셀 관련
volatile float weight = 0.0f;
volatile long raw_data = 0;

// 2. 불꽃 감지 센서 상태 (0: 정상, 1: 화재감지)
volatile uint8_t flame_detected = 0; 

// 3. [NEW] 리드 스위치 (문) 상태
// 0: 닫힘(자석 붙음), 1: 열림(자석 떨어짐)
volatile uint8_t g_IsDoorOpen = 0;
// 문 열린 횟수 카운트
volatile uint32_t g_DoorOpenCount = 0;

// 4. 온습도 센서
volatile uint32_t g_Temperature = 0;
volatile uint32_t g_Humidity = 0;

// 5. 진동 센서
volatile uint32_t g_VibrationCount = 0;   // 총 진동 횟수
volatile uint8_t  g_VibrationDetected = 0; // 현재 상태 (0:조용, 1:진동)

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

/* DHT11 관련 함수 */
void DHT11_GPIO_Output(void);
void DHT11_GPIO_Input(void);
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi);
void DHT11_Delay_us(uint32_t us);


// 주변장치 클럭 부여
void RCC_Configure(void)
{  
	/* USART1, USART2 TX/RX port clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // USART1 (PA9, PA10)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // USART2 (PD5, PD6), 불꽃 감지 (PD3)

	/* HX711 clock enable*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // DATA - PB6, SCK - PB6

	/* 리드 스위치 clock enable*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

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

	
    /* 4. 불꽃 감지 센서 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 풀업 입력 설정
    GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* 5. 리드 스위치 (PC3) - 문 감지 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 내부 풀업 사용
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* 6. 진동 센서 (PC1) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 보통 모듈은 Floating 사용
    GPIO_Init(GPIOC, &GPIO_InitStructure);
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
 * [추가] DHT11 드라이버 함수 구현 (PC2 사용)
 * ================================================================ */

// DHT11 전용 미세 딜레이 (기존 Delay_us와 다르게 튜닝됨)
void DHT11_Delay_us(uint32_t us)
{
    us *= 8; 
    while(us--) { __NOP(); }
}

void DHT11_GPIO_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PC2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void DHT11_GPIO_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PC2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint8_t i, j;

    // 1. Start Signal
    DHT11_GPIO_Output();
    GPIO_ResetBits(GPIOC, GPIO_Pin_2);
    Delay_ms(20); // 18ms 이상 Low
    GPIO_SetBits(GPIOC, GPIO_Pin_2);
    DHT11_Delay_us(30);

    // 2. Check Response
    DHT11_GPIO_Input();
    DHT11_Delay_us(10);
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1) return 0; // 응답 없음
    
    while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 0); // Low 대기
    while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1); // High 대기

    // 3. Data Read (40 bits)
    for (i = 0; i < 5; i++)
    {
        for (j = 0; j < 8; j++)
        {
            while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 0); // Low 대기

            DHT11_Delay_us(40); // 40us 후 High인지 확인

            if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1)
            {
                data[i] |= (1 << (7 - j));
                while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1); // 남은 High 대기
            }
        }
    }

    // 4. Checksum
    if ((data[0] + data[1] + data[2] + data[3]) == data[4])
    {
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

    // HX711 초기화
    HX711_Init_State();
    Delay_ms(2000);
    HX711_Tare();

    /* 리드 스위치 상태 관리용 변수 (지역) */
    uint8_t lastDoorState = 0;
	/* [추가] 온습도 센서 타이머용 변수 */
    int dht_timer = 0;
	// [추가] 진동 센서 이전 상태 저장 변수
    uint8_t lastVibState = 0;
    lastVibState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
    
    // 초기 문 상태 읽기
    lastDoorState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);

    while (1)
    {
        /* -------------------------------------------
         * 1. 로드셀 (무게 측정)
         * ------------------------------------------- */
        raw_data = HX711_Read_Average(10);
        weight = (float)(raw_data - Zero_Offset) / Calibration_Factor;
        
        /* -------------------------------------------
         * 2. 불꽃 감지 센서 (PD3)
         * ------------------------------------------- */
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == Bit_RESET) {
            if (flame_detected == 0) {
                 // 화재 감지 시 동작 (필요 시 UART 전송 코드 추가)
                 flame_detected = 1; 
            }
        } else {
            flame_detected = 0;
        }

        /* -------------------------------------------
         * 3. [NEW] 리드 스위치 (문 감지) (PC3)
         * ------------------------------------------- */
        // 현재 상태 읽기
        uint8_t currentDoorState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
        
        // 전역 변수 업데이트 (디버깅용)
        g_IsDoorOpen = currentDoorState;

        // 문이 열림 감지 (0 -> 1 : Rising Edge)
        // 닫혀있음(0) -> 열림(1)
        if (lastDoorState == 0 && currentDoorState == 1)
        {
            g_DoorOpenCount++; // 카운트 증가
            Delay_ms(50);      // 바운싱 방지 (떨림 제거)
        }
        // 문이 닫힘 감지 (1 -> 0 : Falling Edge)
        else if (lastDoorState == 1 && currentDoorState == 0)
        {
            Delay_ms(50);      // 바운싱 방지
        }
        
        // 상태 업데이트
        lastDoorState = currentDoorState;

		/* -------------------------------------------
         * 4. 온습도 센서
         * ------------------------------------------- */
		dht_timer++; // 루프 돌 때마다 카운트 증가

        // 루프 딜레이가 10ms이므로, 200번 돌면 약 2초 (10ms * 200 = 2000ms)
        if (dht_timer >= 200) 
        {
            dht_timer = 0; // 카운터 초기화
            
            uint8_t t = 0, h = 0;
            // DHT11 읽기 시도 (PC2 핀)
            if (DHT11_Read_Data(&t, &h) == 1)
            {
                g_Temperature = t;
                g_Humidity = h;
            }
            // 실패하면 이전 값 유지 (혹은 에러처리)
        }

		/* -------------------------------------------
         * 4. [NEW] 진동 센서 (PC1)
         * ------------------------------------------- */
        uint8_t currentVibState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
        g_VibrationDetected = currentVibState;

        // 진동 감지 (0 -> 1 : Rising Edge)
        // 평소엔 0이다가 흔들리면 1이 되는 타입 기준
        if (lastVibState == 0 && currentVibState == 1) 
        {
            g_VibrationCount++; // 카운트 증가
            
            // [중요] 진동은 타다닥! 하고 여러 번 울리므로
            // 한번 감지하면 0.05초 정도는 무시해줍니다.
            Delay_ms(50); 
        }
        
        lastVibState = currentVibState;

        /* 루프 딜레이 */
        Delay_ms(10);
    }
}
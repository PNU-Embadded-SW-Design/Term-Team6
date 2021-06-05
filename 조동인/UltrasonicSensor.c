#include <includes.h>

static  OS_TCB   AppTaskStartTCB;
static  OS_TCB   AppTask1_TCB; // Ultrasonic Sensor

static  CPU_STK  AppTaskStartStk[APP_TASK_START_STK_SIZE]; 
static  CPU_STK  AppTask1_Stk[128];

static  void  AppTaskStart  	(void *p_arg); 
static  void  AppTask1   		(void *p_arg); 

#define US_TIMER					TIM3

#define US_TRIG_PORT				GPIOB
#define US_TRIG_PIN					GPIO_Pin_0		//TIM3 Ch3 (trig output), PB0

#define US_ECHO_PORT				GPIOA
#define US_ECHO_PIN				GPIO_Pin_6		//TIM Ch1 (echo input), PA6
#define US_TIMER_TRIG_SOURCE		TIM_TS_TI1FP1

void EnableHCSR04PeriphClock(void);
void UART_Configuration(void);
void initMeasureTimer(void); 
void initPins(void);
void InitHCSR04(void);
int HCSR04GetDistance(void);
void sendData(int);
 
int  main (void)
{
    OS_ERR  err;
    BSP_IntDisAll();
    OSInit(&err);

    OSTaskCreate((OS_TCB     *)&AppTaskStartTCB,
                 (CPU_CHAR   *)"App Task Start",
                 (OS_TASK_PTR )AppTaskStart, 
                 (void       *)0,
                 (OS_PRIO     )APP_TASK_START_PRIO,
                 (CPU_STK    *)&AppTaskStartStk[0],
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )0,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSStart(&err);
}

static  void  AppTaskStart (void *p_arg)
{
    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;
    OS_ERR      err;
    //CPU_TS  ts;
    
   (void)p_arg;

    BSP_Init();
    CPU_Init();

    cpu_clk_freq = BSP_CPU_ClkFreq();
    cnts         = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;
    OS_CPU_SysTickInit(cnts);

    #if OS_CFG_STAT_TASK_EN > 0u
      OSStatTaskCPUUsageInit(&err);
    #endif

    CPU_IntDisMeasMaxCurReset();
    
     EnableHCSR04PeriphClock();
     UART_Configuration();
     initMeasureTimer();
     initPins();
    
    OSTaskCreate((OS_TCB     *)&AppTask1_TCB,
                 (CPU_CHAR   *)"App Task1",
                 (OS_TASK_PTR )AppTask1, 
                 (void       *)0,
                 (OS_PRIO     )4,
                 (CPU_STK    *)&AppTask1_Stk[0],
                 (CPU_STK_SIZE)128/10,
                 (CPU_STK_SIZE)128,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )0,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    while (DEF_TRUE) {
        OSTimeDlyHMSM(0, 0, 1, 0, 
                      OS_OPT_TIME_HMSM_STRICT, 
                      &err);
    }
}

void EnableHCSR04PeriphClock() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  }

void UART_Configuration()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// RCC Configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // TX, PA9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // RX, PA10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// UART Port ¼³Á¤
	USART_InitStructure.USART_BaudRate    = 9600;
	USART_InitStructure.USART_WordLength   = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits    = USART_StopBits_1;
	USART_InitStructure.USART_Parity    = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode    = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

void initMeasureTimer() {
	RCC_ClocksTypeDef RCC_ClocksStatus;
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	u16 prescaler = RCC_ClocksStatus.SYSCLK_Frequency / 1000000 - 1; //1 tick = 1us (1 tick = 0.165mm resolution)

	TIM_DeInit(US_TIMER);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_Prescaler = prescaler;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(US_TIMER, &TIM_TimeBaseInitStruct);

	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 15; //us
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(US_TIMER, &TIM_OCInitStruct);

	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICFilter = 0;

	TIM_PWMIConfig(US_TIMER, &TIM_ICInitStruct);
	TIM_SelectInputTrigger(US_TIMER, US_TIMER_TRIG_SOURCE);
	TIM_SelectMasterSlaveMode(US_TIMER, TIM_MasterSlaveMode_Enable);

	TIM_CtrlPWMOutputs(US_TIMER, ENABLE);

	TIM_ClearFlag(US_TIMER, TIM_FLAG_Update);
}

void initPins() {
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(US_TRIG_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(US_ECHO_PORT, &GPIO_InitStructure);
}

void InitHCSR04() {
	EnableHCSR04PeriphClock();
	initPins();
	initMeasureTimer();
}

int HCSR04GetDistance() {
	(US_TIMER)->CNT = 0;
	TIM_Cmd(US_TIMER, ENABLE);
	while(!TIM_GetFlagStatus(US_TIMER, TIM_FLAG_Update));
	TIM_Cmd(US_TIMER, DISABLE);
	TIM_ClearFlag(US_TIMER, TIM_FLAG_Update);
	return (TIM_GetCapture2(US_TIMER)-TIM_GetCapture1(US_TIMER))*165/1000;
}

void sendData(int n){
    char c;
  	
    c = 0x30 + (n /100)%10;
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    USART_SendData(USART1,c);
    c = 0x30 + (n /10)%10;
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    USART_SendData(USART1,c);
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    USART_SendData(USART1,'.');
    c = 0x30 + n%10;
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    USART_SendData(USART1,c);
    
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    USART_SendData(USART1,'c');
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    USART_SendData(USART1,'m');
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    USART_SendData(USART1,'\r');
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    USART_SendData(USART1,'\n');
}

static void AppTask1 (void *p_arg)  // Ultrasonic Sensor
{
	//OS_ERR err;
	p_arg = p_arg;
        
	InitHCSR04();
  
	while (1) {
          int dist = HCSR04GetDistance();
          sendData(dist);
        }
} 

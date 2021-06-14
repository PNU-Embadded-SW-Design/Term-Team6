#include <includes.h>

static OS_TCB AppTaskStartTCB;
static OS_TCB AppUltrasonicSensor_TCB; 
static OS_TCB AppWaterSensor_TCB; 
static OS_TCB AppServoMotor_TCB; 
static OS_TCB AppWaterPump_TCB;

static CPU_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static CPU_STK AppUltrasonicSensor_Stk[128];
static CPU_STK AppWaterSensor_Stk[128];
static CPU_STK AppServoMotor_Stk[128];
static CPU_STK AppWaterPump_Stk[128];

static void AppTaskStart  	(void *p_arg); 
static void AppUltrasonicSensorTask (void *p_arg); 
static void AppWaterSensorTask (void *p_arg);
static void AppServoMotorTask (void *p_arg); 
static void AppWaterPumpTask (void *p_arg);

#define US_TIMER TIM3 // channel3
#define US_TRIG_PORT GPIOB
#define US_TRIG_PIN GPIO_Pin_0
#define US_ECHO_PORT GPIOA
#define US_ECHO_PIN	GPIO_Pin_6

#define WATERSENSOR_PORT GPIOC
#define WATERSENSOR_PIN GPIO_Pin_5

#define SERVO_TIMER TIM4 // channel3
#define SERVO_PORT GPIOB
#define SERVO_PIN GPIO_Pin_8

#define WATERPUMP_PORT GPIOD
#define WATERPUMP_PIN GPIO_Pin_11

#define TOUCH_PORT GPIOC
#define TOUCH1_PIN GPIO_Pin_8
#define TOUCH2_PIN GPIO_Pin_9

void GPIO_Configuration(void);
void ADC_Configuration(void);
void TIM3_Configuration(void);
void TIM4_Configuration(void);
void UART_Configuration(void);

int HCSR04GetDistance(void);
void change_pwm_pulse(int);
void delay(u32);
void sendUSData(int);
void sendWSData(int n);

int distance = 0;
int waterLevel = 0;
int ServoSig = 0;
int WaterPumpSig = 0;

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
    
    GPIO_Configuration();
    ADC_Configuration();
    TIM3_Configuration();
    TIM4_Configuration();
    UART_Configuration();
    
    // Round Robin 
    OSSchedRoundRobinCfg((CPU_BOOLEAN)DEF_TRUE, 
                         (OS_TICK    )10,
                         (OS_ERR    *)&err);
                         
	// Ultrasonic Sensor Task
    OSTaskCreate((OS_TCB     *)&AppUltrasonicSensor_TCB,
                 (CPU_CHAR   *)"Ultrasonic Sensor Task",
                 (OS_TASK_PTR )AppUltrasonicSensorTask, 
                 (void       *)0,
                 (OS_PRIO     )5,
                 (CPU_STK    *)&AppUltrasonicSensor_Stk[0],
                 (CPU_STK_SIZE)128/10,
                 (CPU_STK_SIZE)128,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )2.5,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
                 
	// Water Sensor Task
    OSTaskCreate((OS_TCB *)&AppWaterSensor_TCB,
                 (CPU_CHAR *)"Water Sensor Task",
                 (OS_TASK_PTR)AppWaterSensorTask,
                 (void *) 0,
                 (OS_PRIO) 5,
                 (CPU_STK *) &AppWaterSensor_Stk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE/10,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY) 0,
                 (OS_TICK) 2.5,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR),
                 (OS_ERR *) &err);
    
    // Servo Motor Task
    OSTaskCreate((OS_TCB     *)&AppServoMotor_TCB,
                 (CPU_CHAR   *)"Servo Motor Task",
                 (OS_TASK_PTR )AppServoMotorTask, 
                 (void       *)0,
                 (OS_PRIO     )5,
                 (CPU_STK    *)&AppServoMotor_Stk[0],
                 (CPU_STK_SIZE)128/10,
                 (CPU_STK_SIZE)128,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )2.5,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
                 
    // Water Pump Task
    OSTaskCreate((OS_TCB *)&AppWaterPump_TCB,
               (CPU_CHAR *)"Water Pump Task",
               (OS_TASK_PTR)AppWaterPumpTask,
               (void *) 0,
               (OS_PRIO) 5,
               (CPU_STK *) &AppWaterPump_Stk[0],
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE/10,
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
               (OS_MSG_QTY) 0,
               (OS_TICK) 2.5,
               (void *) 0,
               (OS_OPT)(OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR),
               (OS_ERR *) &err);

    while (DEF_TRUE) {
        OSTimeDlyHMSM(0, 0, 1, 0, 
                      OS_OPT_TIME_HMSM_STRICT, 
                      &err);
    }
}

static void AppUltrasonicSensorTask (void *p_arg)
{
	//OS_ERR err;
	p_arg = p_arg;
  
	while (1) {
    	distance = HCSR04GetDistance();
        sendUSData(distance);
    }
}

static void AppWaterSensorTask (void *p_arg)
{
  //OS_ERR err;
  p_arg = p_arg;
  
  while(1){
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1,0x2) == RESET);
    
    waterLevel = ADC_GetConversionValue(ADC1);
    sendWSData(waterLevel);
  }
}

static  void  AppServoMotorTask (void *p_arg)
{
    //OS_ERR err;
    p_arg = p_arg;
    
    while (1) {
	  ServoSig = GPIO_ReadInputDataBit(TOUCH_PORT, TOUCH1_PIN);
	  
      if(ServoSig == 1) {
		if(distance < 20)) change_pwm_pulse(1800);
	    else change_pwm_pulse(1400);
	      
	    delay(1000000);
  	  }
    }
}


static void AppWaterPumpTask (void *p_arg)
{
  //OS_ERR err;
  p_arg = p_arg;
  
  while(1){
	WaterPumpSig = GPIO_ReadInputDataBit(TOUCH_PORT,TOUCH2_PIN);
    
    if(WaterPumpSig == 1) {
		if(distance < 20 && waterLevel < 2000)
			GPIO_ResetBits(WATERPUMP_PORT, WATERPUMP_PIN); // pump on
	}
	else GPIO_SetBits(WATERPUMP_PORT, WATERPUMP_PIN); // pump off
  }
}

void GPIO_Configuration()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB
  						| RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(US_TRIG_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(US_ECHO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin =  SERVO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SERVO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = WATERPUMP_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(WATERPUMP_PORT,&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  TOUCH1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(TOUCH_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = TOUCH2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(TOUCH_PORT, &GPIO_InitStructure);
}

void ADC_Configuration(){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
  
  GPIO_InitTypeDef gpio_init;
  gpio_init.GPIO_Pin = GPIO_Pin_5;
  gpio_init.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC,&gpio_init);
  
  ADC_InitTypeDef adc_init;
  adc_init.ADC_Mode = ADC_Mode_Independent;
  adc_init.ADC_ScanConvMode = DISABLE;
  adc_init.ADC_ContinuousConvMode = DISABLE;
  adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  adc_init.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1,&adc_init);
  
  ADC_RegularChannelConfig(ADC1,ADC_Channel_15,1,ADC_SampleTime_55Cycles5);
  
  ADC_Cmd(ADC1,ENABLE);
  
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
}

void TIM3_Configuration() {
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
	TIM_SelectInputTrigger(US_TIMER, TIM_TS_TI1FP1);
	TIM_SelectMasterSlaveMode(US_TIMER, TIM_MasterSlaveMode_Enable);

	TIM_CtrlPWMOutputs(US_TIMER, ENABLE);

	TIM_ClearFlag(US_TIMER, TIM_FLAG_Update);
}

void TIM4_Configuration()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; // Timer 설정  
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    // 50Hz Timer Clock(20ms)
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1; // SystemCoreClock = 72MHz
    TIM_TimeBaseStructure.TIM_Period = 20000-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit(SERVO_TIMER, &TIM_TimeBaseStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM mode 설정  
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1400;
    TIM_OC3Init(SERVO_TIMER, &TIM_OCInitStructure); // TIM4 Chnnel3
    TIM_OC3PreloadConfig(SERVO_TIMER, TIM_OCPreload_Disable);
    
    TIM_ARRPreloadConfig(SERVO_TIMER, ENABLE);
    TIM_Cmd(SERVO_TIMER, ENABLE);
}

int HCSR04GetDistance()
{
	(US_TIMER)->CNT = 0;
	TIM_Cmd(US_TIMER, ENABLE);
	while(!TIM_GetFlagStatus(US_TIMER, TIM_FLAG_Update));
	TIM_Cmd(US_TIMER, DISABLE);
	TIM_ClearFlag(US_TIMER, TIM_FLAG_Update);
	return (TIM_GetCapture2(US_TIMER)-TIM_GetCapture1(US_TIMER))*165/1000;
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

	// UART Port 설정
	USART_InitStructure.USART_BaudRate    = 9600;
	USART_InitStructure.USART_WordLength   = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits    = USART_StopBits_1;
	USART_InitStructure.USART_Parity    = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode    = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

void change_pwm_pulse(int pwm_pulse) 
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pwm_pulse;
    TIM_OC3Init(SERVO_TIMER, &TIM_OCInitStructure);
}

void delay(u32 delay)
{
  while(delay--);
}

void sendUSData(int n){
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

void sendWSData(int n){
  char c;
  
  c = 0x30 + (n / 1000)%10;
  while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
  USART_SendData(USART1,c);
  c = 0x30 + (n / 100)%10;
  while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
  USART_SendData(USART1,c);
  c = 0x30 + (n / 10)%10;
  while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
  USART_SendData(USART1,c);
  c = 0x30 + (n)%10;
  while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
  USART_SendData(USART1,c);
  
  while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
  USART_SendData(USART1,'\r');
  while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
  USART_SendData(USART1,'\n');
}

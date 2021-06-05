#include <includes.h>

static OS_TCB AppTaskStartTCB;
static OS_TCB AppWaterSensor_TCB;
static OS_TCB AppTask1_TCB;     //test task

static CPU_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static CPU_STK AppWaterSensor_Stk[128];
static CPU_STK AppTask1_Stk[APP_TASK_START_STK_SIZE]; // test stk

static void AppTaskStart (void *p_arg);
static void AppWaterSensorTask (void *p_arg);
static void AppTask1 (void *p_arg); // test task

void write(int n);

// function
void RTC_Configuration(void);
void Clock_Init(void);
void USART_OptionInit();

int signal = 0;

int main (void){
  OS_ERR err;
  BSP_IntDisAll();
  OSInit(&err);  
  
  OSTaskCreate((OS_TCB     *)&AppTaskStartTCB,                /* Create the start task                                */
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

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}

static void AppTaskStart (void *p_arg){
  CPU_INT32U cpu_clk_freq;
  CPU_INT32U cnts;
  OS_ERR err;
  CPU_TS ts;
  (void) p_arg;
  
  BSP_Init();
  CPU_Init();
  
  cpu_clk_freq = BSP_CPU_ClkFreq();
  cnts = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;
  OS_CPU_SysTickInit(cnts);
  
    
  GPIO_InitTypeDef gpio_init,gpio_init2;
  
  //led 불빛 task 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);//clock give
  //RCC_APB2Periph_ADC1
  gpio_init.GPIO_Pin = (GPIO_Pin_2);
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD,&gpio_init);
  //PD11 을 이용하여 water pump 를 조정 한다.
  // 버튼 입력
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
  gpio_init2.GPIO_Pin = (GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
  gpio_init2.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio_init2.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC,&gpio_init2);
  
  //ADC 설정--------------------------
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//clock give
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
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
  //------------------
  
  //----------usart1----------
  USART_OptionInit();
  //----------------------------------------
  
  //RTC
  RTC_Configuration();
  Clock_Init();
  //
  
  // task 를 round robin 으로 하시 위해서 
  OSSchedRoundRobinCfg((CPU_BOOLEAN)DEF_TRUE, 
                         (OS_TICK    )10,
                         (OS_ERR    *)&err);
  
  
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
               (OS_TICK) 3,
               (void *) 0,
               (OS_OPT)(OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR),
               (OS_ERR *) &err);
  
  // test task
  OSTaskCreate((OS_TCB     *)&AppTask1_TCB,
                 (CPU_CHAR   *)"App Task 1",
                 (OS_TASK_PTR )AppTask1, 
                 (void       *)0,
                 (OS_PRIO     )5,
                 (CPU_STK    *)&AppTask1_Stk[0],
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )3,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
  
  #if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);                                 /* Compute CPU capacity with no task running        */
#endif

  CPU_IntDisMeasMaxCurReset();
  
  while(DEF_TRUE){
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    u32 t = 0;
    t = RTC_GetCounter();       //현재 시간 받기
    write(t);
    OSTimeDlyHMSM(0,0,1,0,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  }
}
static void AppWaterSensorTask (void *p_arg){
  OS_ERR err;
  p_arg = p_arg;
  
  int adc_value = 0;
  while(1){
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1,0x2) == RESET);
    adc_value = ADC_GetConversionValue(ADC1);
    
    signal = adc_value;
    
    signal = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2); // down
    //write(adc_value);
    if(signal == 0){
      GPIO_ResetBits(GPIOD, GPIO_Pin_11);
    }else{
      GPIO_SetBits(GPIOD,GPIO_Pin_11);
    }
  }
}
static void AppTask1 (void *p_arg){
  OS_ERR err;
  p_arg = p_arg;
  int ccc = 0;
  int f = 0;
  while(1){
    if( ccc >= 100000){
      //write(1123);
      ccc = 0;
    }
    ccc +=1;
  }
}

void RTC_Configuration(void){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP,ENABLE);
  
  PWR_BackupAccessCmd(ENABLE);
  
  BKP_DeInit();
  
  RCC_LSEConfig(RCC_LSE_ON);
  
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
  
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  RCC_RTCCLKCmd(ENABLE);
  
  RTC_WaitForSynchro();
  RTC_WaitForLastTask();
  
  RTC_ITConfig(RTC_IT_SEC,ENABLE);
  RTC_WaitForLastTask();
  RTC_SetPrescaler(32767);
  RTC_WaitForLastTask();
}
void Clock_Init(void){
  if(BKP_ReadBackupRegister(BKP_DR1)!= 0xA5A5){
    RTC_Configuration();
    RTC_WaitForLastTask();
    RTC_SetCounter(0);
    RTC_WaitForLastTask();
    BKP_WriteBackupRegister(BKP_DR1,0xA5A5);
  }else{
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
    RTC_ITConfig(RTC_IT_SEC,ENABLE);
  }
  RCC_ClearFlag();
}

void USART_OptionInit(){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  
  GPIO_InitTypeDef gpio_init;
  //TXD write pin9
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Pin = GPIO_Pin_9;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_init);
  
  //Rx read pin10
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio_init.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA,&gpio_init);
  
  //uart1 Task
  USART_InitTypeDef uart_init;
  uart_init.USART_BaudRate = 9600;
  uart_init.USART_WordLength = USART_WordLength_8b;
  uart_init.USART_StopBits = USART_StopBits_1;
  uart_init.USART_Parity = USART_Parity_No;
  uart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  uart_init.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
  USART_Init(USART1,&uart_init);
  USART_Cmd(USART1,ENABLE);
}
void write(int n){
  char c=0x30;
  c = 0x30 + (n / 10000)%10;
  while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
  USART_SendData(USART1,c);
  
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
  USART_SendData(USART1,'\n');
  while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
  USART_SendData(USART1,'\r');
}
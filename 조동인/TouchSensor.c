#include <includes.h>

static  OS_TCB   AppTaskStartTCB;
static  OS_TCB   AppTask1_TCB; // Touch Sensor

static  CPU_STK  AppTaskStartStk[APP_TASK_START_STK_SIZE]; 
static  CPU_STK  AppTask1_Stk[128];

static  void  AppTaskStart  	(void *p_arg); 
static  void  AppTask1   	(void *p_arg); 

void Setup(void);

int TouchSig1 = 0;
int TouchSig2 = 0;

int  main (void)
{
    OS_ERR  err;
    BSP_IntDisAll();
    OSInit(&err);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // LED1(PD2), LED2(PD3)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD,&GPIO_InitStructure);

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
    
    Setup();
    
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

void Setup()
{
   GPIO_InitTypeDef GPIO_InitStructure;
   
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9; // Touch Sensor1(PC8), Touch Sensor2(PC9)
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static  void  AppTask1 (void *p_arg)
{
  //OS_ERR err;
  p_arg = p_arg;
  
  while (1) {
    TouchSig1 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8);
    TouchSig2 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9);
    
    // Touch Sensor1 신호존재여부에 따라 LED1 ON/OFF
    if(TouchSig1 == 0) GPIO_ResetBits(GPIOD, GPIO_Pin_2); 
    else GPIO_SetBits(GPIOD,GPIO_Pin_2);
    
    // Touch Sensor2 신호존재여부에 따라 LED2 ON/OFF
    if(TouchSig2 == 0) GPIO_ResetBits(GPIOD, GPIO_Pin_3); 
    else GPIO_SetBits(GPIOD,GPIO_Pin_3);
  }
}

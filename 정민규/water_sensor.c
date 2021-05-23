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


int signal = 0;

int main (void){
  OS_ERR err;
  BSP_IntDisAll();
  OSInit(&err);
  
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
  
  //ADC 설정
  
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
  
  // task 를 round robin 으로 하시 위해서 
  OSSchedRoundRobinCfg((CPU_BOOLEAN)DEF_TRUE, 
                         (OS_TICK    )10,
                         (OS_ERR    *)&err);
  
  
  // Water Pump Task
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
    OSTimeDlyHMSM(0,0,0,100,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  }
}
static void AppWaterSensorTask (void *p_arg){
  OS_ERR err;
  p_arg = p_arg;
  while(1){
    signal = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2); // down
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
      f = !f;
      if(f == 0)
        GPIO_SetBits(GPIOD,GPIO_Pin_2);
      else
        GPIO_ResetBits(GPIOD, GPIO_Pin_2);
      ccc = 0;
    }
    ccc +=1;
  }
}
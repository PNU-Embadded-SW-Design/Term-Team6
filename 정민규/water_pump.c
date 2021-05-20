#include <includes.h>

static OS_TCB AppTaskStartTCB;
static OS_TCB AppTask1_TCB;
static OS_TCB AppTask2_TCB;
static OS_TCB AppTask3_TCB;
static OS_TCB AppTask4_TCB;

static CPU_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static CPU_STK AppTask1_Stk[128];
static CPU_STK AppTask2_Stk[128];
static CPU_STK AppTask3_Stk[128];
static CPU_STK AppTask4_Stk[128];

static void AppTaskStart (void *p_arg);
static void AppTask1 (void *p_arg);
static void AppTask2 (void *p_arg);
static void AppTask3 (void *p_arg);
static void AppTask4 (void *p_arg);

int main (void){
  OS_ERR err;
  BSP_IntDisAll();
  OSInit(&err);
  
  GPIO_InitTypeDef gpio_init,gpio_init2;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);//clock give
  gpio_init.GPIO_Pin = (GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_7);
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD,&gpio_init);
  
  
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
  gpio_init2.GPIO_Pin = (GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
  gpio_init2.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio_init2.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC,&gpio_init2);
  
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
  
  //1
  OSTaskCreate((OS_TCB *)&AppTask1_TCB,
               (CPU_CHAR *)"App Task 1",
               (OS_TASK_PTR)AppTask1,
               (void *) 0,
               (OS_PRIO) 1,
               (CPU_STK *) &AppTask1_Stk[0],
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE/10,
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
               (OS_MSG_QTY) 0,
               (OS_TICK) 0,
               (void *) 0,
               (OS_OPT)(OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR),
               (OS_ERR *) &err);
  
  //2
  OSTaskCreate((OS_TCB *)&AppTask2_TCB,
               (CPU_CHAR *)"App Task 2",
               (OS_TASK_PTR)AppTask2,
               (void *) 0,
               (OS_PRIO) 2,
               (CPU_STK *) &AppTask2_Stk[0],
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE/10,
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
               (OS_MSG_QTY) 0,
               (OS_TICK) 0,
               (void *) 0,
               (OS_OPT)(OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR),
               (OS_ERR *) &err);
  
  //3
  OSTaskCreate((OS_TCB *)&AppTask3_TCB,
               (CPU_CHAR *)"App Task 3",
               (OS_TASK_PTR)AppTask3,
               (void *) 0,
               (OS_PRIO) 3,
               (CPU_STK *) &AppTask3_Stk[0],
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE/10,
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
               (OS_MSG_QTY) 0,
               (OS_TICK) 0,
               (void *) 0,
               (OS_OPT)(OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR),
               (OS_ERR *) &err);
  
  //4
  OSTaskCreate((OS_TCB *)&AppTask4_TCB,
               (CPU_CHAR *)"App Task 4",
               (OS_TASK_PTR)AppTask4,
               (void *) 0,
               (OS_PRIO) 4,
               (CPU_STK *) &AppTask4_Stk[0],
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE/10,
               (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
               (OS_MSG_QTY) 0,
               (OS_TICK) 0,
               (void *) 0,
               (OS_OPT)(OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR),
               (OS_ERR *) &err);
  #if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);                                 /* Compute CPU capacity with no task running        */
#endif

  CPU_IntDisMeasMaxCurReset();
  
  GPIO_ResetBits(GPIOD,(GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_7));
  while(DEF_TRUE){
    OSTimeDlyHMSM(0,0,0,100,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  }
}


static void AppTask1(void *p_arg){
  OS_ERR err;
  p_arg = p_arg;
  while(1){
    //GPIO_SetBits(GPIOD,GPIO_Pin_2);
    OSTimeDlyHMSM(0,0,0,100,
                  (OS_OPT) OS_OPT_TIME_HMSM_STRICT,
                  (OS_ERR *)&err);
    //GPIO_ResetBits(GPIOD,GPIO_Pin_2);
    OSTimeDlyHMSM(0,0,0,100,
                  (OS_OPT) OS_OPT_TIME_HMSM_STRICT,
                  (OS_ERR *)&err);
  }
}
static void AppTask2(void *p_arg){
  OS_ERR err;
  p_arg = p_arg;
  u8 a = 0;
  while(1){
    a = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2); // down
    if(a){
      GPIO_SetBits(GPIOD,GPIO_Pin_3);
      GPIO_ResetBits(GPIOD, GPIO_Pin_2);
    }else{
      GPIO_ResetBits(GPIOD, GPIO_Pin_3);
      GPIO_SetBits(GPIOD,GPIO_Pin_2);
    }
    OSTimeDlyHMSM(0,0,0,200,
                  (OS_OPT) OS_OPT_TIME_HMSM_STRICT,
                  (OS_ERR *)&err);
  }
}
static void AppTask3(void *p_arg){
  OS_ERR err;
  p_arg = p_arg;
  
  while(1){
    GPIO_SetBits(GPIOD,GPIO_Pin_4);
    OSTimeDlyHMSM(0,0,0,500,
                  (OS_OPT) OS_OPT_TIME_HMSM_STRICT,
                  (OS_ERR *)&err);
    GPIO_ResetBits(GPIOD,GPIO_Pin_4);
    OSTimeDlyHMSM(0,0,0,500,
                  (OS_OPT) OS_OPT_TIME_HMSM_STRICT,
                  (OS_ERR *)&err);
  }
}
static void AppTask4(void *p_arg){
  OS_ERR err;
  p_arg = p_arg;
  
  while(1){
    GPIO_SetBits(GPIOD,GPIO_Pin_7);
    OSTimeDlyHMSM(0,0,1,0,
                  (OS_OPT) OS_OPT_TIME_HMSM_STRICT,
                  (OS_ERR *)&err);
    GPIO_ResetBits(GPIOD,GPIO_Pin_7);
    OSTimeDlyHMSM(0,0,1,0,
                  (OS_OPT) OS_OPT_TIME_HMSM_STRICT,
                  (OS_ERR *)&err);
  }
}
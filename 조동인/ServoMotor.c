#include <includes.h>

static  OS_TCB   AppTaskStartTCB;
static  OS_TCB   AppTask1_TCB; // Servo Motor

static  CPU_STK  AppTaskStartStk[APP_TASK_START_STK_SIZE]; 
static  CPU_STK  AppTask1_Stk[128];

static  void  AppTaskStart  	(void *p_arg); 
static  void  AppTask1   	(void *p_arg); 

void RCC_CNF(void);
void GPIO_CNF(void);
void Timer_CNF(void);
void delay(u32);

/* pwm_pulse : -90(600), 0(1500), 90(2400) */
void change_pwm_pulse(int);

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
    
    RCC_CNF();
    GPIO_CNF();
    Timer_CNF();
    
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

void RCC_CNF()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
}

void GPIO_CNF()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Timer_CNF()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; // Timer 설정  
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    // 50Hz Timer Clock(20ms)
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1; // SystemCoreClock = 72MHz
    TIM_TimeBaseStructure.TIM_Period = 20000-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM mode 설정  
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // TIM4 Chnnel3
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);
    
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void delay(u32 delay)
{
  while(delay--);
}

void change_pwm_pulse(int pwm_pulse) 
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pwm_pulse;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
}

static  void  AppTask1 (void *p_arg)
{
    //OS_ERR err;
    p_arg = p_arg;
    
    int pwm_pulse;
    
    while (1) {
      for(pwm_pulse = 600; pwm_pulse<=2400; pwm_pulse = pwm_pulse+900){ // -90도에서 90도까지 회전 
        change_pwm_pulse(pwm_pulse);
        delay(10000000);
      }
      delay(10000000);
    }
}

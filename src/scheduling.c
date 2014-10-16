
/**
  ******************************************************************************
  * @file    scheduling.c
  * @author  Xue Liu
  * @version V0.1
  * @date    08-15-2014
  * @brief   This file provide basic real-time tasks scheduling
  ******************************************************************************
  */

/**
  ******************************************************************************
  * Include files
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "blinky.h"
#include "sys_clk_init.h"
#include "lab04_tasks.h"
/**
  ******************************************************************************
  * Private function declaration
  ******************************************************************************
  */

void RCC_Configuration(void);
void Motor_GPIO_Configuration(void);
void Motor_TIM_Configuration(TIM_TypeDef *TIM_GROUP, uint16_t CCR1_Pre,
                             uint16_t CCR2_Pre);
void Timer_Configuration(TIM_TypeDef *TIM_GROUP, uint16_t Period);
void NVIC_Configuration(void);
/**
 *
  ******************************************************************************
  * Global Variables
  ******************************************************************************
  */
uint8_t TIM1_ISR_Counter1 = 0;
uint8_t TIM1_ISR_Counter2 = 0;
uint32_t TIM2_ISR_Counter = 0;
MotorSpeeds motor_speeds;
/**
 *
  ******************************************************************************
  * Public functions
  ******************************************************************************
  */

/**
 * @brief main
 * @details Main function
 */
void main(void) {
  /*Init System Clock using External Clock*/
  SystemInit_User();
  /* System Clocks Configuration */
  RCC_Configuration();

  /* GPIO Configuration */
  Motor_GPIO_Configuration();
  LED_Init(LED_GREEN);
  LED_Init(LED_RED);

  /* TIM Configuration */
  NVIC_Configuration();
  Timer_Configuration(TIM1, 20);
  Timer_Configuration(TIM2, 2000);
  while (1) {
    logDebugInfo();
  }
}
/**
  ******************************************************************************
  * Private functions
  ******************************************************************************
  */

/**
 * @brief Motor_RCC_Configuration
 * @details Enable the Clocks of the Four Motors
 */
void RCC_Configuration(void) {
  /* Tim clocks of motors enable */
  RCC_APB1PeriphClockCmd(
      RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM2, ENABLE);
  /* Tim clocks of general timer enable*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOB clock for motors and LEDs enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
}

/**
 * @brief Motor GPIO Configuration
 * @details Enable the gpio channel of the four motors
 */
void Motor_GPIO_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration:TIM3 Channel1, 2, 3 and 4 as alternate function
   * push-pull */
  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief NVIC configuration
 * @details enable Interrupt and set priorities
 */
void NVIC_Configuration(void) {

  NVIC_InitTypeDef NVIC_InitStructure;
  // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  /* enable TIM1 UP IRQ*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* enable TIM2 */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Motor TIM Configuration
 * @details configure the TIM3 and TIM 4 for 4 motors
 *
 * @param TIM_GROUP the TIM Group for the motors
 * @param CCR1_Pre Motor 2(TIM3) or 4 (TIM4)
 * @param CCR2_Pre Motor 1(TIM3) or 3 (TIM4)
 */
void Motor_TIM_Configuration(TIM_TypeDef *TIM_GROUP, uint16_t CCR1_Pre,
                             uint16_t CCR2_Pre) {
  /* -----------------------------------------------------------------------
  TIM Configuration: generate 4 PWM signals with 4 different duty cycles:
  The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM counter
  clock at 24 MHz the Prescaler is computed as following:
   - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
  SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
  and Connectivity line devices and to 24 MHz for Low-Density Value line and
  Medium-Density Value line devices

  The TIMx is running at 36 KHz: TIMx Frequency = TIMx counter clock/(ARR + 1)
                                                = 24 MHz / 666 = 36 KHz
  TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
  TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
  TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
  TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
----------------------------------------------------------------------- */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  uint16_t PrescalerValue = 0;
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t)(SystemCoreClock / 24000000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM_GROUP, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse =
      TIM_TimeBaseStructure.TIM_Period / 100 * CCR1_Pre;

  TIM_OC3Init(TIM_GROUP, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM_GROUP, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse =
      TIM_TimeBaseStructure.TIM_Period / 100 * CCR2_Pre;

  TIM_OC4Init(TIM_GROUP, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM_GROUP, TIM_OCPreload_Enable);

  /*TIM enable Group*/
  TIM_ARRPreloadConfig(TIM_GROUP, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM_GROUP, ENABLE);
}
/**
 * @brief Timer Configuration
 * @details Configure general timers
 *
 * @param TIM_GROUP the Tim Group to configure to general timers
 * @param Period the value of up registers
 */
void Timer_Configuration(TIM_TypeDef *TIM_GROUP, uint16_t Period) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_DeInit(TIM_GROUP);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = (36000 - 1);
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM_GROUP, &TIM_TimeBaseStructure);

  /* clear flag of the TIM Groupe*/
  TIM_ClearFlag(TIM_GROUP, TIM_FLAG_Update);

  /*TIM enable Group*/
  TIM_ITConfig(TIM_GROUP, TIM_IT_Update, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM_GROUP, ENABLE);
}
/**
  ******************************************************************************
  * ISR handlers
  ******************************************************************************
  */
/**
 * @brief TIM1 UP IRQHandler
 * @details IRQ Handler for TIM1
 */
void TIM1_UP_IRQHandler(void) {
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
    /*call the the task 1 called every 10ms*/
    detectEmergency();
    /* Use global counter to count and called every 100 ms*/
    TIM1_ISR_Counter1++;
    if (TIM1_ISR_Counter1 == 10) {
      /* Task2 called every 100 ms*/
      refreshSensorData();
      
      TIM1_ISR_Counter1 = 0;
    }
    TIM1_ISR_Counter2++;
    if (TIM1_ISR_Counter2 == 50) {
      /* Task2 called every 500 ms*/
      LED_Toggle(LED_RED);

      TIM1_ISR_Counter2 = 0;
    }
  }
}

/**
 * @brief TIM2 IRQHandler
 * @details IRQ Handler for TIM2
 */
void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);

    /* change mode after 10 secs */
    TIM2_ISR_Counter++;
    if(TIM2_ISR_Counter > 5 && TIM2_ISR_Counter < 25){
      GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    }
    if(TIM2_ISR_Counter > 25){
      GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, DISABLE);
    }

    /* Toggle the LED every 1000ms */
    LED_Toggle(LED_GREEN);
    /* Task3 called every 1000ms */
    calculateOrientation();
    /* Task 4 called every 1000ms */
    updatePid(&motor_speeds);
    /* Motor 2 and Motor 1 configuration */
    Motor_TIM_Configuration(TIM3, motor_speeds.m2 * 5, motor_speeds.m1 * 5);
    /* Motor 4 and Motor 3 configuration */
    Motor_TIM_Configuration(TIM4, motor_speeds.m4 * 5, motor_speeds.m3 * 5);
  }
}

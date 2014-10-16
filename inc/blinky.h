
/**
  ******************************************************************************
  * @file    blinky.h
  * @author  Xue Liu
  * @version V0.1
  * @date    08-16-2014
  * @brief   Provides LED definations
  ******************************************************************************
  */

/**
  ******************************************************************************
  * Definations
  ******************************************************************************
  */

#define LEDn 2

#define LED_RED_PIN GPIO_Pin_4
#define LED_RED_GPIO_PORT GPIOB
#define LED_RED_GPIO_CLK RCC_APB2Periph_GPIOB

#define LED_GREEN_PIN GPIO_Pin_5
#define LED_GREEN_GPIO_PORT GPIOB
#define LED_GREEN_GPIO_CLK RCC_APB2Periph_GPIOB
  
typedef enum { LED_RED = 0, LED_GREEN = 1 } Led_TypeDef;
/**
  ******************************************************************************
  * Public Functions
  ******************************************************************************
  */
void LED_On(Led_TypeDef LED_NUM);
void LED_Off(Led_TypeDef LED_NUM);
void LED_Toggle(Led_TypeDef LED_NUM);
void LED_Toggle(Led_TypeDef LED_NUM);

/**
  ******************************************************************************
  * @file    blinky.c
  * @author  Xue Liu
  * @version V0.1
  * @date    08-16-2014
  * @brief   Provides LED control functions
  ******************************************************************************
  */

/**
  ******************************************************************************
  * Include Files
  ******************************************************************************
  */

#include "blinky.h"
/**
  ******************************************************************************
  * Private function declaration
  ******************************************************************************
  */

/**
*
******************************************************************************
* Private Global Variables
******************************************************************************
*/

GPIO_TypeDef *LED_PORT[LEDn] = {
    LED_RED_GPIO_PORT, LED_GREEN_GPIO_PORT,
};
const uint16_t LED_PIN[LEDn] = {LED_RED_PIN, LED_GREEN_PIN};
const uint32_t LED_CLK[LEDn] = {LED_RED_GPIO_CLK, LED_GREEN_GPIO_CLK};
/**
  ******************************************************************************
  * Public functions
  ******************************************************************************
  */

/* Init a led
@param none
@retval none
*/
void LED_Init(Led_TypeDef LED_NUM) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED_CLK[LED_NUM], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED_PIN[LED_NUM];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(LED_PORT[LED_NUM], &GPIO_InitStructure);
}

/* LED_On
@brief turn the green led on
@param none
@retval none
*/
void LED_On(Led_TypeDef LED_NUM) { GPIOB->BRR = LED_PIN[LED_NUM]; }
/* LED_Off
@brief turn the green off
@param none
@retval none
*/
void LED_Off(Led_TypeDef LED_NUM) { GPIOB->BSRR = LED_PIN[LED_NUM]; }
/* LED_Toggle()
@param none
@retval none
*/
void LED_Toggle(Led_TypeDef LED_NUM) { GPIOB->ODR ^= LED_PIN[LED_NUM]; }
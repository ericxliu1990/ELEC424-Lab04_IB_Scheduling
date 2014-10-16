/*
A blinky file for Lab03-Blinky, ELEC424 Fall 2014
Board: Bitcraze 's crazyfle
Author: Xue Liu, Zichao Wang 
Drive: STM32F10x_StdPeriph_Lib_V3.5.0
*/

/* Include files*/
#include "blinky.h"
#include "sys_clk_init.h"

/*Public functions*/

/* Init a led 
@param none
@retval none
*/
void LED_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Enable the GPIO_LED Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* LED_On
@brief turn the green led on 
@param none
@retval none
*/
void LED_On()
{
	GPIOB->BRR = GPIO_Pin_5;
}
/* LED_Off
@brief turn the green off
@param none
@retval none
*/
void LED_Off()
{
	GPIOB->BSRR = GPIO_Pin_5;  
}
/* LED_Toggle()
@param none
@retval none
*/
void LED_Toggle()
{
	GPIOB->ODR ^= GPIO_Pin_5;
}
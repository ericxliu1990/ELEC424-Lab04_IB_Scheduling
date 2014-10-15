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

/*Private functions*/

/* main program
@param none
@retval none
*/
int main(void)
{
	SystemInit_User();
	LED_Init();
	
	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
		/* Capture error */ 
		while (1);
	}

	while (1)
	{
		LED_Toggle();

		/* Insert 1000 ms delay */
		_delay_(1000);
	}
}
/* variable defuation for helper functions*/
static __IO uint32_t Delaytime;
/* helper functions*/
/* delay
@brief: use nop operating to delay for N ms
@param: nCount for the time need to delay
@retval: none
*/
static void delay(__IO uint32_t nCount)
{
	__IO uint32_t index = 0; 
	for(index = (7500 * nCount); index != 0; index--)
	{
	}
}

/**
@brief  Inserts a delay time.
@param  nTime: specifies the delay time, unit(ms).
@retval None
  */
void Delay(__IO uint32_t nTime)
{ 
	Delaytime = nTime;

	while(Delaytime != 0);
}

/**
@brief  called by systick handler to decreae the counter
@param  None
@retval None
  */
void Delay_Handler(void)
{
	if (Delaytime != 0x00)
	{ 
		Delaytime--;
	}
}
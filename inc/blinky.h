/*
the blinky header file for Lab03-Blinky, ELEC424 Fall 2014
Board: Bitcraze 's crazyfle
Author: Xue Liu, Zichao Wang 
*/

/* LED definations 
#define LEDn                             2

#define LED_RED_PIN                               GPIO_Pin_4
#define LED_RED_GPIO_PORT                 GPIOB
#define LED_RED_GPIO_CLK                    RCC_APB2Periph_GPIOB
  
#define LED_GREEN_PIN                         GPIO_Pin_5
#define LED_GREEN_GPIO_PORT           GPIOB
#define LED_GREEN_GPIO_CLK              RCC_APB2Periph_GPIOB  
          
typedef enum 
{
	LED_RED = 0,
	LED_GREEN = 1
} Led_TypeDef;

GPIO_TypeDef* LED_PORT[LEDn] = {LED_RED_GPIO_PORT, LED_GREEN_GPIO_PORT,};
const uint16_t LED_PIN[LEDn] = {LED_RED_PIN, LED_GREEN_PIN};
const uint32_t LED_CLK[LEDn] = {LED_RED_GPIO_CLK, LED_GREEN_GPIO_CLK};
*/

/* Delay*/
#ifndef PROCISE_DELAY
	#define _delay_ Delay
#else
	#define _delay_ delay 
#endif

/* Public functions*/
void LED_Init();
void LED_On();
void LED_Off();
void LED_Toggle();
static void delay(__IO uint32_t nCount);
void Delay(__IO uint32_t nTime);
void Delay_Handler(void);
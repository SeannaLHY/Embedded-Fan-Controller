#include "stm32f7xx.h"                  // Device header
#include "Open746i_lcd.h"
#include "Open746i_TS.h"
#include <stdlib.h>
#include <stdio.h>

void SystemClock_Config(void);
extern GT911_Status TS_Data;

int main(void)
{
	SystemClock_Config();
	
	/*
	Now can use LCD functions
	*/
	BSP_LCD_Init();
	BSP_LCD_Clear(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(0, 0, (unsigned char *) "Test LCD Embedded Systems Spring 2023", CENTER_MODE);
	
	TS_Init();

  //GPIO B, F
  RCC->AHB1ENR |= (1<<1)|(1<<5);

  //TIM 3
	RCC->APB1ENR |= (1<<1);
	//TIM 10
	RCC->APB2ENR |= (1<<17);

  //PB5 in AF mode
	GPIOB->MODER |= (2<<10);
	//PF6 in AF mode
	GPIOF->MODER |= (2<<12);

  //PB5 AF mode 2 (connects to TIM3)
	GPIOB->AFR[0] |= (2<<20);
	//PF6 AF mode 3 (connects to TIM10)
	GPIOF->AFR[0] |= (3<<24);
  
  //TIM3_CH2 PWM mode
	TIM3->CCMR1 |= (7<<12); 
	TIM3->CCER |= (1<<4); 

  //TIM10_CH1 PWM mode
	TIM10->CCMR1 |= (7<<4); 
	TIM10->CCER |= (1<<0); 

  TIM3->PSC = 107;
	TIM3->ARR = 20749;
	TIM3->CCR2 = 20000;

  TIM10->PSC = 215;
	TIM10->ARR = 22249;
	TIM10->CCR1 = 20000;

  //start timers
	TIM3->CR1 |= (1<<0);
	TIM10->CR1 |= (1<<0);

	while(1)
	{

		if (TS_Data.Touch)
		{
			TS_Data.Touch = 0;
			
			BSP_LCD_SetTextColor(rand() & 0xFFFF);
			BSP_LCD_FillCircle(TS_Data.X[0], TS_Data.Y[0], 15);

      TIM3->ARR = 20749 + TS_Data.X[0] * (22250-20750) /1024;
	  	TIM10->ARR = 21299 + TS_Data.Y[0] * (21700-21300) /600;

			
		}
	}
}

void SysTick_Handler(void)
{
	HAL_IncTick();
}

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  HAL_PWREx_EnableOverDrive();
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
}


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
//#include "w3c_home.bmp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  TFT_24_7789_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	TFT_24_7789_brightness_test();
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
  LL_PWR_DisableOverDriveMode();
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 100, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_4);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(100000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(100000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3 
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, IM0_Pin|DC_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, RESET_Pin|CS_Pin|WR_Pin|RD_Pin);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3 
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = IM0_Pin|RESET_Pin|CS_Pin|WR_Pin 
                          |RD_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(DC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


//---------------------------------------------------------
/*
(c)2014 Curt Lagerstam - Newhaven Display International, LLC.

 	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
*/
//---------------------------------------------------------

/*******************************************************************************
* Function Name  : TFT_24_7789_Write_Command
* Description    : writes a 1 byte command to 2.4" TFT.
* Input          : command = one byte command (register address)
* Output         : None
* Return         : None
*******************************************************************************/
void TFT_24_7789_Write_Command(unsigned int command)
{
	LL_GPIO_ResetOutputPin(DC_GPIO_Port, DC_Pin);
	LL_GPIO_SetOutputPin(RD_GPIO_Port, RD_Pin);
	LL_GPIO_ResetOutputPin(WR_GPIO_Port, WR_Pin);
	LL_GPIO_WriteOutputPort(GPIOC, command);//when using 8-bit interface (DB17:10)
	LL_mDelay(10);
	LL_GPIO_SetOutputPin(WR_GPIO_Port, WR_Pin);
	LL_mDelay(1);
}

/*******************************************************************************
* Function Name  : TFT_24_7789_Write_Data
* Description    : writes 1 byte of data to 2.4" TFT.
* Input          : data1 = one byte of display data or command parameter
* Output         : None
* Return         : None
*******************************************************************************/
void TFT_24_7789_Write_Data(unsigned int data1)
{
	LL_GPIO_WriteOutputPort(GPIOC, data1);//when using 16-bit interface (DB17:10,DB8:1)//when using 8-bit interface (DB17:10)
	LL_GPIO_SetOutputPin(DC_GPIO_Port, DC_Pin);
	LL_GPIO_ResetOutputPin(WR_GPIO_Port, WR_Pin);
	LL_mDelay(10);
	LL_GPIO_SetOutputPin(WR_GPIO_Port, WR_Pin);
}

/*******************************************************************************
* Function Name  : TFT_24_7789_Init
* Description    : Initializes LCD with built-in ST7789S controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TFT_24_7789_Init(void)
{
	//configure interface, toggle RESET high, and set CS Low
	LL_GPIO_SetOutputPin(IM0_GPIO_Port, IM0_Pin);//8080 8-bit
	LL_GPIO_ResetOutputPin(CS_GPIO_Port, CS_Pin);//chip select
	LL_GPIO_SetOutputPin(RD_GPIO_Port, RD_Pin);
	LL_GPIO_ResetOutputPin(WR_GPIO_Port, WR_Pin);
	LL_GPIO_SetOutputPin(RESET_GPIO_Port, RESET_Pin);
	LL_mDelay(100);
	LL_GPIO_ResetOutputPin(RESET_GPIO_Port, RESET_Pin);
	LL_mDelay(100);

	//exit SLEEP mode
	TFT_24_7789_Write_Command(0x0011);
	LL_mDelay(100);

	//MADCTL: memory data access control
	TFT_24_7789_Write_Command(0x0036);TFT_24_7789_Write_Data(0x0080);

	//COLMOD: Interface Pixel format *** I use 262K-colors in 18bit/pixel
	//format when using 8-bit interface to allow 3-bytes per pixel
	TFT_24_7789_Write_Command(0x003A);TFT_24_7789_Write_Data(0x0006);//*** changed data from 0x0066 to 0x0006

	//PORCTRK: Porch setting
	TFT_24_7789_Write_Command(0x00B2);TFT_24_7789_Write_Data(0x000C);TFT_24_7789_Write_Data(0x0C);TFT_24_7789_Write_Data(0x00);TFT_24_7789_Write_Data(0x33);TFT_24_7789_Write_Data(0x33);

	//GCTRL: Gate Control
	TFT_24_7789_Write_Command(0x00B7);TFT_24_7789_Write_Data(0x0035);

	//VCOMS: VCOM setting
	TFT_24_7789_Write_Command(0x00BB);TFT_24_7789_Write_Data(0x002B);

	//LCMCTRL: LCM Control
	TFT_24_7789_Write_Command(0x00C0);TFT_24_7789_Write_Data(0x002C);

	//VDVVRHEN: VDV and VRH Command Enable
	TFT_24_7789_Write_Command(0x00C2);TFT_24_7789_Write_Data(0x0001);TFT_24_7789_Write_Data(0xFF);
	TFT_24_7789_Write_Command(0x00C3);TFT_24_7789_Write_Data(0x0011);//VRHS: VRH Set
	TFT_24_7789_Write_Command(0x00C4);TFT_24_7789_Write_Data(0x0020);//VDVS: VDV Set
	TFT_24_7789_Write_Command(0x00C6);TFT_24_7789_Write_Data(0x000F);//FRCTRL2: Frame Rate control in normal mode
	TFT_24_7789_Write_Command(0x00D0);TFT_24_7789_Write_Data(0x00A4);TFT_24_7789_Write_Data(0xA1);//PWCTRL1: Power Control 1
	TFT_24_7789_Write_Command(0x00E0);TFT_24_7789_Write_Data(0x00D0);
									  TFT_24_7789_Write_Data(0x0000);
									  TFT_24_7789_Write_Data(0x0005);
									  TFT_24_7789_Write_Data(0x000E);
									  TFT_24_7789_Write_Data(0x0015);
									  TFT_24_7789_Write_Data(0x000D);
									  TFT_24_7789_Write_Data(0x0037);
									  TFT_24_7789_Write_Data(0x0043);
									  TFT_24_7789_Write_Data(0x0047);
									  TFT_24_7789_Write_Data(0x0009);
									  TFT_24_7789_Write_Data(0x0015);
									  TFT_24_7789_Write_Data(0x0012);
									  TFT_24_7789_Write_Data(0x0016);
									  TFT_24_7789_Write_Data(0x0019);//PVGAMCTRL: Positive Voltage Gamma control
	TFT_24_7789_Write_Command(0x00E1);TFT_24_7789_Write_Data(0x00D0);
									  TFT_24_7789_Write_Data(0x0000);
									  TFT_24_7789_Write_Data(0x0005);
									  TFT_24_7789_Write_Data(0x000D);
									  TFT_24_7789_Write_Data(0x000C);
									  TFT_24_7789_Write_Data(0x0006);
									  TFT_24_7789_Write_Data(0x002D);
									  TFT_24_7789_Write_Data(0x0044);
									  TFT_24_7789_Write_Data(0x0040);
									  TFT_24_7789_Write_Data(0x000E);
									  TFT_24_7789_Write_Data(0x001C);
									  TFT_24_7789_Write_Data(0x0018);
									  TFT_24_7789_Write_Data(0x0016);
									  TFT_24_7789_Write_Data(0x0019);//NVGAMCTRL: Negative Voltage Gamma control
	TFT_24_7789_Write_Command(0x002A);TFT_24_7789_Write_Data(0x0000);TFT_24_7789_Write_Data(0x0000);TFT_24_7789_Write_Data(0x0000);TFT_24_7789_Write_Data(0x00EF);//X address set
	TFT_24_7789_Write_Command(0x002B);TFT_24_7789_Write_Data(0x0000);TFT_24_7789_Write_Data(0x0000);TFT_24_7789_Write_Data(0x0001);TFT_24_7789_Write_Data(0x003F);//Y address set

	LL_mDelay(10);
}



/*******************************************************************************
* Function Name  : TFT_24_7789_demo
* Description    : Writes a black and blue screen to NHD-2.4-240320CF-CTXI#.
* Input          : None
* Output         : None
* Return         : 1-end of function reached
*******************************************************************************/
int TFT_24_7789_demo(void)
{

	unsigned short RGB16[25600];
	uint8_t red,green,blue;



	//TFT_24S_Write_Command(0x002C);	//Memory write
	TFT_24_7789_Write_Command(0x002C); //memory write to ST7789

	for (int n=0;n<3;n++){
		memset(RGB16,0x0000,sizeof(RGB16));
		for (int i=0;i<25600;i++)					//for each 24-bit pixel...
		{
			//f_read(&filename, &blue, 1, &blen);	//read the blue 8-bits
			//f_read(&filename, &green, 1, &blen);	//read the green 8-bits
			//f_read(&filename, &red, 1, &blen);		//read the red 8-bits
			if(i > 12000) {
				red = 0;
				green = 0;
				blue = 255;
			}
			else {
				red = 0;
				green = 0;
				blue = 0;
			}

			LL_GPIO_SetOutputPin(DC_GPIO_Port, DC_Pin);

			LL_GPIO_WriteOutputPort(GPIOC, red);
			LL_GPIO_ResetOutputPin(WR_GPIO_Port, WR_Pin);
			LL_GPIO_SetOutputPin(WR_GPIO_Port, WR_Pin);

			LL_GPIO_WriteOutputPort(GPIOC, green);
			LL_GPIO_ResetOutputPin(WR_GPIO_Port, WR_Pin);
			LL_GPIO_SetOutputPin(WR_GPIO_Port, WR_Pin);

			LL_GPIO_WriteOutputPort(GPIOC, blue);
			LL_GPIO_ResetOutputPin(WR_GPIO_Port, WR_Pin);
			LL_GPIO_SetOutputPin(WR_GPIO_Port, WR_Pin);
		}
	}
	TFT_24_7789_Write_Command(0x0029);				//display ON
	return 1;
}

/*******************************************************************************
* Function Name  : TFT_24_7789_brightness_test
* Description    : Adjusts brightness of NHD-2.4-240320CF-CTXI#.
* Input          : None
* Output         : None
* Return         : 1-end of function reached
*******************************************************************************/
int TFT_24_7789_brightness_test(void)
{

	TFT_24_7789_Write_Command(0x0053);TFT_24_7789_Write_Data(0b101100);
	for(unsigned int i = 0;i < 200;i++){
		TFT_24_7789_Write_Command(0x0051);TFT_24_7789_Write_Data(i);
		LL_mDelay(5);
	}

	TFT_24_7789_Write_Command(0x0029);				//display ON
	return 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

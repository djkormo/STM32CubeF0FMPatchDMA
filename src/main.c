/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "resources.h"
#include "algorithm.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t ADC_raw[6];
uint16_t ADC_new[6];
uint16_t ADC_old[6];

volatile uint32_t R = 107374182;

volatile uint8_t countererror = 0;
volatile uint8_t counter = 0;
volatile uint8_t counteradc = 0;
volatile uint16_t counterdac = 0;
volatile uint16_t countertim6 = 0;
volatile uint16_t countertim15 = 0;

volatile uint16_t lutindex1 = 0;
volatile uint16_t lutindex2 = 0;
volatile uint16_t lutindex3 = 0;
volatile uint16_t lutindex = 0;

volatile uint16_t value1_dac = 0;
volatile uint16_t value2_dac = 0;
volatile uint16_t value3_dac = 0;
volatile uint16_t value_dac = 0;

volatile uint8_t index = 0;

volatile uint16_t counttimer1 = 0;
volatile uint16_t counttimer2 = 0;


// 1st sine
volatile uint32_t accumulator1 = 0;
volatile uint16_t accumulator1angle = 0;
volatile uint16_t accumulator1step = 0;

volatile uint32_t accumulator1r = 107374182;
volatile double VoltValue1 = 0.0;

//2nd sine
volatile uint32_t accumulator2 = 0;
volatile uint16_t accumulator2angle = 0;
volatile uint16_t accumulator2step = 0;
volatile uint32_t accumulator2r=107374182*2;
volatile double VoltValue2 = 0.0;

//3rd sine
volatile uint32_t accumulator3 = 0;
volatile uint16_t accumulator3angle = 0;
volatile uint16_t accumulator3step = 0;
volatile uint32_t accumulator3r=107374182*4;
volatile double VoltValue3 = 0.0;


volatile uint16_t ADC_lookup1 =5;
volatile uint16_t ADC_lookup2 =10;
volatile uint16_t ADC_lookup3 =20;
int useLeds = 0;
int useDAC  = 1;
int useADC =  0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM15_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  if (useADC)
  {
	  MX_DMA_Init();
	  MX_ADC_Init();
	  HAL_ADC_Start_DMA(&hadc, ADC_raw, 6);
  }
 // if (useDAC)
  {
	  MX_DAC_Init();
	  MX_TIM6_Init();

	  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) {
	  		Error_Handler();
	  	}

	  	if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK) {
	  		Error_Handler();
	  	}

	  	MX_TIM15_Init();

  }
  /* USER CODE BEGIN 2 */
 // start DMA


  // use DAC


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
   //counter++;
   //HAL_Delay(10);
   /*
   ADC_lookup1++;
   if (ADC_lookup1>=50)
   {
	   ADC_lookup1=1;
   }
	*/
   /*
   ADC_lookup1= (uint16_t) ADC_raw[0]/(100.0);
   ADC_lookup2= (uint16_t) ADC_raw[1]/(100.0);
   ADC_lookup3= (uint16_t) ADC_raw[2]/(100.0);
	*/
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
	//RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
      Error_Handler();
    }

    /**Configure the Systick interrupt time 
    */
   HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
   HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank =1;// ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

	/* ADC Calibration */

	if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) {
		Error_Handler();
	}

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  //sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

void TIM6_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}



/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
  //htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);

	HAL_NVIC_EnableIRQ(TIM6_IRQn);

  // start TIM6 interrput
	if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
		Error_Handler();
	}


}

void TIM15_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim15);

}



/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 50;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 20;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_NVIC_SetPriority(TIM15_IRQn, 0, 0);

	HAL_NVIC_EnableIRQ(TIM15_IRQn);

  // start TIM15 interrput
	if (HAL_TIM_Base_Start_IT(&htim15) != HAL_OK) {
		Error_Handler();
	}


}


/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> TSC_G1_IO3
     PA3   ------> TSC_G1_IO4
     PA6   ------> TSC_G2_IO3
     PA7   ------> TSC_G2_IO4
     PB0   ------> TSC_G3_IO2
     PB1   ------> TSC_G3_IO3
     PB10   ------> I2C2_SCL
     PB11   ------> I2C2_SDA
     PB13   ------> SPI2_SCK
     PB14   ------> SPI2_MISO
     PB15   ------> SPI2_MOSI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin 
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin 
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin 
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_SCL_Pin I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = I2C2_SCL_Pin|I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_SCK_Pin SPI2_MISO_Pin SPI2_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI2_SCK_Pin|SPI2_MISO_Pin|SPI2_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) //check if the interrupt comes from TIM6

	{

		countertim6++;
		counterdac++;

		if (counterdac%50000==0)
				{
					if (useLeds)
					{
						HAL_GPIO_TogglePin(GPIOC, LD6_Pin); // Blue

					}

					if (counterdac%2000==0)
					{
					for (int i=0;i<=2;i++)
						{

							 ADC_new[i]=(0.9)*ADC_old[i]+(0.1)*ADC_raw[i];
							 ADC_old[i]=ADC_new[i];

						}


						/*


	    	    		 accumulator1r=(uint32_t)10737418*
	    	    	  	  		rangeScaleLinear(ADC_new[0],0,4095,1,100);

	    	    		  accumulator2r=(uint32_t)10737418*
	    	    	  	  		rangeScaleLinear(ADC_new[1],0,4095,1,100);

	    	    		  accumulator3r=(uint32_t)107374182*
	    	    	  	  		rangeScaleLinear(ADC_new[2],0,4095,1,10);
						*/
					}


					accumulator1r-=10737418;
					/*
					accumulator2r+=10737418*2;
					accumulator3r-=10737418;
					*/
				}

		// output for DAC


			/*	based on
		    http://amarkham.com/?p=49
		   */


		accumulator1+=accumulator1r;
		//  first 10 (32 -22) bits -> lut table index
		accumulator1angle=(uint16_t)(accumulator1>>22);
		accumulator1step = Sine1024_12bit[accumulator1angle]/(3);

		/*
		accumulator2+=accumulator2r;
		//  first 10 (32 -22) bits -> lut table index
		accumulator2angle=(uint16_t)(accumulator2>>22);
		accumulator2step = Sine1024_12bit[accumulator2angle]/(3);
		*/
		/*
		accumulator3+=accumulator3r;
		//  first 10 (32 -22) bits -> lut table index
		accumulator3angle=(uint16_t)(accumulator3>>22);
		accumulator3step = Sine1024_12bit[accumulator3angle]/(3);
		*/
		/*
		lutindex+=50;
		if (lutindex>=1023)
		{
			lutindex-=1024;
		}

		accumulator1step= Sine1024_12bit[lutindex];
		*/


		//value_dac=(accumulator1step+accumulator1step+accumulator1step)/(3.0);
		//value_dac=(uint16_t) (accumulator1step+accumulator1step+accumulator1step)/3.0;
		//value_dac=accumulator1step+accumulator2step+accumulator3step;
		value_dac=accumulator1step+accumulator2step+accumulator3step;
		//value_dac=accumulator1step;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value_dac);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value_dac);
	}

	if (htim->Instance == TIM15) //check if the interrupt comes from TIM15
	{
		  countertim15++;
		  /*
		   ADC_lookup1= (uint16_t) ADC_raw[0]/(100.0);
		   ADC_lookup2= (uint16_t) ADC_raw[1]/(100.0);
		   ADC_lookup3= (uint16_t) ADC_raw[2]/(100.0);
		*/
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
	 HAL_GPIO_WritePin(GPIOC, LD5_Pin,GPIO_PIN_SET); // Green
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

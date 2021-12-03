 /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f3xx_it.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void proccesDmaData(const uint8_t* sign, uint8_t length);
/* USER CODE END 0 */
uint8_t is_dollar = 0;
uint8_t is_manual = 0;
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

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  USART2_RegisterCallback(proccesDmaData);

  uint8_t tx_data[500];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  uint8_t buffer_mem_occupied = DMA_USART2_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6);

	 	  float total_load = (float)buffer_mem_occupied/DMA_USART2_BUFFER_SIZE*100;

	 	  USART2_PutBuffer(tx_data, sprintf((char*)tx_data, "Buffer capacity: %d bytes, occupied memory: %d bytes, load [in %%]: %3.2f , pwm intensity: %d \n\r",\
	 	  								  DMA_USART2_BUFFER_SIZE,buffer_mem_occupied, total_load,intensity));
	 	  LL_mDelay(3000);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

uint8_t pwm_intensity;

void proccesDmaData(const uint8_t* sign,uint8_t length)
{
	uint8_t cutted_str[8];
	uint8_t pwm_found = 0,str_pos = 0,pwm_pos = 0;
	uint8_t pwm_intensity_str[2];



	for(int i = 0; i< length; i++){


		if(*(sign+i) == '$'){
			is_dollar = 1;
		}

		if(is_dollar){
			cutted_str[str_pos] = *(sign+i);
			str_pos++;

			if(strstr(cutted_str,"$auto$")){
				is_manual = 0;
				memset(cutted_str,0,strlen(cutted_str));
				str_pos = 0;
				is_dollar = 0;
			}
			else if(strstr(cutted_str,"$manual$")){
				pwm_intensity = intensity;
				is_manual = 1;
				memset(cutted_str,0,strlen(cutted_str));
				str_pos = 0;
				is_dollar = 0;
			}

			if (pwm_found){
				if(pwm_pos == 2){
					if(cutted_str[str_pos-1] == '$'){
						sscanf(pwm_intensity_str, "%d", &pwm_intensity);
					}
					memset(pwm_intensity_str,0,strlen(pwm_intensity_str));
					pwm_found = 0;
					pwm_pos = 0;
					is_dollar = 0;
					str_pos = 0;
				}
				else if(cutted_str[str_pos-1]>= '0' && cutted_str[str_pos-1] <= '9'){
					pwm_intensity_str[pwm_pos] = cutted_str[str_pos-1];
					pwm_pos ++;

				}
			}
			if(is_manual && (strstr(cutted_str,"$PWM"))){
				pwm_found = 1;
			}
		}

		else {
			memset(cutted_str,0,strlen(cutted_str));
		}

	}


	return;
}

void setDutyCycle(uint8_t D){
	uint8_t pulse_length = 0;
	pulse_length = ((TIM2->ARR) * D) / 100;
	TIM2->CCR1 = pulse_length;
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

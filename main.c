/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADF4351.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WAIT_LOCK_COUNT  295
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//扫频参数
uint32_t count=0, count1=0, NowFre=0;
uint32_t SweepMinFre = 90000;
uint32_t SweepMaxFre = 110000;
uint32_t extend_SweepMinFre = 90700;
uint32_t extend_SweepMaxFre = 110700;
uint32_t SweepStepFre = 100;
uint16_t SweepTime = 1000;//ms
uint8_t SweepFlag = 0;

uint8_t extend_part = 0;
uint8_t lock_time_flag=0;
uint8_t lock_time=0;
uint8_t state=0;

//串口屏参数
uint8_t RxBuffer [2];    //
uint8_t RxData;
uint8_t pRxBuffer;
uint8_t RxFlag = 0;

int16_t first_ADC_flag = -1;//第一次进入ADC采样标志
uint16_t ADC_Buffer[201];
uint8_t bar_Buffer[201];
uint8_t pADC_Buffer;

uint16_t ADC_value;
uint16_t ADC_noise = 0x2FF;//
uint8_t bar_value;//进度条

uint8_t count_spurious;//杂散波个数
uint16_t spuriousThreshold;//*0.02

uint16_t peak_freq;//最大幅度对应频率
uint16_t peak;//最大幅度
uint8_t  index_peak_freq;//最大幅度对应下标

uint32_t temp =0 ;
uint32_t temp_NowFre = 0;
uint16_t v_PLL = 230;//PLL初始输出
double log_v_PLL ;//log10PLL初始输出

uint8_t v_input = 10;//默认输出10mV

float max_dB = 20.5f;
float min_dB = -37.50f;

float max_v = 1400.0f;
float min_v = 100.0f;

float gain;//dB

//AGC参数
float v0=100;          
uint16_t Voltage = 10;  //单位MV


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//扫频TIM中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==(&htim2))
	{
    if(SweepFlag)
		{
			count ++;
      if(count >= SweepTime / 20)
      {  
        count = 0;        
        if(!extend_part)
        {
          NowFre = SweepMinFre+SweepStepFre*count1;
					count1 ++;
          if(NowFre==SweepMaxFre) 
          {
            count1 = 0;
            NowFre = SweepMaxFre;
          }
          
          ADF4351WriteFreq((double)NowFre/1000);
          temp_NowFre = NowFre / 100;
          printf("x0.val=%d\xFF\xFF\xFF",temp_NowFre);
        }
        else
        {   			 
//					SweepFlag = 0;
            NowFre = extend_SweepMinFre + SweepStepFre * count1;
						count1 ++;
            if(NowFre == extend_SweepMaxFre) 
            {
              count1 = 0;
              NowFre = extend_SweepMaxFre;
            }
            //
            ADF4351WriteFreq((double)NowFre/1000);
            first_ADC_flag = WAIT_LOCK_COUNT;   
        }
      }
		}

    if(lock_time_flag==1)
    {
      lock_time++;
    }
		if(first_ADC_flag != -1)
    {
      if(first_ADC_flag != 0)
      {
        first_ADC_flag --;
      }
      else
      {
        first_ADC_flag = -1;
				uint16_t ADC_temp[8];
				for(int i = 0; i < 8; i++){
					HAL_ADC_Start(&hadc1);
					HAL_ADC_PollForConversion(&hadc1,5);
					HAL_ADC_Stop(&hadc1);
					ADC_temp[i] = HAL_ADC_GetValue(&hadc1);
				}
				uint16_t ADC_temp_max = 0x00;
				uint16_t ADC_temp_min = 0x2FFF;
				uint16_t ADC_temp_sum = 0x00;
				for(int i = 0; i < 8; i++){
					ADC_temp_sum += ADC_temp[i];
					if(ADC_temp[i] > ADC_temp_max){
						ADC_temp_max = ADC_temp[i];
					}
					if(ADC_temp[i] < ADC_temp_min){
						ADC_temp_min = ADC_temp[i];
					}
				}
				ADC_value = (ADC_temp_sum - ADC_temp_min - ADC_temp_max) / 6;

//        HAL_ADC_Start(&hadc1);
//        HAL_ADC_PollForConversion(&hadc1,5);
//        HAL_ADC_Stop(&hadc1);
//        ADC_value= HAL_ADC_GetValue(&hadc1);
				
				if(ADC_value >= ADC_noise)
				{
					ADC_value -= ADC_noise;
				}
				else
				{
					ADC_value  = 0;
				}
        bar_value = ADC_value / 41 ;
        ADC_Buffer [pADC_Buffer] = ADC_value;

        printf("j%d.val=%d\xFF\xFF\xFF", pADC_Buffer, bar_value);

        pADC_Buffer=pADC_Buffer + 1;

				
        if(pADC_Buffer == 201)
        {
					//SweepFlag=0;
          pADC_Buffer = 0;
          peak = 0;

          for(uint8_t i =0;i<201;i++)
          {
            if(ADC_Buffer[i] > peak)
            {
              index_peak_freq = i;
              peak = ADC_Buffer[i];
            }
          }

          spuriousThreshold = peak * 0.02f;
          count_spurious = 0;

          for(uint8_t i =0;i<201;i++)
          {
            if(ADC_Buffer[i] > spuriousThreshold)
            {
              count_spurious ++;
            }
          }
          
          printf("x0.val=%d\xFF\xFF\xFF", 800 + index_peak_freq);
          printf("n0.val=%d\xFF\xFF\xFF", count_spurious-1);
          }
      }
    }

  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
	{
		RxFlag = 1;  
	}
  HAL_UART_Receive_IT(&huart1, &RxData, 1);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5) == GPIO_PIN_SET)
	{
		lock_time_flag=0;
		printf("x2.val=%d\xFF\xFF\xFF",lock_time);
		__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_5);
		  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	}
}

	
int fputc(int ch, FILE *f)
{      
	 HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);    
	return ch;
}



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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	ADF4351Init();
  ADF4351WriteFreq(80.0f);
  NowFre = 80000;
	
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1,&RxData, 1);
	//DAC
  log_v_PLL= log10(v_PLL); 
  gain = 20.0f * (log10(v_input) - log_v_PLL);//dB  
  v0 = min_v + ((gain - min_dB) * (max_v - min_v))/(max_dB - min_dB);  
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v0*4096/3300);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(RxFlag)  
    {
      RxFlag = 0;
      switch (state)
      {
        case 0:
        {
          pRxBuffer = 0;
          if(RxData == 0x55)
          {
            state = 1;
          }
          else
          {
            state = 0;
          }
          break;
        }
        case 1:
        {
          if(RxData == 0x00)
          {
            state = 2;
          }
          else if(RxData == 0x01)
          {
            state = 3;
          }
          else if(RxData == 0x02)
          {
            state = 4;
						
          }
          else if(RxData == 0x09)
          {
            state = 5;
          }
          else if(RxData == 0x04)
          {
            state = 6;
          }
          else if(RxData == 0x05)
          {
            state = 7;
          }
          else if(RxData == 0x06)
          {
            state = 8;
          }
          else if(RxData == 0x07)
          {
            state = 9;
          }
          else if(RxData == 0x08)
          {
            state = 10;
          }
					else if(RxData == 0x10)
          {
            state = 11;
          }
          else 
          {
            state = 0;
          }
          break;
        }
        case 2:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
          if(pRxBuffer == 2)
          {
            state = 0;
            Voltage = RxBuffer[0];
            v_input = Voltage ;
            gain = 20.0f * (log10(v_input) - log_v_PLL);//dB  
            v0 = min_v + ((gain - min_dB) * (max_v - min_v))/(max_dB - min_dB);          
            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v0*4095/3300);
            HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
          }
          break;
        }
        case 3:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
          if(pRxBuffer == 2)
          {
            state = 0;
            SweepTime = RxBuffer[0] * 100;
          }
          break;

        }
        case 4:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
					HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
          if(pRxBuffer == 2)
          {
						lock_time_flag=1;
						lock_time = 0;
            state = 0;
            SweepFlag = 0;
            temp =  (RxBuffer[1] <<8 )| RxBuffer[0];
            NowFre=temp * 100;
            ADF4351WriteFreq((float)temp / 10.0f);
          }
          break;
        }
        case 5:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
          if(pRxBuffer == 2)
          {
            state = 0;
            SweepFlag = 1;
						count1 = 0;
          }
          break;
        }
        case 6:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
          if(pRxBuffer == 2)
          {
            state = 0;
            if(NowFre >=110000)
            {  
              NowFre = 90000;
            }
            else
            {  
              NowFre += 100;
            }
            temp_NowFre = NowFre / 100;
            printf("x0.val=%d\xFF\xFF\xFF",temp_NowFre);
            SweepFlag = 0;
            ADF4351WriteFreq(((float)NowFre) / 1000.0f);
          }
          break;
        }
        case 7:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
          if(pRxBuffer == 2)
          {
            state = 0;
            SweepFlag = 1;
            extend_part = 1;
						pRxBuffer = 0;
						pADC_Buffer = 0;
            first_ADC_flag = -1;
						count1=0;
						
						v_input = 10;
            gain = 20.0f * (log10(v_input) - log_v_PLL);//dB  
            v0 = min_v + ((gain - min_dB) * (max_v - min_v))/(max_dB - min_dB);          
            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v0*4095/3300);
            HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
						
						SweepTime = 6000;
          }
          break;
        }
        case 8:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
          if(pRxBuffer == 2)
          {
            state = 0;
            extend_part = 0;
						SweepFlag = 0;
          }
          break;
        }
        case 9:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
          if(pRxBuffer == 2)
          {
            state = 0;
            //关闭自动扫频
            SweepFlag = 0;
            //发送当前频率
            temp_NowFre = NowFre / 100;
            printf("x0.val=%d\xFF\xFF\xFF",temp_NowFre);
          }
          break;
        }
        case 10:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
          if(pRxBuffer == 2)
          {
            state = 0;
            //关闭自动扫频
            SweepFlag = 0;
            //发送当前频率
            temp_NowFre = NowFre / 100;
            printf("x0.val=%d\xFF\xFF\xFF",temp_NowFre);
          }
          break;
        }
				case 11:
        {
          RxBuffer[pRxBuffer] = RxData;
          pRxBuffer ++;
          if(pRxBuffer == 2)
          {
            state = 0;
            //关闭自动扫频
            SweepFlag = 0;
          }
          break;
        }
        default:
          break;
      }
  }
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

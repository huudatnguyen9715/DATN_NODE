
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "SX1278.h"
#include "ds18b20_mflib.h"
#include "dwt_stm32_delay.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;



/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t time_us;
uint32_t temp=0;
uint16_t adc_value[3];
float volt[3],pH,DO;

int pH_ng , temperature_ng  ;
float pH_tp , temperature_tp ;

int pH_ng_temp = 13 , temperature_ng_temp = 28 ;
float pH_tp_temp = 0.27  , temperature_tp_temp = 0.65 ;
int index_rx ;
char buffer_temp[10];

int m;
int master;
int ret;

char buffer_rx[64];
char buffer[64];
char buffer_rx[64];
int id = 1 ;
int tam=22;
int message_length;

uint8_t ID_1 = 1 ;
//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

uint8_t addr=0;//de ktra dia chi I2C cua cam bien
uint8_t BH1750_ADDR=0x23;// dia chi I2C cua cam bien 
uint8_t MODE= 0x10; // High resolution mode ( do phan giai 1 lux);
uint8_t buffer_lux_8[2] ={0,0};
uint16_t buffer_lux_16;
float lux; // gia tri do sang
float lux_set=5;

//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
		float  temperature_data;
		float humidity_data; 
		
		uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, Sum;
		uint16_t Tempperatue, Humidity , TemptoTest = 30.12;
		
		int check;
		int ck;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//........................................

void delay_u (uint16_t delay) 
	{
	__HAL_TIM_SET_COUNTER(&htim2, 0);   
		while(__HAL_TIM_GET_COUNTER(&htim2) < delay);  // 1 periodic = 1 us
		
		}
	

		
void Set_Pin_Output( GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//***************************************************************************************************

void Set_Pin_Input( GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**************************DS18B20_function******************/	
/*
#define DS18B20_PORT GPIOA
#define DS18B20_PIN  GPIO_PIN_9	
	uint8_t DS18B20_Start(void)
		{
			uint8_t Response=0;
			Set_Pin_Output(DS18B20_PORT,DS18B20_PIN);  // set pin output 
//			HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_RESET); // pull pin low
			delay_u(500); //delay according to datasheet
			
			Set_Pin_Input(DS18B20_PORT,DS18B20_PIN); //set pin input
			delay_u(69); //delay according to datasheet
			
			if(!HAL_GPIO_ReadPin(DS18B20_PORT,DS18B20_PIN)) //if the pin is low the presene pulse is detected
				Response=1;
			else
				Response=0;
			
			delay_u(431); // delay 480u total
			
			return Response;
		}
		void DS18B20_Write(uint8_t data)
		{
			Set_Pin_Output(DS18B20_PORT,DS18B20_PIN);  //set as output
			for(int i=0; i<8;i++)
			{
				if((data & (1<<i)) != 0)  // if bit high "bit 1"
				{
					//write 1
					
					Set_Pin_Output(DS18B20_PORT,DS18B20_PIN);  //set as output
					HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_RESET); // pull pin low
					delay_u(2);  //wait for 2 us
					
					Set_Pin_Input(DS18B20_PORT,DS18B20_PIN); //set as input
					delay_u(65); // wait for 65 us
				}
				else
				{
					//write 0
					
					Set_Pin_Output(DS18B20_PORT,DS18B20_PIN);  //set as output
					HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_RESET); // pull pin low
					delay_u(65);  //wait for 65 us
					
					Set_Pin_Input(DS18B20_PORT,DS18B20_PIN); //set as input
					delay_u(2);  //wait for 2 us
				}
			}
		}
		
		uint8_t DS18B20_Read (void)
		{
			uint8_t value=0;
			
			Set_Pin_Input(DS18B20_PORT,DS18B20_PIN);
			
			for(int i=0;i<8;i++)
			{
				Set_Pin_Output(DS18B20_PORT,DS18B20_PIN); //set as output
				
				HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_RESET); //pull datapin low
				delay_u(2); //delay 2us
				
				HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_SET);
				delay_u(7);
				Set_Pin_Input(DS18B20_PORT,DS18B20_PIN);// set as input
				if(HAL_GPIO_ReadPin(DS18B20_PORT,DS18B20_PIN)) //if pin is high
				{
					value |= (1<<i); //read bit 1
				}
				delay_u(55);
			}
			return value;
		}
	
		void DS18B20_data(void)
			{
				DS18B20_Start();
				DS18B20_Write(0xCC);//skip room
				DS18B20_Write(0x44);//convert t
				
				HAL_Delay(10);
				DS18B20_Start();

				DS18B20_Write(0xCC); //skip room
				DS18B20_Write(0xBE);	//read Scratch-pad

				Temp_byte1=DS18B20_Read(); 
				Temp_byte2=DS18B20_Read(); 
				Tempperatue=((uint16_t)Temp_byte2<<8)|Temp_byte1;		
				temperature_data=(float)Tempperatue/16;
				
										
			}
			*/
//********************************************************************************************
	
	
	
//........................................
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);


//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
//{
//micros *= (SystemCoreClock / 1000000) / 9	;
///* Wait till done */
//while (micros--) ;
//}



__STATIC_INLINE void Delay(__IO uint32_t s)
{
s *= SystemCoreClock / 9	;
while (s--) ;
}



/* USER CODE BEGIN PFP */
/* Private function prototypes ----	-------------------------------------------*/

/* USER CODE END PFP */


/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;
	#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE 
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);


 return ch;
}


GPIO_InitTypeDef GPIO_InitStruct;

void gpio_set_input (void)
{
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


void gpio_set_output (void)
{
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

unsigned char temp_l, temp_h;
uint16_t temp_test;
float temperature;


uint8_t ds18b20_init (void)
{
	gpio_set_output ();   // set the pin as output
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4, 0);  // pull the pin low
	DWT_Delay_us (480);   // delay according to datasheet

	gpio_set_input ();    // set the pin as input
	DWT_Delay_us (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4)))    // if the pin is low i.e the presence pulse is there
	{
		DWT_Delay_us (400);  // wait for 400 us
		return 0;
	}

	else
	{
		DWT_Delay_us (400);
		return 1;
	}
}

void write (uint8_t data)
{
	gpio_set_output ();   // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			gpio_set_output ();  // set as output
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4, 0);  // pull the pin LOW
			DWT_Delay_us (1);  // wait for  us

			gpio_set_input ();  // set as input
			DWT_Delay_us (60);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			gpio_set_output ();
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4, 0);  // pull the pin LOW
			DWT_Delay_us (60);  // wait for 60 us

			gpio_set_input ();
		}
	}
}


uint8_t read (void)
{
	uint8_t value=0;
	gpio_set_input ();

	for (int i=0;i<8;i++)
	{
		gpio_set_output ();   // set as output

		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4, 0);  // pull the data pin LOW
		DWT_Delay_us (2);  // wait for 2 us

		gpio_set_input ();  // set as input
		if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		DWT_Delay_us (60);  // wait for 60 us
	}
	return value;
}

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
	DWT_Delay_Init();
	
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) adc_value,3);
	HAL_ADC_Start_IT(&hadc1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */
	
	
	

	//initialize LoRa module
	SX1278_hw.dio0.port = DIO0_GPIO_Port;
	SX1278_hw.dio0.pin = DIO0_Pin;
	SX1278_hw.nss.port = NSS_GPIO_Port;
	SX1278_hw.nss.pin = NSS_Pin;
	SX1278_hw.reset.port = RESET_GPIO_Port;
	SX1278_hw.reset.pin = RESET_Pin;
	SX1278_hw.spi = &hspi1;

	SX1278.hw = &SX1278_hw;

	printf("Configuring LoRa module\r\n");
	
	SX1278_begin(&SX1278, SX1278_433MHZ, SX1278_POWER_17DBM, SX1278_LORA_SF_8,
			SX1278_LORA_BW_20_8KHZ, 10);
	
	printf("Done configuring LoRaModule\r\n");

	if (master == 1) {
		ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);

	} else {
		ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
	}

	///////

	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

		
  /* USER CODE BEGIN 3 */
		
		
		check = ds18b20_init ();
		write (0xCC);  // skip ROM
		write (0x44);  // convert t

		HAL_Delay (800);

		ds18b20_init ();
		write (0xCC);  // skip ROM
		write (0xBE);  // Read Scratchpad

		temp_l = read();
		temp_h = read();
		temp = (temp_h<<8)|temp_l;
		temperature = (float)temp/16;
		
		temperature_ng = (int)temperature;
		temperature_tp = temperature - temperature_tp;	
   
		
		HAL_ADC_Start_IT(&hadc1);
			if(master ==0) {
				
			ret = SX1278_LoRaRxPacket(&SX1278);

			if (ret > 0) {
				SX1278_read(&SX1278, (uint8_t *) buffer_rx, ret);
				
				
				if(buffer_rx[0] == '1')
				{
						if(buffer_rx[1] != buffer_temp[0])
							{
					if(buffer_rx[1]=='1')	
						{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
	
					}else if(buffer_rx[1]=='0')
					{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);	
					}
						buffer_temp[0]=buffer_rx[1];
				}
					master = 1 ;
				}
			}
		

		}
///******************************************
		
		if (master == 1) 
		{
//					for(int i=0;i<5;i++)
//					{
//								DS18B20_data();
//								HAL_Delay(5);
//					}
//				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//				HAL_Delay(100);
//				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			
		//	message_length = sprintf(buffer, "%d%d",ID_1,Tempperatue);
				message_length = sprintf(buffer, "%d%d",ID_1,temperature_ng);

				
			//message_length = sprintf(buffer, "NODE:%2d pH: %d.%2.0f  DO: %d.%2.0f ",message,pH_ng,pH_tp*100,DO_ng,DO_tp*100);
			ret= SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
								
			for(int i=0;i<3;i++)
			{
			ret= SX1278_LoRaTxPacket(&SX1278, (uint8_t *) buffer, message_length,2000);
							HAL_Delay(200);
			}
				
				master = 0 ;
				ret= SX1278_LoRaEntryRx(&SX1278, 16, 2000);

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
/*I2C init function*/

/* ADC1 init function */

static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
 
/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65534;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TXRX_PORT, TXRX_PIN, GPIO_PIN_RESET);
	
	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	
	/*Configure GPIO pin : TXRX */

	 GPIO_InitStruct.Pin = TXRX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TXRX_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO0_Pin  */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pins :  MODE_Pin */
  GPIO_InitStruct.Pin = MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	
//	if(hadc->Instance==ADC1)
//	{
//		volt[0]=(float)(adc_value[0]*3.3)/4095;
//		pH=volt[0]*140/29-56/29;
//		pH_ng=(int)(pH);
//		pH_tp= pH-pH_ng;
//		
//		volt[1]=(float)(adc_value[1]*3.3)/4095;
//		DO=volt[1];
//		DO_ng=(int)(DO);
//		DO_tp= DO-DO_ng;
//		
//		volt[2]=(float)(adc_value[2]*3.3)/4095;
//	}
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
				
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

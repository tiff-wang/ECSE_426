/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "math.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#define SEG7_A GPIO_PIN_7			//Pin PE7  brown cable
#define SEG7_B GPIO_PIN_8			//Pin PE8  brown cable
#define SEG7_C GPIO_PIN_9			//Pin PE9  yellow cable
#define SEG7_D GPIO_PIN_10		//Pin PE10 yellow cable
#define SEG7_E GPIO_PIN_11		//Pin PE11 gray cable
#define SEG7_F GPIO_PIN_12		//Pin PE12 gray cable
#define SEG7_G GPIO_PIN_13		//Pin PE13 green cable
#define SEG7_DP GPIO_PIN_14		//Pin PE14 green cable

#define SEG7_OUT1 GPIO_PIN_2		//Pin PE2 white cable
#define SEG7_OUT2 GPIO_PIN_4		//Pin PE4 red cable
#define SEG7_OUT3 GPIO_PIN_5		//Pin PE5 brown cable
#define SEG7_OUT4 GPIO_PIN_6		//Pin PE6 white cable 
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int x[] = {0, 0, 0, 0, 0};
float b[] = {0.2, 0.2, 0.2, 0.2, 0.2};
volatile int sysTickFlag;
volatile int displayMode = 0;
volatile int debounce = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
void FIR_C(int input, float *output);
void plot_point(float, float *);
int blueButtonPressed();
void displayLEDValue(int number, int position);
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();

  /* USER CODE BEGIN 2 */
	int adc_val;
	float filtered_val = 0.0;
	
	HAL_ADC_Start_IT(&hadc1);
	
	// Give a initial DAC value
	int dac_val = 0x12;
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dac_val); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float results[3];
  while (1)
  {
		if(debounce > 0){
			debounce = (debounce + 1) % 500;
		}	
		displayLEDValue((int)(results[displayMode] * 100) % 100, 2);
		displayLEDValue((int)(results[displayMode] * 10) % 10, 3);
		displayLEDValue((int)results[displayMode], 4);
		//when in sysTick
			if (sysTickFlag == 1){
				sysTickFlag = 0;
			
				HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
				switch(displayMode) {
					case 0:
						HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
						break;
					case 1:
						HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
						break;
					case 2:
						HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
						break;
					default:
						break;
				}
				
				//acquire value from ADC 
				adc_val = HAL_ADC_GetValue(&hadc1);
				printf("%f\n", adc_val);
				FIR_C(adc_val, &filtered_val);
				float voltage_reading = 3.3 * filtered_val / 255.0;
				plot_point(voltage_reading, results);
				printf("RMS: %f", results[0]);
				printf("min: %f", results[1]);
				printf("max: %f", results[2]);
		}
				
		}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
			 50 Hz sampling frequency
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/50);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 Initialize blue button*/
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure LED GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);
	
	//7-segment display
	GPIO_InitStruct.Pin = SEG7_A | SEG7_B | SEG7_C | SEG7_D | SEG7_E | SEG7_F | SEG7_G | SEG7_DP | SEG7_OUT1 | SEG7_OUT2 | SEG7_OUT3 | SEG7_OUT4 ; 	// initialize the 7-segment display for the LEDs pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //set pins to output push pull mode
	GPIO_InitStruct.Pull = GPIO_NOPULL; // no pull for 7-segment pins
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM; //Speed of pin
	
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); // Initiate the pins with the setting from GPIO_INIT
	// when initialize, set all the pins to LOW
	HAL_GPIO_WritePin(GPIOE, SEG7_A | SEG7_B | SEG7_C | SEG7_D | SEG7_E | SEG7_F | SEG7_G | SEG7_DP | SEG7_OUT1 | SEG7_OUT2 | SEG7_OUT3 | SEG7_OUT4, GPIO_PIN_RESET); 


  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */


void plot_point(float input, float* output) {
	static int count = 0;
	static float min_val = 10.0;
	static float max_val = 0.0;
	static float rms_counter = 0.0;
	if(count == 0) {
		min_val = input;
		max_val = input;
		rms_counter = input * input;
	} else {
		if(input < min_val) min_val = input;
		else if(input > max_val) max_val = input;
		rms_counter += input * input;
	}
	count = (count + 1) % 200;
	output[0] = sqrt(rms_counter / count);
	output[1] = min_val;
	output[2] = max_val;
}


void FIR_C(int Input, float *Output) {
	// shifting 
	int j;
	for(j = 0; j < 4; ++j){
		x[j] = x[j+1];
	}
	// add new element in the end
	x[4] = Input;
	
	float out = 0.0;
	int i;
	for(i = 0; i < 5; ++i) {
		out += x[i] * b[4 - i];
	}
	*Output = out;
}

//allow to display the value on a segment display depending on the number and position
//parameter number provide which number going to be display on the 
void displayLEDValue(int number, int position){
	// set a specific value to set all the segment pins to low 
	if (number == -1){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_RESET);
	}
	
	else if (number == 1){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_RESET);
	}
	
	else if (number == 2){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);	
	}
	
	else if (number == 3){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);		
	}
	
	else if (number == 4){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);
	}
	
	else if (number == 5){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);	
	}
	
	else if (number == 6){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);		
	}
	
	else if (number == 7){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_RESET);		
	}
	
	else if (number == 8){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);
	}
	
	else if (number == 9){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_SET);	
	}
	
	else if (number == 0){
			HAL_GPIO_WritePin(GPIOE, SEG7_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG7_G , GPIO_PIN_RESET);
			
	}
	
	// Set which display to display the number.
	if (position == -1){
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_RESET);
	}
	
	else if (position == 1){
	  HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_RESET);
	}
	
	else if (position == 2){
	  HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_SET);
	}
	
	else if (position == 3){
	  HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_RESET);
	}
	
	else if (position == 4){
	  HAL_GPIO_WritePin(GPIOE, SEG7_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG7_OUT4 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG7_DP , GPIO_PIN_RESET);
	}	
	
	
	int k;
	for (k = 0; k<=5000; k++)
	{}
		
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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

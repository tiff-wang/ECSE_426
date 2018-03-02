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
#include "stm32f4xx_hal.h"
#include "voltmeter.h"
#include "math.h"

/* USER CODE BEGIN Includes */
#include "keypad.h"


/* 7-SEGMENT DISPLAY OUTPUT PINS - GPIOE */ 
#define SEG_A GPIO_PIN_7			
#define SEG_B GPIO_PIN_8			
#define SEG_C GPIO_PIN_1 //GPIOB			
#define SEG_D GPIO_PIN_10			
#define SEG_E GPIO_PIN_11		
#define SEG_F GPIO_PIN_12			
#define SEG_G GPIO_PIN_13		
#define SEG_DP GPIO_PIN_14		
#define SEG_OUT1 GPIO_PIN_2		
#define SEG_OUT2 GPIO_PIN_4		
#define SEG_OUT3 GPIO_PIN_5		
#define SEG_OUT4 GPIO_PIN_6	



#define SYSTICK_FREQUENCY 50
#define ADC_RES 8

/* CALCULATE DESIRED PWM_PERIOD USING
 *      PWM_PERIOD = 84MHz / Desired_Freq
 */

#define PWM_PERIOD 168 // 500kHz

/* CALCULATE DESIRED PRESCALER USING
 *      PRESCALER = (84MHz / Desired_Freq) + 1
 *      PRESCALER < 2^16 (65536)
 */

#define TIM2_PRESCALER 84000
#define TIM2_PERIOD 10

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
enum State {Wait, Output, Sleep};`
enum State state = Wait;

volatile int debounce = 0;
int debounce_mod = 600;

volatile int sysTickFlag;
volatile int sample_counter = 0;
int x[] = {0, 0, 0, 0, 0};

/* FIR Coefficients */
float coeff[5] = {0.2, 0.2, 0.2, 0.2};
int coeff_len = 5;

/* RMS tracker */
float rms_counter = 0.0;
float rms[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int voltage = 0;

float duty_cycle = 1.0;


/* Counter to wait for update before updating the ADC */
int count = 0;

/* STAR key variables */
int key_star_counter = 0 ;
int star_flag = 0 ; //flag that star was previously pressed
int star_release_debounce = 0 ; //avoid release misreadings

/* display digits */
int first_digit, second_digit, third_digit;


float res_filter = 0.0;


/* Duty Cycle calculation weights (From Linear Regression) */
//float w0 = 0.01500013;
//float w1 = 2.84490909;
float w1 = 1.26965858;
float w0 = 0.16727964;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void display(int number, int position);
int get_key(void);
int update_voltage(int digit, int action);
void adjust_pwm(int voltage);

/* USER CODE END PFP */

// for storing adc voltage value
int adc_voltage;

int holding = 0;
int hold_count = 0;


int main(void)
{
	int results = 0;
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();

	HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	// Start TIM2 Timer
	HAL_TIM_Base_Start(&htim2);
	// Start ADC
	HAL_ADC_Start_IT(&hadc1);
	/* USER CODE END 2 */

	/* Infinite loop */
	int key = -1;
	state = Wait;
	 while (1)
  {

		if(debounce > 0){
			debounce = (debounce + 1) % debounce_mod;
		}
		
		/* Display the voltage on the LED screen */
		display(first_digit, 2);
		display(second_digit, 3);
		display(third_digit, 4);
		
  /* USER CODE END WHILE */
    MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */
	if(sysTickFlag == 1){
		sysTickFlag = 0;
		key = get_key();
		
		/* Always check for STAR inputs */
		if(key != STAR && star_flag == 1) {
			if(star_release_debounce < 20){ 
				star_release_debounce++;
			}
			else{
				star_flag = 0;
                
                /* STAR KEY pressed for 1-2 seconds (medium press)*/
				if(key_star_counter > 5 && key_star_counter <  15){
					state = Wait;
                    
					first_digit = 0 ;
					second_digit = 0 ;
					third_digit = 0 ;

                    // set output voltage to 0
                    adjust_pwm(0);
					
					printf("Go to state Wait");
				} 
				key_star_counter = 0;
				star_release_debounce = 0;
			}
		}
		if(key == STAR){
			star_release_debounce = 0;
			key_star_counter++;
            
            /* STAR KEY quickly pressed (short press)*/
			if (state == Wait){
				voltage = update_voltage(0, 1);
			}
			if(star_flag == 1){
				 /* STAR KEY pressed for over 3 seconds (long press)*/
				if (key_star_counter >  15){
				state = Sleep;
                    
                // set output voltage to 0
                adjust_pwm(0);
                
				printf("Go to state Sleep");
				}
			}
			star_flag = 1;
		}
		
        switch(state){
            case Wait:
                /* update voltage on input */
                if(key > -1 && key < 20){
                    voltage = update_voltage(key, 0);
                    printf("voltage : %d \n", voltage);
                }
                /* enter voltage */
                else if(key == POUND){
                    results = voltage;
                    printf("voltage entered: %d \n", results);
                    adjust_pwm(voltage);
                    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                    printf("duty-cycle:%f \n", duty_cycle);
                    state = Output;
                    count = 0 ;
                    voltage = 0 ;
                }
                break;
            
            case Output:
               /* if (count < 40) {
                    count++;
                }
                else{
                    printf("Adc_voltage read : %d \n", adc_voltage);
                    count = 0;
                }
                */
                /* Do nothing and wait until reset to Wait mode */
                break;
                

            case Sleep:
                /* Update the digits to -1 (turn off) */
                first_digit = -1 ;
                second_digit = -1 ;
                third_digit = -1 ;
                voltage = 0 ;
                break;
            
            default:
                break;
        
			}
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/SYSTICK_FREQUENCY);

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
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES; // changed from 3 to 28 cycles, read more into why
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
	htim2.Init.Prescaler = TIM2_PRESCALER;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = TIM2_PERIOD; // Period count per trigger
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = PWM_PERIOD;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = duty_cycle * PWM_PERIOD;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);
}

/** Configure pins as 
 * Analog 
 * Input 
 * Output
 * EVENT_OUT
 * EXTI
 PC3   ------> I2S2_SD
 PA4   ------> I2S3_WS
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

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
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

	/*Configure digit selector pins*/
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*KEYPAD COL GPIO PINS */
    /*Configure GPIO pins : BOOT1_Pin PB12 PB13 PB14
     PB15 */
    GPIO_InitStruct.Pin = BOOT1_Pin|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
    |GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /*KEYPAD ROW GPIO PINS */
    /*Configure GPIO pins : PD8 PD9 PD10 PD11
     OTG_FS_OverCurrent_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
    |OTG_FS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
		//7-segment display
	GPIO_InitStruct.Pin = SEG_A | SEG_B | SEG_D | SEG_E | SEG_F | SEG_G | SEG_DP | SEG_OUT1 | SEG_OUT2 | SEG_OUT3 | SEG_OUT4 ; 	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //push pull mode
	GPIO_InitStruct.Pull = GPIO_NOPULL; // no pull cause already push pull
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; 
	
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); 
	HAL_GPIO_WritePin(GPIOE, SEG_A | SEG_B | SEG_D | SEG_E | SEG_F | SEG_G | SEG_DP | SEG_OUT1 | SEG_OUT2 | SEG_OUT3 | SEG_OUT4, GPIO_PIN_RESET); 

    
	GPIO_InitStruct.Pin = SEG_C ;	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //push pull mode
	GPIO_InitStruct.Pull = GPIO_NOPULL; // no pull cause already push pull
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; 
	
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
	HAL_GPIO_WritePin(GPIOB, SEG_C, GPIO_PIN_RESET); 


	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */

/**
 * @brief  FIR Filter takes in integer inputs serially and returns the filtered data (output)
 * @param  integer input, output address (pointer to index in output array)
 * @retval None
 */
void FIR_C(int input, float *output) {
  // Shift all x to the left
  for(int i = 0 ; i < 4 ; i++){
      x[i] = x[i + 1];
  }
  
  //update x[4] with new input
  x[4] = input;
  
  
  //calculate output
  *output = 0 ;
  for(int i = 0 ; i < 5 ; i++){
      *output = *output + x[i] * coeff[i];
  }
}

/**
 * @brief  Update PWM pulse duty cycle to get desired voltage
 * @param  int voltage
 * @retval None
 */

void adjust_pwm(int voltage) {
	duty_cycle = (voltage / 100.0 - w0) / w1;
    
    // 0 < duty_cycle < 1
	if(duty_cycle < 0.0) duty_cycle = 0.0;
	else if(duty_cycle > 1.0) duty_cycle = 1.0;
	
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.Pulse = duty_cycle * PWM_PERIOD;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}



/**
 * @brief  ADC Converter - function activated at every trigger (by timer2)
 *          function also does the voltage updates and computations
 * @param  ADC_HandleTypeDef hadc
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
	int adc_value = HAL_ADC_GetValue(&hadc1);
	FIR_C(adc_value, &res_filter);
	adc_voltage = 300 * res_filter / ((1 << ADC_RES) - 1);
	
	for(int i = 0 ; i < 9 ; i++){
      rms[i] = rms[i + 1];
  }
	rms[9] = adc_voltage * adc_voltage;
	rms_counter = rms_counter + 1.0;
	float sum = 0;
	
	for(int k = 0 ; k < 10 ; k++){
		sum += rms[k];
	}
	count = (count + 1) % 500;
	adc_voltage = sqrt(sum / ((rms_counter < 10) ? rms_counter : 10));
    /* Update the display digits */
    first_digit = (adc_voltage / 100 ) % 10 ;
    second_digit = (adc_voltage / 10) % 10 ;
    third_digit = adc_voltage % 10 ;
}

/**
 * @brief  This function updates the voltage
 *          action 1: delete last digit
 *          action 0: add new digit (max three digits)
 * @param  int digit, int action
 * @retval int new voltage
 */

int update_voltage(int digit, int action){
	//deleted the last digit
	if(action == 1){
		return voltage / 10;
	}
	printf("%d \n",  (voltage * 10 + digit) % 1000);
	return (voltage * 10 + digit) % 1000;
}

/**
 * @brief  This function is sets the LED displays
 * @param  int number, int position
 * @retval None
 */
void display(int number, int position){
	// set a specific value to set all the segment pins to low 
	if (number == -1){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_RESET);
	}
	
	else if (number == 1){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_RESET);
	}
	
	else if (number == 2){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_SET);	
	}
	
	else if (number == 3){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_SET);		
	}
	
	else if (number == 4){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_SET);
	}
	
	else if (number == 5){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_SET);	
	}
	
	else if (number == 6){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_SET);		
	}
	
	else if (number == 7){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_RESET);		
	}
	
	else if (number == 8){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_SET);
	}
	
	else if (number == 9){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_SET);	
	}
	
	else if (number == 0){
			HAL_GPIO_WritePin(GPIOE, SEG_A , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_B , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEG_C , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_D , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_E , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_F , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEG_G , GPIO_PIN_RESET);
			
	}
	
	// Set which display to display the number.
	if (position == -1){
		HAL_GPIO_WritePin(GPIOE, SEG_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_DP , GPIO_PIN_RESET);
	}
	
	else if (position == 1){
	  HAL_GPIO_WritePin(GPIOE, SEG_OUT1 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_DP , GPIO_PIN_RESET);
	}
	
	else if (position == 2){
	  HAL_GPIO_WritePin(GPIOE, SEG_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT2 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_DP , GPIO_PIN_SET);
	}
	
	else if (position == 3){
	  HAL_GPIO_WritePin(GPIOE, SEG_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT3 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT4 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_DP , GPIO_PIN_RESET);
	}
	
	else if (position == 4){
	  HAL_GPIO_WritePin(GPIOE, SEG_OUT1 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT2 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT3 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SEG_OUT4 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, SEG_DP , GPIO_PIN_RESET);
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

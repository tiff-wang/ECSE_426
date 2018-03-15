#include "main.h"
#include "LED.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

void init_LED(){
	GPIO_InitTypeDef GPIO_InitStruct;
	
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
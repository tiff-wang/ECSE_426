#include "main.h"
#include "keypad.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

extern int debounce ;


uint16_t row_pins[NUM_ROW] = {KEYPAD_ROW1, KEYPAD_ROW2, KEYPAD_ROW3, KEYPAD_ROW4};
uint16_t col_pins[NUM_COL] = {KEYPAD_COL1, KEYPAD_COL2, KEYPAD_COL3, KEYPAD_COL4};

//Keypad Matrix
const int matrix[4][4]={
													{1,4,7,STAR},
													{2,5,8,0},
													{3,6,9,POUND},
													{3,6,9,POUND}
												};


/**
 * @brief  This function reads the pressed key
 * @param  None
 * @retval int: returned key pressed or -1 (if no key pressed)
 */
int get_key(){
	int reading = -1;
    
    /* Add a debouncer to avoid multi reading of a single press
     * the debouncer_mod value can be changed in main.c
     */
	if(debounce == 0){
		debounce++;
        
        /* output current at each column and row inputs until detection of active low */
		for(int col = 0; col < NUM_COL && reading == -1; col++){
			switch (col){
				case 0:
					HAL_GPIO_WritePin(KEYPAD_COL_GPIO, col_pins[0], GPIO_PIN_SET);
					HAL_GPIO_WritePin(KEYPAD_COL_GPIO, col_pins[1], GPIO_PIN_RESET);
					HAL_GPIO_WritePin(KEYPAD_COL_GPIO, col_pins[2], GPIO_PIN_RESET);
					break;
				case 1:
					HAL_GPIO_WritePin(KEYPAD_COL_GPIO, col_pins[0], GPIO_PIN_RESET);
					HAL_GPIO_WritePin(KEYPAD_COL_GPIO, col_pins[1], GPIO_PIN_SET);
					HAL_GPIO_WritePin(KEYPAD_COL_GPIO, col_pins[2], GPIO_PIN_RESET);
					break;
				case 2:
					HAL_GPIO_WritePin(KEYPAD_COL_GPIO, col_pins[0], GPIO_PIN_RESET);
					HAL_GPIO_WritePin(KEYPAD_COL_GPIO, col_pins[1], GPIO_PIN_RESET);
					HAL_GPIO_WritePin(KEYPAD_COL_GPIO, col_pins[2], GPIO_PIN_SET);
					break;
				default: 
					break;
			}
            /* Read the rows to */
			for (int i = 0; i < NUM_ROW; i++){
				if(KEYPAD_ROW_GPIO->IDR & row_pins[i]){
					reading = matrix[col][i];
					break;
				}
			}
		}
	}
	return reading;
}


/**
 * @brief  This function initiates the KEYPAD and the respective GPIO pins
 * @param  None
 * @retval None
 */

void init_keypad(void){
  GPIO_InitTypeDef GPIO_InitStruct;
	
  /*Configure GPIO pins for Columns*/
  GPIO_InitStruct.Pin = KEYPAD_COL1|KEYPAD_COL2|KEYPAD_COL3|KEYPAD_COL4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEYPAD_COL_GPIO, &GPIO_InitStruct);
	
  /*Configure GPIO pins for Row*/
  GPIO_InitStruct.Pin = KEYPAD_ROW1|KEYPAD_ROW2|KEYPAD_ROW3|KEYPAD_ROW4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEYPAD_ROW_GPIO, &GPIO_InitStruct);

}

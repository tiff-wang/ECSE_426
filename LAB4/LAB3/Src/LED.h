#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

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


void init_keypad(void);
void display(int number, int position);
void reset_display();
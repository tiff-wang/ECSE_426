#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

#define KEYPAD_COL_GPIO GPIOB
#define KEYPAD_ROW_GPIO GPIOD
#define KEYPAD_COL1 GPIO_PIN_12 //GPIOB
#define KEYPAD_COL2 GPIO_PIN_13 //GPIOB
#define KEYPAD_COL3 GPIO_PIN_14 //GPIOB
#define KEYPAD_COL4 GPIO_PIN_15 //GPIOB
#define KEYPAD_ROW1 GPIO_PIN_8  //GPIOD
#define KEYPAD_ROW2 GPIO_PIN_9  //GPIOD
#define KEYPAD_ROW3 GPIO_PIN_10 //GPIOD
#define KEYPAD_ROW4 GPIO_PIN_11 //GPIOD
#define STAR 20
#define POUND 21
#define NUM_ROW 4 
#define NUM_COL 4

/*
 * remove for now because COL_3 and COL_4 seem to intefere. 
#define A 10
#define B 11
#define C 12
#define D 13
#define e 14
 */
 

int get_key(void);
void init_keypad(void);

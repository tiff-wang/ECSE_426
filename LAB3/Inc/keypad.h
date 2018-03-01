#ifndef KEYPAD
#define KEYPAD

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "math.h"
#include "segment_display.h"
#include <stdio.h>
#include <ctype.h>

#define KEYPAD_GPIO GPIOE
#define ALL_ROW_PINS GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10
#define ALL_COL_PINS GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
#define ROW_1_PINS GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10
#define ROW_2_PINS GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10
#define ROW_3_PINS GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_10
#define ROW_4_PINS GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9
#define COL_1_VALUE 0xE
#define COL_2_VALUE 0xD
#define COL_3_VALUE 0xB
#define COL_4_VALUE 0x7
#define NUM_ROWS 4
#define NUM_COLS 4
#define ENTER '#'
#define DUMMY_KEY 'x'
#define KEY_SCAN_FREQ 4
#define KEY_SCAN_CLK_DIV (TIM3_DESIRED_RATE / KEY_SCAN_FREQ)

void keypad_init(void);
uint8_t read_cols(void);
int get_key(char * key);
int get_target_angle(volatile uint_fast16_t * ready);

#endif
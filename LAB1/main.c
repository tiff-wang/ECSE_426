#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"                   // ARM::CMSIS:DSP

void FIR_C(int, float *);
int c_math(float *, float *, int);
int cmsis_math(float32_t *input, float32_t *output, uint32_t length);
int asm_math(float *input, float *output, int length);
void printResult (float *output, char *type);

int x[] = {0, 0, 0, 0, 0};
float b[] = {0.1, 0.15, 0.5, 0.15, 0.1};

int main()
{
	// testing
	int size = 10;

	int input[size];
	int c;
	for(c = 0; c < size; ++c) {
		input[c] = c;
	}
	float filtered[size];
	for(c = 0; c < size; ++c) {
		FIR_C(input[c], filtered + c);
	}
	
	float output[5];
	
	asm_math(filtered, output, size);
	printResult(output, "asm_math");
	c_math(filtered, output, size);
	printResult(output, "c_math");
	cmsis_math(filtered, output, size);
	printResult(output, "cmsis_math");
	
	return 0;
}

void printResult (float *output, char *type){
	printf("%s\nRMS: %f\nMax value: %f\nMin value: %f\nMax index: %f\nMin index: %f\n", type, output[0], output[1], output[2], output[3], output[4]);
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

int c_math(float *input, float *output, int length) {
	int c;
	float rms = input[0] * input[0];
	int min_index = 0;
	int max_index = 0;
	for(c = 1; c < length; ++c) {
		rms += input[c] * input[c];
		if(input[c] < input[min_index]) {
			min_index = c;
		} else if (input[c] > input[max_index]) {
			max_index = c;
		}
	}
	rms = sqrt(rms/length);
	output[0] = rms;
	output[1] = input[max_index];
	output[2] = input[min_index];
	output[3] = (float)max_index;
	output[4] = (float)min_index;
	return 0;
}

int cmsis_math(float32_t *input, float32_t *output, uint32_t length) {	
	float32_t rms;
	float32_t min_value;
	float32_t max_value;
	uint32_t min_index; 
	uint32_t max_index;
	
	// arm rms function
	arm_rms_f32(input, length, &rms); 
	// arm min function
	arm_min_f32(input, length, &min_value, &min_index); 
	// arm  max function
	arm_max_f32(input, length, &max_value, &max_index);
	
	// output order: rm; max; min; max index; min index
	output[0] = rms;
	output[1] = max_value;
	output[2] = min_value;
	output[3] = max_index;
	output[4] = min_index;
	return 0; 
}

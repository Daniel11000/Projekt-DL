#include <L298N.h>
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

#define SENSORS_OFFSET 50

void left_motor_drive(int PWM, char command[]){

	PWM %= 101;

	if(!strcmp(command, "FORWARD")){
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
	}else if(!strcmp(command, "BACKWARD")){
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
	}else if(!strcmp(command, "FAST STOP")){
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
	}else if(!strcmp(command, "SOFT STOP")){
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	}
}

void right_motor_drive(int PWM, char command[]){

	PWM %= 101;

	if(!strcmp(command, "FORWARD")){
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM);
	}else if(!strcmp(command, "BACKWARD")){
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM);
	}else if(!strcmp(command, "FAST STOP")){
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100);
	}else if(!strcmp(command, "SOFT STOP")){
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	}
}

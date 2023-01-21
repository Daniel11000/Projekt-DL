/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "L298N.h"
#include <stdlib.h>
#include "ir.h"
#include <stdbool.h>

//#include "PID_function.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//parametry PID
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN -100000.0f
#define PID_LIM_MAX  100000.0f

#define PID_LIM_MIN_CALKA -5.0f
#define PID_LIM_MAX_CALKA 5.0f
#define OKRES_PROBKOWANIA 0.01f

#define N_VAL   64

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {

	//wzmocnienie bloków
	float Kp;
	float Ki;
	float Kd;

	//stała czasowa FDP
	float tau;

	//ograniczenia wyjść
	float limMin;
	float limMax;

	//ograniczenia całki
	float limMinCalka;
	float limMaxCalka;

	//czas próbkowania
	float T;

	//pamięć
	float calka;
	float poprzedniBlad;			/* Required for integrator */
	float pochodna;
	float poprzedniPomiar;		/* Required for differentiator */

	//wyjście
	float wyjscie;

} KontrolerPID;


// Variables for encoder
int counter = 0;

// Initial motor speeds
const uint8_t leftInitSpeed = 70;
const uint8_t rightInitSpeed = 70;

// Variables to keep track of intersection
int numIntersection = 0;
bool intersection = 0;
// Variables for PID Control
int previousError = 0;
int PIDvalue = 0;
int error = 0;
int P = 0;
int I = 0;
int D = 0;
// Gain constants
///*
int Kp = 20;  // proportional
int Ki = 15;  // integral
int Kd = 10;  // derivate
//*/


int16_t ADCdiff = 0;
volatile static int16_t ADCvalue[2];



#define IR_ZERO 14  // (A0)
#define IR_ONE 15  // (A1)
#define IR_TWO 16  // (A2)
#define IR_THREE 17  // (A3)
#define IR_FOUR 18  // (A4)




int adcX = 75;
int basic_speed = 0;
int l_speed;
int r_speed;



///////////////////////////////////////////////////// PID  /////////////////////////////////////////////////////////////
//return error
int returnError(int ADCdiff )
{ int err;


  //Only inner right sensor is on black
  	  	  	  	  	  // Tylko prawy sensor widzi czarne
  if(ADCdiff < (-1 * adcX) )
  { //err = -2;
  err = -1;
  basic_speed = 10;
    }


  //Only middle sensor is on black
  //oba sensory widzą czarne
  else if( (ADCdiff > adcX) && (ADCdiff < (-1 * adcX) ) )
  { err = 0;
  basic_speed = 40;
   }


  // Inner left sensor is on black
  	  	  	  	  	  // Tylko lewy sensor widzi czarne
  else if(ADCdiff > adcX)
  { //err = 2;
  err = 1;
  basic_speed = 10;
    }


  // INTERSECTION (All sensors are on black)
  	  	  	  	  	  // Oba sensory widzą czarne
  else if( (ADCdiff > adcX) && (ADCdiff < (-1 * adcX) ) )
  { intersection = true;
    err = 0;
    basic_speed = 40;
   }
  // Return error
  return err;}

//calculates and returns PID error
int calcPID(int error, int previousError)
{
  P = error;
  I = I + error;
  D = error-previousError;
  int pid = (Kp*P) + (Ki*I) + (Kd*D);
  printf("\t\t\tStary PID = %i \n", pid);
 // pid = pid % 11;
 // pid = pid % 7;
  pid = pid % 19;
  return pid;
};

// set motor speed based on PID error
void setLeftRightSpeed(int PIDval)
{ int rightSpeed = rightInitSpeed + PIDval;

//  int leftSpeed = leftInitSpeed - PIDval; // ----------------------------- ???????????????????
	int leftSpeed = leftInitSpeed + PIDval;


  rightSpeed = rightSpeed % 100;
  leftSpeed = leftSpeed % 100;

  //rightSpeed = 70;
  //leftSpeed = 70;

  rightSpeed = rightSpeed % 79;
  leftSpeed = leftSpeed % 79;

  if(rightSpeed < 65)
  {
	  rightSpeed = 65;
  }

  if(leftSpeed < 65)
    {
	  leftSpeed = 65;
    }

  printf("\t\t\tPID = %i \n", PIDval);

  if(ADCdiff > adcX)		// - skret w lewo -
  {
	  printf("ADCdiff = %i \n", ADCdiff);
	  printf("Right Speed = %i \n", rightSpeed);
	  printf("Left Speed = %i \n", leftSpeed);
	  	   // Swiecenie na Zielono
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	  // ////////////////////////////////////////////////////
	  left_motor_drive(50, "SOFT STOP");
	  right_motor_drive(rightSpeed, "FORWARD");
	  // ////////////////////////////////////////////////////
  }
  else if(ADCdiff < (-1 * adcX))	// - skret w prawo -
  {
	  printf("ADCdiff = %i \n", ADCdiff);
	  printf("Right Speed = %i \n", rightSpeed);
	  printf("Left Speed = %i \n", leftSpeed);
	  	   // Swiecenie na Czerwono
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  // ////////////////////////////////////////////////////
	  left_motor_drive(leftSpeed, "FORWARD");
	  right_motor_drive(50, "SOFT STOP");
	  // ////////////////////////////////////////////////////
  }
  else		// - jazda prosto -
  {
	  printf("ADCdiff = %i \n", ADCdiff);
	  printf("Right Speed = %i \n", rightSpeed);
	  printf("Left Speed = %i \n", leftSpeed);
	  	   // Swiecenie na Zolto
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//	  left_motor_drive(85,"FORWARD");
//	  right_motor_drive(85,"FORWARD");
	  left_motor_drive(70,"FORWARD");
	  right_motor_drive(70,"FORWARD");
  }



  if(rightSpeed <= 0)
  {
	  rightSpeed = 10;
  }

  if(leftSpeed <= 0)
  {
	  leftSpeed = 10;
  }



};
// /*
// increments intersection after intersection is detected
void countIntersection()
{ while (intersection)
  { forward();  // Should you delete this line of code?
    if (!(digitalRead(IR_ZERO) && digitalRead(IR_FOUR)))
    { numIntersection+=1;  // put in return error code?
      break;
    }
  }
}
// */
// follows straight line
void followLine()
{ //Read Sensor
  int * ADCdiff = ADCvalue[0] - ADCvalue[1];
  //Error for PID
  error = returnError(ADCdiff);
  // PID Calculation
  PIDvalue = calcPID(error, previousError);
  previousError = error;
  // Set motor speeds based on error
  setLeftRightSpeed(PIDvalue);

};//end followLine()










int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

typedef enum {
	ON,
	OFF
}state_t;

volatile state_t state = OFF;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BLUE_BUTTON_Pin) {
	  if(state == ON){
		  state = OFF;
	  }else{
		  state = ON;
	  }
  }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim21)
  {
    switch (HAL_TIM_GetActiveChannel(&htim21))
    {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        ir_tim_interrupt();
        break;
      default:
        break;
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  	  	  	  //  Konfiguracja GPIO

    GPIO_InitTypeDef gpioA9;				// obiekt gpio będący konfiguracją portów GPIO
    gpioA9.Pin = GPIO_PIN_9; 				// konfigurujemy pin 9
    gpioA9.Mode = GPIO_MODE_OUTPUT_PP;	// jako wyjście
    gpioA9.Pull = GPIO_NOPULL;			// rezystory podciągające są wyłączone
    gpioA9.Speed = GPIO_SPEED_FREQ_LOW;	// wystarczą nieskie częstotliwości przełączania
    HAL_GPIO_Init(GPIOA, &gpioA9);		// inicjalizacja modułu GPIOA

  	GPIO_InitTypeDef gpioA8;				// obiekt gpio będący konfiguracją portów GPIO
  	gpioA8.Pin = GPIO_PIN_8; 				// konfigurujemy pin 8
  	gpioA8.Mode = GPIO_MODE_OUTPUT_PP;	// jako wyjście
  	gpioA8.Pull = GPIO_NOPULL;			// rezystory podciągające są wyłączone
  	gpioA8.Speed = GPIO_SPEED_FREQ_LOW;	// wystarczą nieskie częstotliwości przełączania
  	HAL_GPIO_Init(GPIOA, &gpioA8);		// inicjalizacja modułu GPIOA

  	GPIO_InitTypeDef gpioA10;				// obiekt gpio będący konfiguracją portów GPIO
  	gpioA10.Pin = GPIO_PIN_10; 				// konfigurujemy pin 10
  	gpioA10.Mode = GPIO_MODE_OUTPUT_PP;	// jako wyjście
  	gpioA10.Pull = GPIO_NOPULL;			// rezystory podciągające są wyłączone
  	gpioA10.Speed = GPIO_SPEED_FREQ_LOW;	// wystarczą nieskie częstotliwości przełączania
   	HAL_GPIO_Init(GPIOA, &gpioA10);		// inicjalizacja modułu GPIOA

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADCvalue, 2);

  HAL_TIM_Base_Start(&htim21);
  HAL_TIM_IC_Start_IT(&htim21, TIM_CHANNEL_1);




  while (1)
  {

	  int ir_ReadValue = ir_read();
	  if (ir_ReadValue != -1) {
	    printf("code = %02x\n", ir_ReadValue);
	  }
	  if (ir_ReadValue == IR_CODE_ONOFF) 	// Zmiana trybu jazdy - Line Follower /  Jazda na Pilota
	  {
		  if(state == ON)
		  {
			  state = OFF;
	  		  left_motor_drive(100,"FAST STOP");
	  		  right_motor_drive(100,"FAST STOP");
		  }
		  else
		  {
			  state = ON;
		  }
	  }
	  else if(ir_ReadValue == IR_CODE_REWIND) // - Jazda w lewo -
	  {
			left_motor_drive(75,"BACKWARD");
			right_motor_drive(75,"FORWARD");
				 // Swiecenie na Zielono
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	  }
	  else if(ir_ReadValue == IR_CODE_FORWARD) // - Jazda w prawo-
	  {
			left_motor_drive(75,"FORWARD");
			right_motor_drive(75,"BACKWARD");
				 // Swiecenie na Czerwono
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  }
	  else if(ir_ReadValue == IR_CODE_PLUS) // - Jazda porsto-
	  {
			left_motor_drive(75,"FORWARD");
			right_motor_drive(75,"FORWARD");
				 // Swiecenie na Zolto
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	  }
	  else if(ir_ReadValue == IR_CODE_MINUS)  // - Jazda w tyl-
	  {
			left_motor_drive(75,"BACKWARD");
			right_motor_drive(75,"BACKWARD");
				 // Brak Swiecenia (swieci tylko na niebiesko)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  }
	  else if(ir_ReadValue == IR_CODE_PLAY)  // - Zatrzymanie (srodkowy przycisk) -
	  {
		  	left_motor_drive(100,"FAST STOP");
  		  	right_motor_drive(100,"FAST STOP");
  		  		 // Brak Swiecenia (swieci tylko na niebiesko)
  		  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  		  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  }



		  	  ADCdiff = ADCvalue[0] - ADCvalue[1];					// ||||||||||||||||||||||||||||||||||||||||||||



		  	if(state == OFF)
		  	{
		  		 // Swiecenie na niebiesko (gdy tryb jazdy na pilota)
		  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		  	}

	  	  	 // /*
		  	if(state == ON)				// Line Follower z PID
		  	{
		  		followLine();
		  		 // brak swiecenia na niebiesko (gdy tryb Line Followera)
		  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		  	}
		  	//*/





		  	  /*
										//  Line Follower bez PID
		  	  if(state == ON){
				  if(ADCdiff > 75)
				  {
					printf("ADCdiff = %i (lewa strona)\n", ADCdiff); // PID -
	//		  			left_motor_drive(100 + pid.wyjscie,"BACKWARD");
	//		  			right_motor_drive(60 - pid.wyjscie,"FORWARD");
	//		  			left_motor_drive(70,"BACKWARD");
						left_motor_drive(0,"FAST STOP");
						right_motor_drive(75,"FORWARD");

				  }else if(ADCdiff < -75){
					printf("ADCdiff = %i (prawa strona)\n", ADCdiff); // PID +
	//		  			left_motor_drive(60 + pid.wyjscie,"FORWARD");
	//		  			right_motor_drive(100 - pid.wyjscie,"BACKWARD");
						left_motor_drive(75,"FORWARD");
						right_motor_drive(0,"FAST STOP");
	//		  			right_motor_drive(70,"BACKWARD");
				  }else{
						printf("ADCdiff = %i (srodek)\n", ADCdiff);
						left_motor_drive(78,"FORWARD");
						right_motor_drive(78,"FORWARD");
				  }
		  	  }




		  	  */



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

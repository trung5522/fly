/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu.h"
#include "stdbool.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bool set_gyro_angle = false;
volatile long ch[8];
volatile long tick;
volatile uint8_t pulse;
float dt =0.007;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
MPU9250_t mpu;


int receiver_input_channel_1;
int receiver_input_channel_2;
int receiver_input_channel_3;
int receiver_input_channel_4;
int receiver_input_channel_5;
int receiver_input_channel_6;

int throttle;

long gyro_x_cal;
long gyro_y_cal;
long gyro_z_cal;

int loop_timer;

int gyro_x;
int gyro_y;
int gyro_z;


long acc_x;
long acc_y;
long acc_z;
long acc_total_vector;

float angle_roll;
float angle_pitch;
float angle_yaw;

float angle_roll_output;
float angle_pitch_output;
float angle_yaw_output;

float angle_roll_acc;
float angle_pitch_acc;
float angle_yaw_acc;

float roll_level_adjust;
float pitch_level_adjust;

float gyro_roll_input;
float gyro_pitch_input;
float gyro_yaw_input;

float pid_roll_setpoint;
float pid_pitch_setpoint;
float pid_yaw_setpoint;


float pid_p_gain_roll		= 1.68;			//1.5
float pid_i_gain_roll		= 0.055;			//0.02
float pid_d_gain_roll		= 15;			//15

float pid_p_gain_pitch		= 1.68;			//1.5
float pid_i_gain_pitch		= 0.055;			//0.02
float pid_d_gain_pitch		= 15;			//15

float pid_p_gain_yaw		= 0.03;			//0.02
float pid_i_gain_yaw		= 0.0002;
float pid_d_gain_yaw		= 10;			//15

float pid_i_mem_roll;
float pid_i_mem_pitch;
float pid_i_mem_yaw;

float pid_last_roll_d_error;
float pid_last_pitch_d_eroor;
float pid_last_yaw_d_error;

float pid_error_temp;

float pid_roll_output;
float pid_pitch_output;
float pid_yaw_output;

int pid_max_roll			= 400;
int pid_max_pitch			= 400;
int pid_max_yaw				= 400;

int esc_1;
int esc_2;
int esc_3;
int esc_4;

int start;

int min_throthle = 1070;
int max_throthle = 2000;
int disable_motor = 1000;


float turning_speed = 5.0;
bool auto_level = true;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int mapValue(int value, int inMin, int inMax, int outMin, int outMax);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_11){
		tick = __HAL_TIM_GET_COUNTER(&htim4);
		__HAL_TIM_SET_COUNTER(&htim4,0);
		if(tick < 2005){
			if(pulse==2){
				if(tick<1000) ch[pulse]=ch[pulse];
				else
					ch[pulse]= tick;
			}
			else{
				if(tick<1200) ch[pulse]=ch[pulse];
				else
					ch[pulse]= mapValue(tick, 1200, 1700, 1000, 2000);
				//ch[pulse]= tick;
			}
			pulse++;
		}
		else{
			__HAL_TIM_SET_COUNTER(&htim4,0);
			pulse = 0;
		}

	}

}

int mapValue(int value, int inMin, int inMax, int outMin, int outMax) {
if (value < inMin) return outMin;
else if (value > inMax) return outMax;

return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_Delay(10);
     __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
     __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
     __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
     __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
     HAL_Delay(10);

     MPU9250SetDefault(&mpu);
          while(!(setupMPU(&mpu, MPU9250_ADDRESS)==1)) {
         //	 int i;
         //	 for (i=0;i<50;i++){HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);}
          }
          HAL_Delay(100);

          if(updateMPU(&mpu)==1){
         	 //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
          }
          HAL_Delay(2000);
          loop_timer = __HAL_TIM_GET_COUNTER(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 	  receiver_input_channel_1 = ch[2]; //thr
	 	  receiver_input_channel_2 = ch[0]; //roll
	 	  receiver_input_channel_3 = ch[1];	//pitch
	 	  receiver_input_channel_4 = ch[3]; //yaw
	 	  receiver_input_channel_5 = ch[4];	 //sw left
	 	  receiver_input_channel_6 = ch[5]; //sw right
	 	 if(updateMPU(&mpu)==1){
	 			  gyro_x = mpu.g[0]-2.53;
	 			  gyro_y = mpu.g[1]-(-1.95);
	 			  gyro_z = mpu.g[2]-(-0.06);
	 			 // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	 	 }
	 	gyro_pitch_input 	= ( gyro_pitch_input * 0.7 ) + (float)( gyro_y  * 0.3);
	 	gyro_roll_input 	= ( gyro_roll_input * 0.7 ) + (float)( gyro_x  * 0.3);
	 	gyro_yaw_input 	= ( gyro_yaw_input * 0.7 ) + (float)( gyro_z  * 0.3);

	 	angle_pitch_acc = mpu.rpy[1]-3.0;		// -1
	 	angle_roll_acc =mpu.rpy[0]+0.5 ;		// -2.5
	 	angle_yaw_acc = mpu.rpy[2];
	 	//angle_yaw = angle_yaw_acc;

	 	if ( set_gyro_angle ) {
	 		angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
	 		angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
	 		angle_yaw = angle_yaw*0.996 + angle_roll_acc * 0.004;

	 	}
	 	else{
	 		angle_pitch = angle_pitch_acc;
	 		angle_roll = angle_roll_acc;
	 		angle_yaw = angle_yaw_acc;
	 		set_gyro_angle = true;
	 	}
	 		  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
	 		  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
	 		  angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;

	 		  pitch_level_adjust = angle_pitch_output * 15;
	 		  roll_level_adjust = angle_roll_output * 15;

	 		  if ( !auto_level ){
	 			  pitch_level_adjust =0;
	 			  roll_level_adjust =0;
	 		  }


	 		  if ( receiver_input_channel_1 < 1050 && receiver_input_channel_4 < 1050 ) start =1;

	 		  if ( start == 1 && receiver_input_channel_1 < 1050 && receiver_input_channel_4 > 1450 ){
	 			  start = 2;
	 			  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);

	 			  pid_i_mem_roll = 0;
	 			  pid_last_roll_d_error = 0;
	 			  pid_i_mem_pitch = 0;
	 			  pid_last_pitch_d_eroor = 0;
	 			  pid_i_mem_yaw = 0;
	 			  pid_last_yaw_d_error = 0;
	 		  }

	 		  if ( start == 2 && receiver_input_channel_1 < 1050 && receiver_input_channel_4 > 1950 ){
	 			  start =0;
	 			  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	 		  }

	 		  if ( receiver_input_channel_5 > 1500 ) turning_speed = 5;
	 		  else turning_speed = 3;


	 		  pid_roll_setpoint =0;
	 		  if ( receiver_input_channel_2 > 1508 ) pid_roll_setpoint = (receiver_input_channel_2 - 1508);
	 		  else if ( receiver_input_channel_2  < 1492 ) pid_roll_setpoint = ( receiver_input_channel_2  - 1492 );

	 		  pid_roll_setpoint -= roll_level_adjust;
	 		  pid_roll_setpoint /= turning_speed;

	 		  pid_pitch_setpoint =0;
	 		  if ( receiver_input_channel_3 > 1508 ) pid_pitch_setpoint = ( receiver_input_channel_3 - 1508 );
	 		  else if ( receiver_input_channel_3 < 1492 ) pid_pitch_setpoint = ( receiver_input_channel_3 - 1492 );

	 		  pid_pitch_setpoint -= pitch_level_adjust;
	 		  pid_pitch_setpoint /= turning_speed;


	 		  pid_yaw_setpoint =0;
	 		  if ( receiver_input_channel_1 > 1050 ){
	 			  if ( receiver_input_channel_4 > 1508 ) pid_yaw_setpoint = ( receiver_input_channel_4 - 1508 ) / turning_speed;
	 			  else if ( receiver_input_channel_4 < 1492 ) pid_yaw_setpoint = ( receiver_input_channel_4 - 1492 ) / turning_speed;
	 		  }


	 		  //roll calculation
	 		  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	 		  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

	 		  if ( pid_i_mem_roll > pid_max_roll ) pid_i_mem_roll = pid_max_roll;
	 		  else if ( pid_i_mem_roll < pid_max_roll * -1 ) pid_i_mem_roll = pid_max_roll * -1;

	 		  pid_roll_output = ( pid_p_gain_roll * pid_error_temp ) + pid_i_mem_roll + ( pid_d_gain_roll * ( pid_error_temp - pid_last_roll_d_error));

	 		  if ( pid_roll_output > pid_max_roll ) pid_roll_output = pid_max_roll;
	 		  else if ( pid_roll_output < pid_max_roll * -1) pid_roll_output = pid_max_roll * -1;

	 		  pid_last_roll_d_error = pid_error_temp;


	 		  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	 		  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

	 		  if ( pid_i_mem_pitch > pid_max_pitch ) pid_i_mem_pitch = pid_max_pitch;
	 		  else if ( pid_i_mem_pitch < pid_max_pitch * -1 ) pid_i_mem_pitch = pid_max_pitch * -1;

	 		  pid_pitch_output = ( pid_p_gain_pitch * pid_error_temp ) + pid_i_mem_pitch + ( pid_d_gain_pitch * ( pid_error_temp - pid_last_pitch_d_eroor));

	 		  if ( pid_pitch_output > pid_max_pitch ) pid_pitch_output = pid_max_pitch;
	 		  else if ( pid_pitch_output < pid_max_pitch * -1 ) pid_pitch_output = pid_max_pitch * -1;

	 		  pid_last_pitch_d_eroor = pid_error_temp;


	 		  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	 		  pid_i_mem_yaw += pid_p_gain_yaw * pid_error_temp;

	 		  if ( pid_i_mem_yaw > pid_max_yaw ) pid_i_mem_yaw = pid_max_yaw;
	 		  else if ( pid_i_mem_yaw < pid_max_yaw * -1 ) pid_i_mem_yaw = pid_max_yaw * -1;

	 		  pid_yaw_output = ( pid_p_gain_yaw * pid_error_temp ) + pid_i_mem_yaw + ( pid_d_gain_yaw * ( pid_error_temp - pid_last_yaw_d_error ));

	 		  if ( pid_yaw_output > pid_max_yaw ) pid_yaw_output = pid_max_yaw;
	 		  else if ( pid_yaw_output < pid_max_yaw * -1 ) pid_yaw_output = pid_max_yaw * -1;

	 		  pid_last_yaw_d_error = pid_error_temp;

	 		  throttle = receiver_input_channel_1;


	 		  if ( start == 2 ){
	 			  if ( throttle > 1900 ) throttle = 1900;

	 			  esc_1 = throttle - pid_pitch_output + pid_roll_output - pid_yaw_output;        //Calculate the pulse for esc 1 (front-right - CCW).
	 			  esc_2 = throttle + pid_pitch_output + pid_roll_output + pid_yaw_output;        //Calculate the pulse for esc 2 (rear-right - CW).
	 			  esc_3 = throttle + pid_pitch_output - pid_roll_output - pid_yaw_output;        //Calculate the pulse for esc 3 (rear-left - CCW).
	 			  esc_4 = throttle - pid_pitch_output - pid_roll_output + pid_yaw_output;        //Calculate the pulse for esc 4 (front-left - CW).

	 			  if ( esc_1 < min_throthle ) esc_1 = min_throthle;
	 			  if ( esc_2 < min_throthle ) esc_2 = min_throthle;
	 			  if ( esc_3 < min_throthle ) esc_3 = min_throthle;
	 			  if ( esc_4 < min_throthle ) esc_4 = min_throthle;

	 			  if ( esc_1 > max_throthle ) esc_1 = max_throthle;
	 			  if ( esc_2 > max_throthle ) esc_2 = max_throthle;
	 			  if ( esc_3 > max_throthle ) esc_3 = max_throthle;
	 			  if ( esc_4 > max_throthle ) esc_4 = max_throthle;


	 		  }else{
	 			  esc_1 = disable_motor;
	 			  esc_2 = disable_motor;
	 			  esc_3 = disable_motor;
	 			  esc_4 = disable_motor;
	 		  }

	 		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,esc_1);
	 		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,esc_2);
	 		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,esc_3);
	 		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,esc_4);

	 		  if ( __HAL_TIM_GET_COUNTER(&htim3) - loop_timer > 4070 ){
	 	//		  HAL_GPIO_TogglePin(led_status_GPIO_Port, led_status_Pin);
	 		  }

	 	//	  cuoi = HAL_GetTick() - dau;
	 		  while ( __HAL_TIM_GET_COUNTER(&htim2) - loop_timer < 4000 );
	 		  loop_timer = __HAL_TIM_GET_COUNTER(&htim2);


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
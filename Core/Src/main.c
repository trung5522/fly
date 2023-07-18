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
float dt =0.006;
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

int cal_int;
long gyro_x_cal;
long gyro_y_cal;
long gyro_z_cal;

uint16_t loop_timer;

int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;


long acc_x;
long acc_y;
long acc_z;
long acc_total_vector;

float angle_roll;
float angle_pitch;
float angle_yaw;
float angle_pitch_1;
float angle_roll_1;

float angle_roll_output;
float angle_pitch_output;
float angle_yaw_output;

float angle_roll_acc;
float angle_pitch_acc;
float angle_yaw_acc;
float angle_pitch_acc_1;
float angle_roll_acc_1;

float roll_level_adjust;
float pitch_level_adjust;

float gyro_roll_input;
float gyro_pitch_input;
float gyro_yaw_input;

float pid_roll_setpoint;
float pid_pitch_setpoint;
float pid_yaw_setpoint;


float pid_p_gain_roll		= 0.5;			//1.5
float pid_i_gain_roll		= 3.5;			//0.02
float pid_d_gain_roll		= 0.01;			//15

float pid_p_gain_pitch		= 0.5;			//1.5
float pid_i_gain_pitch		= 3.5;			//0.02
float pid_d_gain_pitch		= 0.01;			//15

float pid_p_gain_yaw		= 2;			//0.02
float pid_i_gain_yaw		= 12;
float pid_d_gain_yaw		= 0.0;			//15

float pid_i_mem_roll;
float pid_i_mem_pitch;
float pid_i_mem_yaw;

float pid_i_mem_roll_last;
float pid_i_mem_pitch_last;
float pid_i_mem_yaw_last;

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

//float KalmanOut[2];
/* Kalman filter variables */
float Q_anglez= 0.008f;; // Process noise variance for the accelerometer
float Q_biasz= 0.003f; // Process noise variance for the gyro bias
float R_measurez= 0.05f; // Measurement noise variance - this is actually the variance of the measurement noise

float Q_angle= 0.008f; // Process noise variance for the accelerometer
float Q_bias= 0.003f; // Process noise variance for the gyro bias
float R_measure= 0.05f;// Measurement noise variance - this is actually the variance of the measurement noise



float angle = 0.0f;; // The angle calculated by the Kalman filter - part of the 2x1 state vector
float bias =0.0f;; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
float S;
float K[2];
float y;
float P00_temp;
float P01_temp;


float anglez = 0.0f; // The angle calculated by the Kalman filter - part of the 2x1 state vector
float biasz =0.0f;// The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float ratez; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

float Pz[2][2]; // Error covariance matrix - This is a 2x2 matrix
float Sz;
float Kz[2];
float yz;
float P00_tempz;
float P01_tempz;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int mapValue(int value, int inMin, int inMax, int outMin, int outMax);
void gyro_signalen(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
float Kalman_getAngle_roll(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
   rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += (Q_bias) * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
     S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
     // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
     y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    P00_temp = P[0][0];
    P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};
//
float Kalman_getAngle_pitch(float newAnglez, float newRatez, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    ratez = newRatez - biasz;
    anglez += dt * ratez;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    Pz[0][0] += dt * (dt*Pz[1][1] - Pz[0][1] - Pz[1][0] + Q_anglez);
    Pz[0][1] -= dt * Pz[1][1];
    Pz[1][0] -= dt * Pz[1][1];
    Pz[1][1] += (Q_biasz ) * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
     Sz = Pz[0][0] + R_measurez; // Estimate error
    /* Step 5 */
     // Kalman gain - This is a 2x1 vector
    Kz[0] = Pz[0][0] / Sz;
    Kz[1] = Pz[1][0] / Sz;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
     yz = newAnglez - anglez; // Angle difference
    /* Step 6 */
    anglez += Kz[0] * yz;
    biasz += Kz[1] * yz;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    P00_tempz = Pz[0][0];
    P01_tempz = Pz[0][1];

    Pz[0][0] -= Kz[0] * P00_tempz;
    Pz[0][1] -= Kz[0] * P01_tempz;
    Pz[1][0] -= Kz[1] * P00_tempz;
    Pz[1][1] -= Kz[1] * P01_tempz;

    return anglez;
};
//void kalman(float KalmanState,float KalmanUncer,float KalmanInput, float KalmanMeasure){
//	KalmanState = KalmanState +0.007*KalmanInput;
//	KalmanUncer = KalmanUncer +0.007*0.007*4*4;
//	float KalmanGain = KalmanUncer*1/(1*KalmanUncer+3*3);
//	KalmanState = KalmanState + KalmanGain*(KalmanMeasure-KalmanState);
//	KalmanUncer = (1-KalmanGain)*KalmanUncer;
//	KalmanOut[0]=KalmanState;
//	KalmanOut[1]=KalmanUncer;
//}


void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_11){
		tick = __HAL_TIM_GET_COUNTER(&htim4);
		__HAL_TIM_SET_COUNTER(&htim4,0);
		if(tick < 2008){
			if(pulse==2){
//				if(tick<1000) ch[pulse]=ch[pulse];
//				else
					ch[pulse]= tick;
			}
			else if((pulse ==4) || (pulse ==5)){
				ch[pulse]= tick;
			}
			else{
//				if(tick<1200 || tick >1700 ) ch[pulse]=ch[pulse];
//				else
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
void calibrate_gyro(void) {
  cal_int = 0;                                                                        //Set the cal_int variable to zero.
  if (cal_int != 2000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
      if (cal_int % 25 == 0) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                    //Change the led status every 125 readings to indicate calibration.
      gyro_signalen();                                                                //Read the gyro output.
      gyro_x_cal += gyro_x;                                                     //Ad roll value to gyro_roll_cal.
      gyro_y_cal += gyro_y;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_z_cal += gyro_z;                                                       //Ad yaw value to gyro_yaw_cal.
      HAL_Delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
    }
    //red_led(HIGH);                                                                     //Set output PB3 low.
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_x_cal /= 2000;                                                            //Divide the roll total by 2000.
    gyro_y_cal /= 2000;                                                           //Divide the pitch total by 2000.
    gyro_z_cal /= 2000;                                                             //Divide the yaw total by 2000.
  }
}
void gyro_signalen(void) {
//  HWire.beginTransmission(gyro_address);                       //Start communication with the gyro.
//  HWire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
//  HWire.endTransmission();                                     //End the transmission.
//  HWire.requestFrom(gyro_address, 14);                         //Request 14 bytes from the MPU 6050.
//  acc_y = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_x variable.
//  acc_x = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_y variable.
//  acc_z = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_z variable.
//  temperature = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the temperature variable.
//  gyro_roll = HWire.read() << 8 | HWire.read();                //Read high and low part of the angular data.
//  gyro_pitch = HWire.read() << 8 | HWire.read();               //Read high and low part of the angular data.
//  gyro_yaw = HWire.read() << 8 | HWire.read();                 //Read high and low part of the angular data.
	//update_accel_gyro(&mpu);
	if(updateMPU(&mpu)==1){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
   	 //HAL_Delay(100);
//    gyro_x = mpu.g[0];
//	gyro_y = mpu.g[1];                                            //Invert the direction of the axis.
//    gyro_z = mpu.g[2];                                              //Invert the direction of the axis.

//  if (level_calibration_on == 0) {
//    acc_y -= acc_pitch_cal_value;                              //Subtact the manual accelerometer pitch calibration value.
//    acc_x -= acc_roll_cal_value;                               //Subtact the manual accelerometer roll calibration value.
//  }

    gyro_x = mpu.g[0] - 2.57;                                  //Subtact the manual gyro roll calibration value.
    gyro_y = mpu.g[1] + 2.1;                                //Subtact the manual gyro pitch calibration value.
    gyro_z = mpu.g[2] - 0.035;                                    //Subtact the manual gyro yaw calibration value.

	}
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
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

  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_GPIO_Init();
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

          int i;
          for(i=0;i<100;i++){
               if(updateMPU(&mpu)==1){
              	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,1);
              	 HAL_Delay(100);
               }
           }

          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
          HAL_Delay(2000);
          calibrate_gyro();
          loop_timer = __HAL_TIM_GET_COUNTER(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//uint32_t dau = HAL_GetTick();
 	  	  receiver_input_channel_1 = ch[2]; //thr
	 	  receiver_input_channel_2 = ch[0]; //roll
	 	  receiver_input_channel_3 = ch[1];	//pitch
	 	  receiver_input_channel_4 = ch[3]; //yaw
	 	  receiver_input_channel_5 = ch[4];	 //sw left
	 	  receiver_input_channel_6 = ch[5]; //sw right
//	 	 if(updateMPU(&mpu)==1){
//	 			  gyro_x = mpu.g[0]-2.53;
//	 			  gyro_y = mpu.g[1]-(-1.95);
//	 			  gyro_z = mpu.g[2]-(-0.06);
//	 			 // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//	 	 }
	 	gyro_signalen();
//	 		 			  gyro_x = mpu.g[0]-2.53;
//	 		 			  gyro_y = mpu.g[1]-(-1.95);
//	 		 			  gyro_z = mpu.g[2]-(-0.15);
	 	gyro_pitch_input 	= ( gyro_pitch_input * 0.7 ) + (float)( gyro_y  * 0.3);
	 	gyro_roll_input 	= ( gyro_roll_input * 0.7 ) + (float)( gyro_x  * 0.3);
	 	gyro_yaw_input 	= ( gyro_yaw_input * 0.7 ) + (float)( -gyro_z  * 0.3);

	 	//Gyro angle calculations
	 	  //0.0000611 = 1 / (250Hz / 65.5)
//	 	  angle_pitch += (float)gyro_y * 0.004;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
//	 	  angle_roll += (float)gyro_x * 0.004;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
//	 	  angle_yaw += (float)gyro_z * 0.004;                                        //Calculate the traveled yaw angle and add this to the angle_yaw variable.
//	 	  if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
//	 	  else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

	 	 //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
//	 	   angle_pitch -= angle_roll * sin((float)gyro_z * 0.000069822);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
//	 	   angle_roll += angle_pitch * sin((float)gyro_z * 0.000069822);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

//	 	   angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;       //Calculate the difference between the gyro and compass heading and make a small correction.
//	 	   if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
//	 	   else if (angle_yaw >= 360) angle_yaw -= 360;   									//If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.


	 	  //Accelerometer angle calculations
//	 	  acc_total_vector = sqrt((mpu.a[0] *mpu.a[0] ) + (mpu.a[1] * mpu.a[1]) + (mpu.a[2] * mpu.a[2]));    //Calculate the total accelerometer vector.
//
//	 	  if (abs(mpu.a[1]) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
//	 	    angle_pitch_acc_1 = asin((float)mpu.a[1] / acc_total_vector) * 57.296;              //Calculate the pitch angle.
//	 	  }
//	 	  if (abs(mpu.a[0]) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
//	 	    angle_roll_acc_1 = asin((float)mpu.a[0] / acc_total_vector) * 57.296;               //Calculate the roll angle.
//	 	  }

	 	//angle_pitch_acc = mpu.rpy[1]+5.63;		// -1
	 	//angle_roll_acc =mpu.rpy[0]-0.76;		// -2.5
	 	//angle_yaw_acc = mpu.rpy[2];
	 	//angle_yaw = angle_yaw_acc;

//	 	 angle_pitch_acc_1 -= 0.57;		// -0.08
//	 	 angle_roll_acc_1 -= -0.427;		// -1
//
	 	angle_pitch_acc = Kalman_getAngle_pitch((-(mpu.rpy[1] + 1.57)), mpu.g[1], 0.006);//(-(mpu.rpy[1] + 1.42));
	 	angle_roll_acc = Kalman_getAngle_roll((mpu.rpy[0] + 0.38), mpu.g[0], 0.006);//(mpu.rpy[0] + 0.38);

	 	if ( set_gyro_angle ) {
	 		angle_pitch = angle_pitch * 0.8 + angle_pitch_acc * 0.2;
	 		angle_roll = angle_roll * 0.8 + angle_roll_acc * 0.2;
	 		//angle_yaw = angle_yaw*0.996 + angle_roll_acc * 0.004;

	 	}
	 	else{
	 		angle_pitch = (-(mpu.rpy[1] + 0.15));
	 		angle_roll = (mpu.rpy[0] + 0.3);
	 		//angle_yaw = 0;
	 		set_gyro_angle = true;
	 	}
	 		  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
	 		  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
	 		  //angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;

	 		  pitch_level_adjust = angle_pitch_output * 15;
	 		  roll_level_adjust =  angle_roll_output * 15;

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
	 			  pid_i_mem_roll_last = 0;

	 			  pid_i_mem_pitch = 0;
	 			  pid_last_pitch_d_eroor = 0;
	 			  pid_i_mem_pitch_last = 0;

	 			  pid_i_mem_yaw = 0;
	 			  pid_last_yaw_d_error = 0;
	 			  pid_i_mem_yaw_last = 0;
	 		  }

	 		  if ( start == 2 && receiver_input_channel_1 < 1050 && receiver_input_channel_4 > 1950 ){
	 			  start =0;
	 			  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	 		  }

	 		  if ( receiver_input_channel_5 > 1500 ) turning_speed = 7;
	 		  else turning_speed = 5;


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
	 		  pid_error_temp = -gyro_roll_input + pid_roll_setpoint;
	 		  pid_i_mem_roll =  pid_i_mem_roll_last + 0.5 * pid_i_gain_roll * dt * (pid_error_temp +pid_last_roll_d_error) ;

	 		  if ( pid_i_mem_roll > pid_max_roll ) pid_i_mem_roll = pid_max_roll;
	 		  else if ( pid_i_mem_roll < pid_max_roll * -1 ) pid_i_mem_roll = pid_max_roll * -1;

	 		  pid_roll_output = ( pid_p_gain_roll * (pid_error_temp)) + pid_i_mem_roll + ( pid_d_gain_roll * (( pid_error_temp - pid_last_roll_d_error) / dt));

	 		  if ( pid_roll_output > pid_max_roll ) pid_roll_output = pid_max_roll;
	 		  else if ( pid_roll_output < pid_max_roll * -1) pid_roll_output = pid_max_roll * -1;

	 		  pid_last_roll_d_error = pid_error_temp;
	 		  pid_i_mem_roll_last = pid_i_mem_roll;

	 		  // pitch calculation
	 		  pid_error_temp = -gyro_pitch_input + pid_pitch_setpoint;
	 		  pid_i_mem_pitch = pid_i_mem_pitch_last + 0.5*pid_i_gain_pitch*dt*(pid_error_temp+pid_last_pitch_d_eroor);

	 		  if ( pid_i_mem_pitch > pid_max_pitch ) pid_i_mem_pitch = pid_max_pitch;
	 		  else if ( pid_i_mem_pitch < pid_max_pitch * -1 ) pid_i_mem_pitch = pid_max_pitch * -1;

	 		  pid_pitch_output = ( pid_p_gain_pitch * (pid_error_temp) ) + pid_i_mem_pitch + ( pid_d_gain_pitch * (( pid_error_temp - pid_last_pitch_d_eroor)/dt));

	 		  if ( pid_pitch_output > pid_max_pitch ) pid_pitch_output = pid_max_pitch;
	 		  else if ( pid_pitch_output < pid_max_pitch * -1 ) pid_pitch_output = pid_max_pitch * -1;

	 		  pid_last_pitch_d_eroor = pid_error_temp;
	 		  pid_i_mem_pitch_last = pid_i_mem_pitch;

	 		  // yaw calculation
	 		  pid_error_temp = -gyro_yaw_input + pid_yaw_setpoint;
	 		  pid_i_mem_yaw  = pid_i_mem_yaw_last + 0.5 * pid_i_gain_yaw * dt * (pid_error_temp+pid_last_yaw_d_error);

	 		  if ( pid_i_mem_yaw > pid_max_yaw ) pid_i_mem_yaw = pid_max_yaw;
	 		  else if ( pid_i_mem_yaw < pid_max_yaw * -1 ) pid_i_mem_yaw = pid_max_yaw * -1;

	 		  pid_yaw_output = ( pid_p_gain_yaw * (pid_error_temp) ) + pid_i_mem_yaw + ( pid_d_gain_yaw * (( pid_error_temp - pid_last_yaw_d_error )/dt));

	 		  if ( pid_yaw_output > pid_max_yaw ) pid_yaw_output = pid_max_yaw;
	 		  else if ( pid_yaw_output < pid_max_yaw * -1 ) pid_yaw_output = pid_max_yaw * -1;

	 		  pid_last_yaw_d_error = pid_error_temp;
	 		  pid_i_mem_yaw_last = pid_i_mem_pitch;



	 		  throttle = receiver_input_channel_1;


	 		  if ( start == 2 ){
	 			  if ( throttle > 1800 ) throttle = 1800;

	 			  esc_1 = throttle - pid_pitch_output - pid_roll_output - pid_yaw_output;        //Calculate the pulse for esc 1 (front-right - CCW).
	 			  esc_2 = throttle + pid_pitch_output - pid_roll_output + pid_yaw_output;        //Calculate the pulse for esc 2 (rear-right - CW).
	 			  esc_3 = throttle + pid_pitch_output + pid_roll_output - pid_yaw_output;        //Calculate the pulse for esc 3 (rear-left - CCW).
	 			  esc_4 = throttle - pid_pitch_output + pid_roll_output + pid_yaw_output;        //Calculate the pulse for esc 4 (front-left - CW).

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

	 		  if (abs(__HAL_TIM_GET_COUNTER(&htim2) - loop_timer) > 7000 ){
	 			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	 		  }

	 	//	  cuoi = HAL_GetTick() - dau;
	 		// uint32_t cuoi = HAL_GetTick() - dau;
	 		 //uint32_t cuoi2 = abs(__HAL_TIM_GET_COUNTER(&htim2) - loop_timer);
	 		 while ( abs(__HAL_TIM_GET_COUNTER(&htim2) - loop_timer) < 6000 );
	 		 //cuoi2 = abs(__HAL_TIM_GET_COUNTER(&htim2) - loop_timer);
	 		 __HAL_TIM_SET_COUNTER(&htim2,0);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim2.Init.Prescaler = 71;
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
  htim3.Init.Prescaler = 71;
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
  htim4.Init.Prescaler = 71;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

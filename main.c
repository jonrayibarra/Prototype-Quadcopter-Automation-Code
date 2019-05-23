/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TJ_MPU6050.h"
#include "dwt_stm32_delay.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FMPI2C_HandleTypeDef hfmpi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;
/* USER CODE END 0 */
	float sampling_period;
	float PI = 3.1415926535897;
	float acc_x, acc_y, acc_z;
	float gyro_x, gyro_y, gyro_z;
	double gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0; //changed from float to double
	float angle_pitch_gyro, angle_roll_gyro, angle_yaw_gyro;
	float angle_pitch_acc, angle_roll_acc, angle_yaw_acc, acc_total_vector;
	float angle_pitch_output, angle_roll_output, angle_yaw_output;
	bool set_gyro_angles = false;
	uint16_t dutyCycle = 0;
	//PID Variables
	float pid_error_roll, pid_error_pitch, pid_error_yaw;
	float pid_i_mem_roll, pid_i_mem_pitch, pid_i_mem_yaw;
	float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
	float pid_d_last_roll_error, pid_d_last_pitch_error, pid_d_last_yaw_error;
	float pid_out_roll, pid_out_pitch, pid_out_yaw;
	//Tune
	float pid_p_gain_roll = 1, pid_i_gain_roll = 1, pid_d_gain_roll = 18;
	float pid_p_gain_pitch = 1, pid_i_gain_pitch = 1, pid_d_gain_pitch = 18;
	float pid_p_gain_yaw = 1, pid_i_gain_yaw = 1, pid_d_gain_yaw = 1;
	
	//ADDING NEW VARIABLES FOR PID CALCULATIONS
	int pid_max_roll = 400;
	int pid_max_pitch = 400;
	int pid_max_yaw = 400;
	
	double gyro_pitch, gyro_roll, gyro_yaw;
	float pid_error_temp;
	float gyro_roll_input;
	float gyro_pitch_input;
	float gyro_yaw_input;
	float angle_pitch;
	float angle_roll;
	
//	int calcounter;

	double gyro_fixed_x;
	double gyro_fixed_y;
	double gyro_fixed_z;
	
	double acc_x_cal = 0, acc_y_cal = 0, acc_z_cal = 0;
	
	double acc_fixed_x;
	double acc_fixed_y;
	double acc_fixed_z;
	
	float pitch_level_adjust;
	float roll_level_adjust;
	
	// Drone needs a delay to power up?
	int delayon = 1;
	
	// Duty cycle variables for changing motor speeds
	float dc1, dc2, dc3, dc4;
	
	////////////////////////////////////////////
	//Motors
	float motor_1_fr, motor_2_rr, motor_3_rl, motor_4_fl;
	//Loop Timer
	uint32_t oldtime;
	int count;
	uint32_t newtime;
	int test, result;
	bool temp;
	long looptimer;
	float t1, t2, t3, t5, t6, t7, t8, t9;
	float ccr1_, ccr2_, ccr3_, ccr4_;
	int t4;
	char rx_buffer[50], tx_buffer[50];
	bool led_state = false;
	
	// bluetooth code
	char rxData[30];

	// boolean for motors
	int motorstate = 0;
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
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
  MX_DMA_Init();
  MX_RTC_Init();
  MX_FMPI2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	// added blut code
		char txData[30] = "Hello World\r\n";
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxData, 10);
	
	//1. Initialize the MPU6050 module and I2C
	MPU6050_Init(&hfmpi2c1);
	//2. Configure Accel and Gyro Parameters
	myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit = 0; //0 is normal mode
	MPU6050_Config(&myMpuConfig);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		//Output is 200Hz accordingly google calculation + Liz (caveat emptor)
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

		count = 0;
		temp = false;
		oldtime = 0;
		DWT_Delay_Init();
		
		//Aggregate loop average not working, numbers derived by observation
		//gyro_x_cal = -140;
		gyro_y_cal = 45;
		gyro_z_cal = 70;
		
		t1 = 0; // used for gyro x cal, then used for angle pitch gyro, then PID setpoints
		t2 = 0; // used for gyro y cal, then used for angle roll gyro, then used for roll calculations (calculate_pid())
		t3 = 0; // used for gyro z cal
		t4 = 0; // used for absolute value comparisons
		t5 = 0; // used for determining esc 1 pulse
		t6 = 0; // used for determining esc 2 pulse
		t7 = 0; // used for determining esc 3 pulse
		t8 = 0; // used for determining esc 4 pulse
		t9 = 0;

//		// calibration code working now should loop 2000 times not 200
//		for (int calcounter = 0; calcounter < 100; calcounter++)
//		{
//			MPU6050_Get_Gyro_RawData(&myGyroRaw);
//			MPU6050_Get_Accel_RawData(&myAccelRaw);
//			t1 = myGyroRaw.x;
//			gyro_x_cal += t1;
//			t2 = myGyroRaw.y;
//			gyro_y_cal += t2;
//			t3 = myGyroRaw.z;
//			gyro_z_cal += t3;
//		}
//		
//		t1 = gyro_x_cal;
//		gyro_x_cal = t1 / 100;
//		t2 = gyro_y_cal;
//		gyro_y_cal = t2 / 100;
//		t3 = gyro_z_cal;
//		gyro_z_cal = t3 / 100;
		
//		// accelerometer calibration
//		for (int calcounter = 0; calcounter < 100; calcounter++)
//		{
//			MPU6050_Get_Gyro_RawData(&myGyroRaw);
//			MPU6050_Get_Accel_RawData(&myAccelRaw);
//			t1 = myAccelRaw.x;
//			acc_x_cal += t1;
//			t2 = myAccelRaw.y;
//			acc_y_cal += t2;
//			t3 = myAccelRaw.z;
//			acc_z_cal += t3;
//		}
//		
//		t1 = acc_x_cal;
//		acc_x_cal = t1 / 100;
//		t2 = acc_y_cal;
//		acc_y_cal = t2 / 100;
//		t3 = acc_z_cal;
//		acc_z_cal = t3 / 100;

					 //Solution
			//MPU6050_Get_Gyro_RawData(&myGyroRaw);
			//t1 = myGyroRaw.x;
			//t2 += t1;
		
		test = 49;
		result = sqrt(test);
		looptimer = HAL_GetTick();
		
  while (1)
  {	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);					
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  //Raw Data
		MPU6050_Get_Gyro_RawData(&myGyroRaw);
		MPU6050_Get_Accel_RawData(&myAccelRaw);
		
		// ONLY RAW VALUES
		gyro_x = myGyroRaw.x;
		gyro_y = myGyroRaw.y;
		gyro_z = myGyroRaw.z;
		
		acc_x = myAccelRaw.x;
		acc_y = myAccelRaw.y;
		acc_z = myAccelRaw.z;
		
		// Fixing angles using calibration code and raw gyro outputs
		gyro_fixed_x = myGyroRaw.x - gyro_x_cal;
		gyro_fixed_y = myGyroRaw.y - gyro_y_cal;
		gyro_fixed_z = myGyroRaw.z - gyro_z_cal;
		
		acc_fixed_x = myAccelRaw.x - acc_x_cal;
		acc_fixed_y = myAccelRaw.y - acc_y_cal;
		acc_fixed_z = myAccelRaw.z - acc_z_cal;
		
		// gyro PID input
		gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_x / 65.5) *0.3);
		gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_y / 65.5) *0.3);
		gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_z / 65.5) *0.3);
		
		// traveled x angle
		// 0.0000611 = 1 / 250Hz / 65.5
		// 0.0152672 = 1/ 1Hz / 65.5
		// 0.00010178 = 1/ 150Hz / 65.5
		t1 = gyro_x / (6.6666 * 65.5);
		angle_pitch_gyro += t1;   		
		
		// traveled y angle
		t2 = gyro_y / (6.6666 * 65.5);
		angle_roll_gyro += t2;
		
		// transferring roll angle to pitch angle after yawed
		// 0.000001066 = 0.0000611 * (3.142(PI) / 180degrees)
		// 0.000266463 = 0.0152672 * (3.142(PI) / 180degrees)
		// 0.000001777 = 0.00010178 * (3.142(PI) / 180degrees)
		t1 = angle_roll_gyro * sin(gyro_z * 0.000001777);
		angle_pitch_gyro -= t1;
		
		t2 = angle_pitch_gyro * sin(gyro_z * 0.000001777);
		angle_roll_gyro += t2;
		
		acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
		
		t4 = acc_y;		// need integer for absolute value
		if (abs(t4) < acc_total_vector)
		{
			angle_pitch_acc = asin(acc_y / acc_total_vector) * 57.296;
		}
		
		t4 = acc_x;		// need integer for absolute value
		if (abs(t4) < acc_total_vector)
		{
			angle_roll_acc = asin(acc_x / acc_total_vector) * -57.296;
		}
		
		pitch_level_adjust = angle_pitch_gyro; // * 15
		roll_level_adjust = angle_roll_gyro; // * 15
		
		if (set_gyro_angles)
		{
		// correcting the drift of gyro pitch with acc pitch
		t1 = angle_pitch_gyro;
		angle_pitch_gyro = t1 * 0.9 + angle_pitch_acc * 0.1;
		
		// correcting the drift of gyro roll with acc roll
		t1 = angle_roll_gyro;
		angle_roll_gyro = t1 * 0.9 + angle_roll_acc * 0.1;
		}
		else
		{
			angle_pitch_gyro = angle_pitch_acc; //Set the gyro pitch angle equal to the accelerometer pitch angle 
			angle_roll_gyro = angle_roll_acc; //Set the gyro roll angle equal to the accelerometer roll angle 
			set_gyro_angles = true;
			
			// reset PID controllers
			pid_i_mem_roll = 0;
			pid_i_mem_pitch = 0;
			pid_i_mem_yaw = 0;
		
			pid_d_last_roll_error = 0;
			pid_d_last_pitch_error = 0;
			pid_d_last_yaw_error = 0;
		}
		
		// starting PID calculations
		pid_roll_setpoint = 0;
		pid_pitch_setpoint = 0;
		
		t1 = pid_roll_setpoint;
		pid_roll_setpoint = t1 - roll_level_adjust;
		t1 = pid_roll_setpoint;
		pid_roll_setpoint = t1 / 3;
		
		t1 = pid_pitch_setpoint;
		pid_pitch_setpoint = t1 - pitch_level_adjust;
		t1 = pid_pitch_setpoint;
		pid_pitch_setpoint = t1 / 3;
		
		// yaw setpoint?
		
		// calculating PID
		// roll calculations
		pid_error_temp = gyro_roll_input - pid_roll_setpoint;
		
		t2 = pid_i_gain_roll * pid_error_temp;
		pid_i_mem_roll += t2;
		
		if (pid_i_mem_roll > pid_max_roll)
		{
			pid_i_mem_roll = pid_max_roll;
		}
		else if (pid_i_mem_roll < pid_max_roll * -1)
		{
			pid_i_mem_roll = pid_max_roll * -1;
		}
		
		pid_out_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_d_last_roll_error);
		
		if (pid_out_roll > pid_max_roll)
		{
			pid_out_roll = pid_max_roll;
		}
		else if (pid_out_roll < pid_max_roll * -1)
		{
			pid_out_roll = pid_max_roll * -1;
		}
		
		pid_d_last_roll_error = pid_error_temp;
		
		// pitch calculations
		pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
		
		t2 = pid_i_gain_pitch * pid_error_temp;
		pid_i_mem_pitch += t2;
		
		if (pid_i_mem_pitch > pid_max_pitch)
		{
			pid_i_mem_pitch = pid_max_pitch;
		}
		else if (pid_i_mem_pitch < pid_max_pitch * -1)
		{
			pid_i_mem_pitch = pid_max_pitch * -1;
		}
		
		pid_out_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_d_last_pitch_error);
		
		if (pid_out_pitch > pid_max_pitch)
		{
			pid_out_pitch = pid_max_pitch;
		}
		else if (pid_out_pitch < pid_max_pitch * -1)
		{
			pid_out_pitch = pid_max_pitch * -1;
		}
		
		pid_d_last_pitch_error = pid_error_temp;
		
		// yaw calculations
		pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
		
		t2 = pid_i_gain_yaw * pid_error_temp;
		pid_i_mem_yaw += t2;
		
		if (pid_i_mem_yaw > pid_max_yaw)
		{
			pid_i_mem_yaw = pid_max_yaw;
		}
		else if (pid_i_mem_yaw < pid_max_yaw * -1)
		{
			pid_i_mem_yaw = pid_max_yaw * -1;
		}
		
		pid_out_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_d_last_yaw_error);
		
		if (pid_out_yaw > pid_max_yaw)
		{
			pid_out_yaw = pid_max_yaw;
		}
		else if (pid_out_yaw < pid_max_yaw * -1)
		{
			pid_out_yaw = pid_max_yaw * -1;
		}
		
		pid_d_last_yaw_error = pid_error_temp;
		
		dutyCycle += 2;
		if (dutyCycle >= 38) dutyCycle = 36;
		
		t5 = dutyCycle + (pid_out_roll/100) - (pid_out_pitch/100) - (pid_out_yaw/100); // checking esc 1
		t6 = dutyCycle + (pid_out_roll/100) + (pid_out_pitch/100) + (pid_out_yaw/100); // checking esc 2
		t7 = dutyCycle - (pid_out_roll/100) + (pid_out_pitch/100)  - (pid_out_yaw/100); // checking esc 3
		t8 = dutyCycle - (pid_out_roll/100) - (pid_out_pitch/100)  + (pid_out_yaw/100); // checking esc 4
		
		if (t5 < 10) t5 = 10;
		if (t6 < 10) t6 = 10;
		if (t7 < 10) t7 = 10;
		if (t8 < 10) t8 = 10;
		
		if (t5 > 40) t5 = 40;
		if (t6 > 40) t6 = 40;
		if (t7 > 40) t7 = 40;
		if (t8 > 40) t8 = 40;
		
		
//*****************************************************************************	

		//Motor Output
		if(motorstate == true)
		{
			htim1.Instance->CCR1 = t5;
			htim1.Instance->CCR2 = t6;
			htim1.Instance->CCR3 = t7;
			htim1.Instance->CCR4 = t8;
		}
		
		else
		{
			htim1.Instance->CCR1 = 0;
			htim1.Instance->CCR2 = 0;
			htim1.Instance->CCR3 = 0;
			htim1.Instance->CCR4 = 0;
		}

//		if(motorstate == true)
//		{
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); //blue
//			if(dutyCycle <= 0) dutyCycle = 4;
//			dutyCycle-= 2;
//		}
//		// curently the motors run at start
//		if(motorstate == 1 && delayon == 1) // motor starting
//		{
//			if(dutyCycle < 30)
//			{
//				dutyCycle += 2;
//			}
//			
//			if(dutyCycle >= 30) 
//			{
//				delayon = 0;
//			}
//		}
//		
//		else if (motorstate == 1 && delayon == 0) // motor already started
//		{
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
//			if(dutyCycle >= 30) dutyCycle=28;
//			dutyCycle += 2;
//		}
//		
//		else if (motorstate == 0 && delayon == 0) // motor off and we want to decrement it slowly 
//		{
//			if(dutyCycle > 2)
//			{
//				dutyCycle -= 2;
//			}
//			
//			if(dutyCycle <= 2) 
//			{
//				delayon = 1;
//			}
//		}
//		
//		else // motor is off and setup for next startup
//		{
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
////			if(dutyCycle <= 2) dutyCycle = 4;
////			dutyCycle -= 2;
//		}
				// removing delay because of loop timer
//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); //blue
			//Use this to figure out loop time of code
			if(HAL_GetTick() - looptimer > 130)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); //blue
			}; 	
		
			//Control Looptime
			while(HAL_GetTick() - looptimer < 150);
			looptimer = HAL_GetTick();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_FMPI2C1;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Fmpi2c1ClockSelection = RCC_FMPI2C1CLKSOURCE_APB;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FMPI2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMPI2C1_Init(void)
{

  /* USER CODE BEGIN FMPI2C1_Init 0 */

  /* USER CODE END FMPI2C1_Init 0 */

  /* USER CODE BEGIN FMPI2C1_Init 1 */

  /* USER CODE END FMPI2C1_Init 1 */
  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0x20303E5D;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMPI2C1_Init 2 */

  /* USER CODE END FMPI2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 96;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  //huart1.Init.BaudRate = 115200;  changed this
	huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA6 PA7 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12 
                           PB13 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
	HAL_UART_Transmit(&huart1, (uint8_t *)rxData, strlen(rxData), 10);
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); 
	if(motorstate == 0)
	{
		motorstate = 1;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	}
	else
	{
		motorstate = 0;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

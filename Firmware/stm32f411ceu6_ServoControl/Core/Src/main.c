/* USER CODE BEGIN Header */
/*
  Author: Ajit Thapa Magar
  Date: 8/20/2026
  Project: 4DOF Robotic Arm 
*/
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PCA9685_ADDRESS 0x80 //original 7-bit address is 0x40 but bit shifted by 1 for 8-bits.
#define MODE1 0x00 //the main register where bits are manipulated to sleep or wake the chip.
#define PRESCALE_ADDRESS 0xFE
#define CHANNEL_REGISTER 0x06
#define claw_open 18
#define claw_close 136 //angle for opening and closing based on version 1

//measurements of the robot
#define L1 112.83
#define L2 112.75
#define L3 130
#define BASE_HEIGHT 140
#define RAD_TO_DEG 57.2957795
#define DEG_TO_RAD 0.01745329

// The starting home position of your arm
// The starting home position of your arm (Safe tall standby)
float current_x = 100.0;
float current_y = 0.0;
float current_z = 150.0;
float current_pitch = -90.0; // Add this so the arm remembers its wrist angle


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void PCA9685_init(I2C_HandleTypeDef *hi2c);
void PCA9685_setPWM(I2C_HandleTypeDef *hi2c, uint8_t servo_channel, uint16_t on_tick, uint16_t off_tick);
void PCA9685_SetServoAngle(uint8_t Channel, float Angle);
void setAllAngle(float waist, float shoulder, float elbow, float wrist, float claw);
void reachCoordinate(float x, float y, float z, float pitch_deg, float claw_angle);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//initialize the board by making it sleep, and applying prescale so that the information can be passed in a similar frequency
//that the servo wants.
void PCA9685_init(I2C_HandleTypeDef *hi2c){
	uint8_t sleep_mode = 0x10;
	uint8_t prescale_val = 121; //25MHz/(2^8 steps * target Hz (50 Hz) => 121
	uint8_t wake_mode = 0x20;

	//make the chip sleep before writing the prescaler.
	HAL_I2C_Mem_Write(hi2c, PCA9685_ADDRESS, MODE1, I2C_MEMADD_SIZE_8BIT, &sleep_mode, 1, 100);
	HAL_Delay(10);

	//write prescale
	HAL_I2C_Mem_Write(hi2c, PCA9685_ADDRESS, PRESCALE_ADDRESS, I2C_MEMADD_SIZE_8BIT, &prescale_val, 1, 100);
	HAL_Delay(10);

	HAL_I2C_Mem_Write(hi2c, PCA9685_ADDRESS, MODE1, I2C_MEMADD_SIZE_8BIT, &wake_mode, 1, 100);
	HAL_Delay(10);
}

//method that sends command to the channel registers
void PCA9685_setPWM(I2C_HandleTypeDef *hi2c, uint8_t servo_channel, uint16_t on_tick, uint16_t off_tick)
{
	uint8_t start_register  = CHANNEL_REGISTER + (4 * servo_channel);

	//store the date into an array
	uint8_t pwm_data[4];
	/*
	 * on low 0x06
	 * on high 0x07
	 * off low 0x08
	 * off high 0x09  This is for default channel of 0.
	 */
	pwm_data[0] = on_tick & 0xFF; //this cuts off the upper 8 bits out of 16 bits data.
	pwm_data[1] = (on_tick >> 8) & 0xFF; //this slides the data 8 bits to the right and cuts off the upper 8 bits so now its only 8 bits remaining
	pwm_data[2] = off_tick & 0xFF; //this cuts off the upper 8 bits out of 16 bits data.
	pwm_data[3] = (off_tick >> 8) & 0xFF;

	//send the data
	HAL_I2C_Mem_Write(hi2c, PCA9685_ADDRESS, start_register, I2C_MEMADD_SIZE_8BIT, pwm_data, 4, 100);
}

void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
  float Value;
  // 50 Hz servo then 4095 Value --> 20 milliseconds
  // 0 degree --> 0.5 ms(102.4 Value) and 180 degree --> 2.5 ms(511.9 Value)
  uint16_t smallTick = 102.4;
  uint16_t bigTick = 511.9;
  Value = (Angle * (bigTick - smallTick) / 180.0) + smallTick;
  PCA9685_setPWM(&hi2c1, Channel, 0, (uint16_t)Value);
//  PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

//helper to code all parts at once
void setAllAngle(float waist, float shoulder, float elbow, float wrist, float claw)
{
	PCA9685_SetServoAngle(0, waist);
	PCA9685_SetServoAngle(1, shoulder);
	PCA9685_SetServoAngle(2, elbow);
	PCA9685_SetServoAngle(3, wrist);
	PCA9685_SetServoAngle(8, claw);
}


void reachCoordinate(float x, float y, float z, float pitch_deg, float claw_angle)
{
	//base angle and reach
	float waist_angle_rad = atan2(y,x);
	float r = sqrt((x * x) + (y * y)); //reach distance from the center

	//values based on wrist
	float pitch_rad = pitch_deg * DEG_TO_RAD;
	float r_new = r - L3 * cos(pitch_rad); // r - x where x = right side from L3 -> pitch deg, this finds the new coordinate based on the length and angle
	float z_new = z - BASE_HEIGHT - (L3 * sin(pitch_rad)); // similar to the above where it finds a new coordinate.

	//check to see if the coordinate is reachable
	float invisible_length = sqrt( (r_new * r_new) + (z_new * z_new));
	if(invisible_length > L1 + L2) //checks if the invisible line from shoulder to coordinate is smaller than both length combined.
	{
		return; //Not possible.
	}

	//find the angles for the shoulder and elbow
	//shoulder
	float shoulder_angle1_rad = acos( ((L1 * L1) + (invisible_length * invisible_length) - (L2 * L2)) / (2 * L1 * invisible_length)); //law of cosines
	float shoulder_angle2_rad = atan2(z_new, r_new); //other angle
	float shoulder_angle_deg = (shoulder_angle1_rad + shoulder_angle2_rad) * RAD_TO_DEG;
	//elbow
	float elbow_angle_rad  = acos ( ((L1 * L1) + (L2 * L2) - (invisible_length * invisible_length)) / (2 * L1 * L2)); //law of cosines
	float elbow_angle_deg = 180 - (elbow_angle_rad * RAD_TO_DEG); //included 180 to get the angle with respect to shoulder.

	//wrist angle
	float wrist_angle_deg = pitch_deg - (shoulder_angle_deg - elbow_angle_deg);

	//Account for change in direction of angles

	//waist 0=CW, 90=Center, 180=CCW
	float servo_waist = 90.0 + (waist_angle_rad * RAD_TO_DEG);

	//shoulder 180 -> back 0 -> forward
	float servo_shoulder = shoulder_angle_deg;

	//elbow
	float servo_elbow = 90.0 + elbow_angle_deg;

	// Wrist: 0=Forward, 90=Straight, 180=Back
	float servo_wrist = 90.0 + wrist_angle_deg;

	//execute
	setAllAngle(servo_waist, servo_shoulder, servo_elbow, servo_wrist, claw_angle);
}

void smoothMoveTo(float target_x, float target_y, float target_z, float target_pitch, float claw_angle, int steps, int step_delay_ms)
{
    //Calculate how far to move in each direction per step
    float dx = (target_x - current_x) / steps;
    float dy = (target_y - current_y) / steps;
    float dz = (target_z - current_z) / steps;
    float dpitch = (target_pitch - current_pitch) / steps; //new pitch

    for (int i = 1; i <= steps; i++)
    {
        float step_x = current_x + (dx * i);
        float step_y = current_y + (dy * i);
        float step_z = current_z + (dz * i);
        float step_pitch = current_pitch + (dpitch * i);

        reachCoordinate(step_x, step_y, step_z, step_pitch, claw_angle);
        HAL_Delay(step_delay_ms);
    }

    //update the main memory
    current_x = target_x;
    current_y = target_y;
    current_z = target_z;
    current_pitch = target_pitch;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // If the PCA9685 does NOT respond, blink the PA8 LED rapidly and freeze
  if (HAL_I2C_IsDeviceReady(&hi2c1, PCA9685_ADDRESS, 3, 100) != HAL_OK) {
      while (1) {
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
          HAL_Delay(100);
      }
  }

  	PCA9685_init(&hi2c1);
	//Starting position
	reachCoordinate(current_x, current_y, current_z, current_pitch, claw_open);
	HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  	//Hover directly over the 30mm cube
	        smoothMoveTo(150.0, 0.0, 100.0, -90.0, claw_open, 20, 15);
	        HAL_Delay(500);

	        //Drop to exactly Z=15 (center of the 30mm cube)
	        smoothMoveTo(150.0, 0.0, 25.0, -90.0, claw_open, 20, 20);
	        HAL_Delay(200);

	        //Snap the claw shut
	        PCA9685_SetServoAngle(8, claw_close);
	        HAL_Delay(800);

	        //Pull it straight up to clear the desk
	        smoothMoveTo(150.0, 0.0, 100.0, -90.0, claw_close, 20, 15);
	        HAL_Delay(500);

	        //Linear tracking
			// Moving 100mm -> 100 steps ensures a waypoint every 1 millimeter.
			smoothMoveTo(150.0, 100.0, 100.0, -90.0, claw_close, 100, 10);
			HAL_Delay(200);

			//moving 200mm (from 100 to -100) -> 200 steps to maintain 1mm resolution.
			smoothMoveTo(150.0, -100.0, 100.0, -90.0, claw_close, 200, 10);
			HAL_Delay(200);

			//Show the picked item to camera
			smoothMoveTo(150.0, 0.0, 120.0, -45.0, claw_close, 100, 10);
			HAL_Delay(1000); // Pause for the camera

			// diagonally also requires high steps to stay linear
			smoothMoveTo(50.0, 150.0, 100.0, -90.0, claw_close, 150, 10);
			HAL_Delay(500);

	        //Descend to Z=15 and release
	        smoothMoveTo(50.0, 150.0, 20.0, -90.0, claw_close, 20, 15);
	        HAL_Delay(200);
	        PCA9685_SetServoAngle(8, claw_open);
	        HAL_Delay(800);

	        //Return to Standby tall posture
	        smoothMoveTo(current_x, current_y, 150.0, -90.0, claw_open, 20, 15);
	        smoothMoveTo(100.0, 0.0, 150.0, -90.0, claw_open, 60, 15);

	        // Wait 3 seconds before repeating the showcase
	        HAL_Delay(1000);
//	        smoothMoveTo(100.0, 0.0, 150.0, -90.0, claw_open, 60, 15);
	        for(int  i = 0; i < 3; i++){
		        PCA9685_SetServoAngle(3, 90+45);
		        HAL_Delay(300);
		        PCA9685_SetServoAngle(3, 90-45);
		        HAL_Delay(300);
	        }
	        HAL_Delay(6000);


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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

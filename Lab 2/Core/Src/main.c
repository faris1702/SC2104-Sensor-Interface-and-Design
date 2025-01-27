	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "MPU6050.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
 I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t IMU_ADDR = 0x68<<1;
IMU_Data imu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
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
  uint8_t status = IMU_Initialise(&imu, &hi2c2, &huart3);
  char sbuf[64];
  float roll_a=0.0, pitch_a=0.0;
  float dt = 0.0;
  uint32_t millisOld = 0, millisNow = 0;
  float roll_g_now = 0.0, pitch_g_now = 0.0, yaw_g_now = 0.0;
  float roll_g_old = 0.0, pitch_g_old = 0.0, yaw_g_old = 0.0;
  float CF_roll, CF_pitch, CF_roll_old, CF_pitch_old;
  float w;
  float var_gyro = 16.0, var_acc = 9.0;
  float KG_roll=0.0, KG_pitch=0.0, KF_roll=0.0, KF_pitch=0.0;
  float var_groll=0.0,var_gpitch=0.0;
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
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
    w = 0.7;
    CF_roll = 0.0;
    CF_pitch = 0.0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	IMU_AccelRead(&imu);
	IMU_GyroRead(&imu);
	//0:X, 1:Y, 2:Z


	//--------------------Accelerometer----------------------------
//    sprintf(sbuf, "%5.2f,%5.2f,%5.2f,", imu.acc[0],imu.acc[1],imu.acc[2]);
//    HAL_UART_Transmit(&huart3, sbuf, strlen(sbuf), HAL_MAX_DELAY);

    //Finding angle
    //Roll angle = tan-1(x/z)
	roll_a = atan2(imu.acc[0],imu.acc[2]);

    //Pitch angle = tan-1(y/z)
    pitch_a = atan2(imu.acc[1],imu.acc[2]);

    //convert to radian
    roll_a = roll_a * 57.3;
    pitch_a = pitch_a * 57.3;
//    sprintf(sbuf, "%5.2f,%5.2f", roll_a, pitch_a);
//    HAL_UART_Transmit(&huart3, sbuf, strlen(sbuf), HAL_MAX_DELAY);

    //--------------------Gyroscope-------------------------------------
//    sprintf(sbuf,"%5.2f,%5.2f,%5.2f", imu.gyro[0],imu.gyro[1],imu.gyro[2]);
//	HAL_UART_Transmit(&huart3, sbuf, strlen(sbuf), HAL_MAX_DELAY);

    millisNow = HAL_GetTick();
    dt = (millisNow - millisOld) * 0.001;  //elapsed time in milliseconds
    millisOld = millisNow;

    //Finding angle
    //a[n] = a[n-1] + imu.gyro * t
    roll_g_old = roll_g_now;
    pitch_g_old = pitch_g_now;
    yaw_g_old = yaw_g_now;


      roll_g_now += imu.gyro[0] * dt;
      pitch_g_now += imu.gyro[1]* dt;
	  yaw_g_now += imu.gyro[2] * dt;

//      sprintf(sbuf,"%5.2f,%5.2f,%5.2f\r\n", roll_g_now,pitch_g_now,yaw_g_now);
//    roll_g_now = roll_g_old + imu.gyro[0] * dt;    //roll
//    pitch_g_now = pitch_g_old + imu.gyro[1] * dt;   //pitch
//    yaw_g_now = yaw_g_old + imu.gyro[2] * dt;     //yaw

//    roll_g_now += imu.gyro[0] * dt;    //roll
//    pitch_g_now += imu.gyro[1] * dt;   //pitch
//    yaw_g_now += imu.gyro[2] * dt;     //yaw

//    sprintf(sbuf,"%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\r\n", roll_a, pitch_a, roll_g_now, pitch_g_now, yaw_g_now,);
//	HAL_UART_Transmit(&huart3, sbuf, strlen(sbuf), HAL_MAX_DELAY);


    //---------------Sensor Fusion---------------------

    //Complementary Filter.....................
    //a[n] = (1-w)*(a_a[n]) + w*(pitch_g_now)
    w = 0.9;

    CF_roll_old = CF_roll;
    CF_roll = (1-w)*(roll_a) + w * (CF_roll_old + imu.gyro[0]*dt);    //roll

    CF_pitch_old = CF_pitch;
    CF_pitch = (1-w)*(pitch_a) + w * (CF_pitch_old + imu.gyro[1]*dt); //pitch

//    sprintf(sbuf,"%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\r\n", roll_a, pitch_a, roll_g_now, pitch_g_now, yaw_g_now,CF_roll,CF_pitch);
//    HAL_UART_Transmit(&huart3, sbuf, strlen(sbuf), HAL_MAX_DELAY);


    //Kalman Filter....................
    //1. Get readings from Gyro and Accel
    //2. Calculate roll and pitch angles from Accel
    //   roll_a, pitch_a
    //3. Calculate roll and pitch angles from Gyro
    //   roll_g_now, pitch_g_now
    //4. Calculate uncertainty/variance of Gyro pitch and roll angles
    //   -- old_var_gyro = old_var_gyro + (dt^2 * var_gyro)
    //5. Calculate Kalman Gain
    //   -- KG = var_gyro / (var_gyro + var_acc)
    //6. Use KG to update the estimated roll/pitch angles of Gyroscope
    //   obtained in step 3
    //   -- new_x_gyro = old_x_gyro + KG * (x_acc - old_x_gyro)
    //7. Use KG to update uncertainty of Gyro readings in step 4
    //   -- new_var_gyro = (1-KG)*old_var_gyro
    //8. Repeat from step 1

    //Variance values
    //var_acc = 9
    //var_gyro = 16
    //init_var_gyro = 4

//
//    var_gyro_old = var_gyro_old + (dt*dt*var_gyro);
//    KG_roll = var_gyro/(var_gyro + var_acc);
//    //roll
//    roll_KF = roll_g_now + KG_roll * (roll_a - roll_g_now);
//    //pitch
//    pitch_KF = pitch_g_now + KG_roll * (pitch_a - pitch_g_now);
//    var_gyro = (1 - KG_roll) * var_gyro_old;

    KF_roll = KF_roll + imu.gyro[0]*dt*dt;
    KF_pitch = KF_pitch + imu.gyro[1]*dt*dt;

    var_groll = var_groll + dt*dt*var_gyro;
    var_gpitch = var_gpitch + dt*dt*var_gyro;

    KG_roll = var_groll/(var_groll + var_acc);
    KG_pitch = var_gpitch/(var_gpitch+var_acc);

    KF_roll = KF_roll + KG_roll*(roll_a - KF_roll);
    KF_pitch = KF_pitch + KG_pitch*(pitch_a - KF_pitch);

    var_groll = (1-KG_roll)*var_groll;
    var_gpitch = (1-KG_pitch)*var_gpitch;

    sprintf(sbuf,"%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\r\n", roll_a, pitch_a, roll_g_now, pitch_g_now, yaw_g_now,CF_roll,CF_pitch, KF_roll,KF_pitch);
    HAL_UART_Transmit(&huart3, sbuf, strlen(sbuf), HAL_MAX_DELAY);
    //finish
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

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

/* USER CODE BEGIN Header */
/**
  *******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for the WRO Future Engineers Self-Driving Car
  *
  * This program controls the self-driving robotic vehicle for the WRO Future Engineers
  * category challenge, focusing on autonomous navigation of a parkour. The vehicle
  * utilizes sensor fusion and computer vision for state estimation and action planning.
  *
  * The STM32 microcontroller interfaces with a motor shield to control the vehicle's
  * motion, and incorporates an IMU (Inertial Measurement Unit) and distance sensors
  * for environmental awareness. The vehicle is designed to use optimal strategies for
  * mission-solving, stability, and control, with additional focus on precision and
  * real-time sensor data handling.
  *
  * Key Functionalities:
  * - Motor control through a motor shield based on sensor data and commands from the
  *   main SBC (Single-Board Computer).
  * - Sensor fusion using IMU and distance sensors to estimate the state of the vehicle
  *   and parkour.
  * - Implementation of control algorithms for navigation and obstacle avoidance.
  * - Periodic communication and data exchange with the main SBC for coordinated actions.
  * - Integration of planning strategies to ensure mission stability and efficient solving.
  *
  * The system also includes safety mechanisms and feedback to ensure the vehicle
  * behaves predictably and autonomously within the constraints of the parkour environment.
  *
  * Copyright (c) 2024 AFTON Team.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <motor_drv.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "sensor_vl53l7ch.h"
#include "sensor_imu.h"
#include "serial_dd_protocol.h"
#include "sensor_vl53l4cd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* GPIO defines for VL53L4CD XSHUT pins */
#define VL53L4CD_LEFT_XSHUT_GPIO_Port GPIOA
#define VL53L4CD_LEFT_XSHUT_Pin       GPIO_PIN_5
#define VL53L4CD_RIGHT_XSHUT_GPIO_Port GPIOA
#define VL53L4CD_RIGHT_XSHUT_Pin       GPIO_PIN_7
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Global variables */
VL53L4CD_ResultsData_t left_results;
VL53L4CD_ResultsData_t right_results;
Dev_t left_sensor_address = 0x54;  // New I2C address
Dev_t right_sensor_address = 0x56; // New I2C address

/* Global motor speed (0–100%) */
uint8_t global_motor_speed = 50;  // default speed

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern volatile uint8_t new_message_ready;
extern volatile DD_Uart_Message received_message;
extern TIM_HandleTypeDef htim3;  // or htim9, htim10 depending on your config

TIM_HandleTypeDef* motor_pwm_tim = &htim3;
uint32_t motor_pwm_channel = TIM_CHANNEL_1;  // or whatever channel you use

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* Helper to enable one sensor at a time */
void VL53L4CD_Enable(GPIO_TypeDef* port, uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

void VL53L4CD_Disable(GPIO_TypeDef* port, uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_Delay(10);
}

/* Initialization routine for both sensors */
void init_dual_vl53l4cd_sensors() {
    VL53L4CD_Disable(VL53L4CD_LEFT_XSHUT_GPIO_Port, VL53L4CD_LEFT_XSHUT_Pin);
    VL53L4CD_Disable(VL53L4CD_RIGHT_XSHUT_GPIO_Port, VL53L4CD_RIGHT_XSHUT_Pin);

    // Init LEFT
    VL53L4CD_Enable(VL53L4CD_LEFT_XSHUT_GPIO_Port, VL53L4CD_LEFT_XSHUT_Pin);
    VL53L4CD_SensorInit(0x52);  // Default address
    VL53L4CD_SetI2CAddress(0x52, left_sensor_address);
    VL53L4CD_StartRanging(left_sensor_address);

    // Init RIGHT
    VL53L4CD_Enable(VL53L4CD_RIGHT_XSHUT_GPIO_Port, VL53L4CD_RIGHT_XSHUT_Pin);
    VL53L4CD_SensorInit(0x52);
    VL53L4CD_SetI2CAddress(0x52, right_sensor_address);
    VL53L4CD_StartRanging(right_sensor_address);
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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  motor_init(&htim3, TIM_CHANNEL_1, MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin);

	printf("Hello, UART via CP2102!\n");

	// IMU sensor variables
	int16_t ax, ay, az, gx, gy, gz;

//	// Initialize IMU
//	if (sensor_imu_init(&hi2c2) != HAL_OK) {
//	  printf("Failed to initialize IMU\n");
//	  while (1);
//	}
	/*********************************/
	/*   VL53L7CX ranging variables  */
	/*********************************/

	VL53L7CX_Configuration dev;
	VL53L7CX_ResultsData results;
	uint8_t status, isReady, loop = 0;


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/
	status = sensor_vl53l7ch_init(&dev);
	if (status){
	printf("VL53L7CX ULD Loading failed\n");
	return status;
	}

	/*********************************/
	/*         Ranging loop          */
	/*********************************/
	status = sensor_vl53l7ch_start_ranging(&dev);
	if (status){
		printf("VL53L7CX Ranging loop failed\n");
		return status;
	}


//	// -------------------------------------------------------
//	// Now, immediately after init/start, read one 8×8 frame
//	// (or wrap in a while(1) to continuously print for debugging).
//	// -------------------------------------------------------
//	while (1) {
//	    // 1) Wait until data is ready
//	    status = sensor_vl53l7ch_check_data_ready(&dev, &isReady);
//	    if (status != 0) {
//	        printf("Error checking data ready: %u\n", status);
//	        break;
//	    }
//	    if (!isReady) {
//	        HAL_Delay(10);
//	        continue;
//	    }
//
//	    // 2) Get the 8×8 ranging data
//	    status = sensor_vl53l7ch_get_ranging_data(&dev, &results);
//	    if (status != 0) {
//	        printf("Error getting VL53L7CX data: %u\n", status);
//	        break;
//	    }
//
//	    // 1) Print the raw 8×8 distance matrix
//	   printf("Distances (mm):\r\n");
//	   for (uint8_t row = 0; row < 8; row++) {
//		   for (uint8_t col = 0; col < 8; col++) {
//			   uint8_t idx = row * 8 + col;
//			   // Print each distance as a 4‐digit field
//			   printf("%4u", results.distance_mm[idx]);
//			   if (col < 7) {
//				   printf(" ");
//			   }
//		   }
//		   printf("\r\n");
//	   }
//	   printf("\r\n");  // blank line separator
//
//	   // 2) Print the raw 8×8 status matrix
//	   printf("Statuses:\r\n");
//	   for (uint8_t row = 0; row < 8; row++) {
//		   for (uint8_t col = 0; col < 8; col++) {
//			   uint8_t idx = row * 8 + col;
//			   // Print each status as a 2‐digit decimal (pad if desired)
//			   printf("%2u", results.target_status[idx]);
//			   if (col < 7) {
//				   printf(" ");
//			   }
//		   }
//		   printf("\r\n");
//	   }
//	   printf("\r\n");  // blank line separator
//
//	    // 5) Delay before next frame
//	    HAL_Delay(100);
//	}


	/*********************************/
	/*   VL53L4CD   variables        */
	/*********************************/
	VL53L4CD_ResultsData_t left_results;
	VL53L4CD_ResultsData_t right_results;
	Dev_t left_sensor_address = 0x54;  // New I2C address
	Dev_t right_sensor_address = 0x56; // New I2C address


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/
	init_dual_vl53l4cd_sensors();

	/*********************************/
	/*         Read Data vl53l4cd    */
	/*********************************/
	// --- VL53L4CD Test Read After Init ---
	uint8_t isReadyL = 0, isReadyR = 0;
	char test_buf[100];

	HAL_Delay(50);  // Allow sensor to stabilize

	VL53L4CD_CheckForDataReady(left_sensor_address, &isReadyL);
	if (isReadyL) {
	    VL53L4CD_ClearInterrupt(left_sensor_address);
	    VL53L4CD_GetResult(left_sensor_address, &left_results);
	    snprintf(test_buf, sizeof(test_buf), "INIT TEST - LEFT: %d mm\r\n", left_results.distance_mm);
//	    HAL_UART_Transmit(&huart1, (uint8_t *)test_buf, strlen(test_buf), 100);
	}

	VL53L4CD_CheckForDataReady(right_sensor_address, &isReadyR);
	if (isReadyR) {
	    VL53L4CD_ClearInterrupt(right_sensor_address);
	    VL53L4CD_GetResult(right_sensor_address, &right_results);
	    snprintf(test_buf, sizeof(test_buf), "INIT TEST - RIGHT: %d mm\r\n", right_results.distance_mm);
//	    HAL_UART_Transmit(&huart1, (uint8_t *)test_buf, strlen(test_buf), 100);
	}


	uint32_t led_blink_counter = 0;  // Counter for LED blinking

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 // LED blinking every 500ms (e.g., increment counter every 100ms)
	if (led_blink_counter++ >= 500) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // Toggle LED
		led_blink_counter = 0;  // Reset the counter
	}

	// Check if a new message is received
	if (new_message_ready) {
		// Process the received_message
		new_message_ready = 0;  // Reset flag

		switch (received_message.command) {
			case 0x0001:  // READ_GYRO
//				if (sensor_imu_read_gyro(&hi2c2, &gx, &gy, &gz) == HAL_OK) {
//					// Prepare data as a string
//					char gyro_data[50];
//					snprintf(gyro_data, sizeof(gyro_data), "Gyro: X=%d, Y=%d, Z=%d", gx, gy, gz);
//
//					// Send response to PC
//					send_response(0x0001, gyro_data);
//				}
				break;
			case 0x0002:  // READ_ACCEL
//				if (sensor_imu_read_accel(&hi2c2, &ax, &ay, &az) == HAL_OK) {
//					// Prepare data as a string
//					char accel_data[50];
//					snprintf(accel_data, sizeof(accel_data), "Accel: X=%d, Y=%d, Z=%d", ax, ay, az);
//
//					// Send response to PC
//					send_response(0x0002, accel_data);
//				}
				break;
			case 0x0003:  // READ_DISTANCE_SENSOR
			{
			    status = sensor_vl53l7ch_check_data_ready(&dev, &isReady);
			    if (!isReady) {
			        send_response(0x0003, "Data not ready");
			        break;
			    }

			    status = sensor_vl53l7ch_get_ranging_data(&dev, &results);
			    if (status) {
			        send_response(0x0003, "Error getting data");
			        break;
			    }

			    // Build a single buffer containing both the 8×8 distances and the 8×8 statuses
			    char distance_data[2048];
			    memset(distance_data, 0, sizeof(distance_data));
			    char *ptr = distance_data;
			    int   offset = 0;

			    // 1) Header: stream count
			    offset += snprintf(ptr + offset,
			                       sizeof(distance_data) - offset,
			                       "Stream %u\r",
			                       dev.streamcount);

			    // 2) Print the 8×8 distance matrix
			    offset += snprintf(ptr + offset,
			                       sizeof(distance_data) - offset,
			                       "Distances (mm):\r");
			    for (uint8_t row = 0; row < 8; row++) {
			        for (uint8_t col = 0; col < 8; col++) {
			            uint8_t idx = row * 8 + col;
			            offset += snprintf(ptr + offset,
			                               sizeof(distance_data) - offset,
			                               "%4u",
			                               results.distance_mm[idx]);
			            if (col < 7) {
			                offset += snprintf(ptr + offset,
			                                   sizeof(distance_data) - offset,
			                                   " ");
			            }
			        }
			        offset += snprintf(ptr + offset,
			                           sizeof(distance_data) - offset,
			                           "\r");
			    }
			    offset += snprintf(ptr + offset,
			                       sizeof(distance_data) - offset,
			                       "\r");

			    // 3) Print the 8×8 status matrix
			    offset += snprintf(ptr + offset,
			                       sizeof(distance_data) - offset,
			                       "Statuses:\r");
			    for (uint8_t row = 0; row < 8; row++) {
			        for (uint8_t col = 0; col < 8; col++) {
			            uint8_t idx = row * 8 + col;
			            offset += snprintf(ptr + offset,
			                               sizeof(distance_data) - offset,
			                               "%3u",
			                               results.target_status[idx]);
			            if (col < 7) {
			                offset += snprintf(ptr + offset,
			                                   sizeof(distance_data) - offset,
			                                   " ");
			            }
			        }
			        offset += snprintf(ptr + offset,
			                           sizeof(distance_data) - offset,
			                           "\r");
			    }
			    offset += snprintf(ptr + offset,
			                       sizeof(distance_data) - offset,
			                       "\r");

			    // 4) Send the combined buffer back
			    send_response_len(0x0003, distance_data, offset);
			    HAL_Delay(5);
			    break;
			}


			case 0x0004:  // FORWARD
			{
			    motor_forward(global_motor_speed);
			    send_response(0x0004, "Forward");
			    break;
			}
			case 0x0005:  // REVERSE
			{
			    motor_reverse(global_motor_speed);
			    send_response(0x0005, "Reverse");
			    break;
			}
			case 0x0101:  // FORWARD with degrees
			{
				uint16_t deg = received_message.data[0] | (received_message.data[1] << 8);
			    motor_forward_degrees(deg,  global_motor_speed);
			    send_response(0x0101, "Moved Forward by degrees");
			    break;
			}
			case 0x0102:  // REVERSE with degrees
			{
				uint16_t deg = received_message.data[0] | (received_message.data[1] << 8);
			    motor_reverse_degrees(deg,  global_motor_speed);
			    send_response(0x0102, "Moved Reverse by degrees");
			    break;
			}
			case 0x0103:  // STOP
			{
			  motor_stop();
			  send_response(0x0103, "Stopped");
			  break;
			}
			case 0x0104:  // SPEED
			{
			    global_motor_speed = received_message.data[0];
			    motor_set_speed(global_motor_speed);
			    send_response(0x0104, "Speed updated");
			    break;
			}
			/* In the main command switch-case: */
			case 0x0105:  // READ VL53L4CD LEFT & RIGHT
			{
			    char buf[100];
			    uint8_t isReadyL = 0, isReadyR = 0;
			    int len = 0;

			    VL53L4CD_CheckForDataReady(left_sensor_address, &isReadyL);
			    if (isReadyL) {
			        VL53L4CD_ClearInterrupt(left_sensor_address);
			        VL53L4CD_GetResult(left_sensor_address, &left_results);
			    }

			    VL53L4CD_CheckForDataReady(right_sensor_address, &isReadyR);
			    if (isReadyR) {
			        VL53L4CD_ClearInterrupt(right_sensor_address);
			        VL53L4CD_GetResult(right_sensor_address, &right_results);
			    }

			    // Build combined message
			    len = snprintf(buf, sizeof(buf),
			                   "LEFT: %d mm | RIGHT: %d mm",
			                   isReadyL ? left_results.distance_mm : -1,
			                   isReadyR ? right_results.distance_mm : -1);

			    send_response_len(0x0105, buf, len);
			    break;
			}


			default:
				printf("Unknown command received\n");
				break;
		}


		// Reset the message length after processing
		received_message.length = 0;
	}

    // Read encoder data for RPM calculation
    uint32_t rpm = motor_get_rpm();
//    printf("Motor RPM: %lu\n", rpm);

	// Delay to make sure the LED toggle is visible
	HAL_Delay(1);  // Small delay (1ms) to allow for blinking without blocking other processes
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  status = sensor_vl53l7ch_stop_ranging(&dev);
  printf("End of ULD demo\n");
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  // Enable UART RX interrupt
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_DIR_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_DIR_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Enable NVIC interrupts for EXTI lines 10-15 (PB12 and PB13 fall here)
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  GPIO_InitStruct.Pin = VL53L4CD_LEFT_XSHUT_Pin | VL53L4CD_RIGHT_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_12 || GPIO_Pin == GPIO_PIN_13) {
        uint8_t ch_a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
        uint8_t ch_b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
//        motor_encoder_update(ch_a, ch_b);
        motor_encoder_update(ch_b, ch_a);
        // DEBUG:
//        printf("Interrupt: ENC A=%d, B=%d\n", ch_a, ch_b);
    }
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
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

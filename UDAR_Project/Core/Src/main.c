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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t msg_buffer[64] = { 0 };
volatile uint8_t byte[2] = { 0 };
uint8_t test = 0x55;
volatile uint16_t distance = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
int take_measure = 0;		// 0 - means no object   1 - means ys object
int first = 0;
int second = 0;
/* USER CODE BEGIN PFP */
static void ADC_Select_CH(int CH);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rcv_intpt_flag = 0;
static void ADC_Select_CH(int CH) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	switch (CH) {
	case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 6:
		sConfig.Channel = ADC_CHANNEL_6;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 7:
		sConfig.Channel = ADC_CHANNEL_7;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 8:
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 9:
		sConfig.Channel = ADC_CHANNEL_9;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 10:
		sConfig.Channel = ADC_CHANNEL_10;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 11:
		sConfig.Channel = ADC_CHANNEL_11;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 12:
		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 13:
		sConfig.Channel = ADC_CHANNEL_13;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 14:
		sConfig.Channel = ADC_CHANNEL_14;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 15:
		sConfig.Channel = ADC_CHANNEL_15;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	int TIM3_Ch1_DCVAL = 1500;
	int TIM3_Ch2_DCVAL = 1500;
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	TIM3->PSC = 16 - 1;
	TIM3->ARR = 20000 - 1;
	TIM3->CCR1 = TIM3_Ch1_DCVAL;
	TIM3->CCR2 = TIM3_Ch1_DCVAL;
	int mode = 0; // 0 auto, 1 manual

	int object_count = 0;
	int degree_to_center = 0;
	int angular_width = 0;
	int pov = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	TIM3_Ch1_DCVAL = (1500);
	TIM3_Ch2_DCVAL = (500);

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// AUTO
		mode = HAL_GPIO_ReadPin(GPIOC, B1_Pin);

		if (mode == 1) {
			TIM3->CCR1 = TIM3_Ch1_DCVAL;
			TIM3->CCR2 = TIM3_Ch2_DCVAL;

			while (TIM3_Ch2_DCVAL <= 2500) {
				if (distance > 1000) {	// No object detected
					HAL_GPIO_WritePin(GPIOB, BLUE_Pin | GREEN_Pin | RED_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GREEN_Pin, GPIO_PIN_SET);
					if (take_measure == 1) {	// FLAG that makes it read once
						second = TIM3_Ch2_DCVAL;
						TIM3->CCR2 = (first + second) / 2;
						object_count++;
						// SET UP LASER, have a for loop 0 -> 3
						// Turn laser on wait 250 turn off wait 250
						// can get rid of that delasy

						int w = 0;
						for (w; w < 3; w++) {
							HAL_GPIO_WritePin(LASERn_GPIO_Port, LASERn_Pin,
									GPIO_PIN_RESET);
							HAL_Delay(200);
							HAL_GPIO_WritePin(LASERn_GPIO_Port, LASERn_Pin,
									GPIO_PIN_SET);
							HAL_Delay(200);
						}

						//setting the right values for the print statement
						degree_to_center = ((((first + second) / 2) / 20) * 1.8)
								- 135;
						pov = 1900 - 1140 - 211;
						angular_width = second - first - pov;

						//printing the values
						sprintf((char*) msg_buffer,
								"\r\n Object #:%d  Degree:%d  Distance:%d  Angular-Width:%d\n",
								object_count, degree_to_center, distance,
								angular_width);
						HAL_UART_Transmit(&huart6, msg_buffer,
								strlen((char*) msg_buffer), 500);
					}
					take_measure = 0;
				}
				TIM3_Ch2_DCVAL += 20;
				HAL_Delay(200);
				TIM3->CCR2 = TIM3_Ch2_DCVAL;
				HAL_UART_Receive_IT(&huart1, &byte, 2);
				HAL_UART_Transmit(&huart1, &test, 1, 500);
				while (rcv_intpt_flag == (00)) {
				};
				distance = (byte[0] << 8) + byte[1];

				if (distance < 1000) {	// object sensed
					HAL_GPIO_WritePin(GPIOB, BLUE_Pin | GREEN_Pin | RED_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);

					if (take_measure == 0) {	// FLAG that makes it read once
						first = TIM3_Ch2_DCVAL;
					}
					take_measure = 1;
				}

				rcv_intpt_flag = 00;
			}

		}
		// MANUAL
		else if (mode == 0) {
			//set the intial postion
			TIM3_Ch1_DCVAL = (1500);
			TIM3_Ch2_DCVAL = (1500);
			TIM3->CCR1 = TIM3_Ch1_DCVAL;
			TIM3->CCR2 = TIM3_Ch2_DCVAL;
			//turn the led off
			HAL_GPIO_WritePin(LASERn_GPIO_Port, LASERn_Pin, GPIO_PIN_SET);

			while (1) {

				//----------------------------testing the analog stick

				//BLUE RGB
				/*Configure GPIO pin Output Level */
				HAL_GPIO_WritePin(GPIOB, BLUE_Pin | GREEN_Pin | RED_Pin,
						GPIO_PIN_RESET);
				/*Configure GPIO pin Output Level */
				HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_SET);
				HAL_Delay(200);

				//GREEN RGB
				HAL_GPIO_WritePin(GPIOB, BLUE_Pin | GREEN_Pin | RED_Pin,
						GPIO_PIN_RESET);
				/*Configure GPIO pin Output Level */
				HAL_GPIO_WritePin(GPIOB, GREEN_Pin, GPIO_PIN_SET);
				HAL_Delay(200);

				//RED RGB
				HAL_GPIO_WritePin(GPIOB, BLUE_Pin | GREEN_Pin | RED_Pin,
						GPIO_PIN_RESET);
				/*Configure GPIO pin Output Level */
				HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
				HAL_Delay(200);

				//---------------------------------------------------------------

				//read the joystick values
				ADC_Select_CH(9);
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 1000);
				uint8_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
//				sprintf((char*) msg_buffer,
//						"\r\n Digitized Analog Value for Y: %d", ADC_CH9);
//				HAL_UART_Transmit(&huart6, msg_buffer,
//						strlen((char*) msg_buffer), 500);
				HAL_ADC_Stop(&hadc1);

				ADC_Select_CH(14);
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 1000);
				uint8_t ADC_CH14 = HAL_ADC_GetValue(&hadc1);
//				sprintf((char*) msg_buffer,
//						"\r\n Digitized Analog Value for X: %d\n\n", ADC_CH14);
				HAL_ADC_Stop(&hadc1);

				HAL_UART_Receive_IT(&huart1, &byte, 2);
				HAL_UART_Transmit(&huart1, &test, 1, 500);
				while (rcv_intpt_flag == (00)) {
				};
				distance = (byte[0] << 8) + byte[1];
				sprintf((char) msg_buffer, "\r\n Distance Sensed (mm)= %d",
						distance);
				HAL_UART_Transmit(&huart6, msg_buffer,
						strlen((char) msg_buffer), 500);
				rcv_intpt_flag = 00;

				//------------------------------------------------------------

				//if the servo in the right range
				if (TIM3_Ch2_DCVAL <= 2500 && TIM3_Ch2_DCVAL >= 500) {
					//MOVEMENT FOR THE TILT
					if (ADC_CH9 < 69) {
						TIM3_Ch1_DCVAL -= 40;
					} else if (ADC_CH9 < 120) {
						TIM3_Ch1_DCVAL -= 20;
					} else if (ADC_CH9 < 140) {
						TIM3_Ch1_DCVAL += 0;
					} else if (ADC_CH9 < 181) {
						TIM3_Ch1_DCVAL += 40;
					} else if (ADC_CH9 < 256) {
						TIM3_Ch1_DCVAL += 20;
					}

					//MOVEMENT FOR THE PAN

					//------------------------------------------------------
					if (ADC_CH14 < 69) {
						TIM3_Ch2_DCVAL -= 40;
					} else if (ADC_CH14 < 120) {
						TIM3_Ch2_DCVAL -= 20;
					} else if (ADC_CH14 < 144) {
						TIM3_Ch2_DCVAL += 0;
					} else if (ADC_CH14 < 181) {
						TIM3_Ch2_DCVAL += 40;
					} else if (ADC_CH14 < 256) {
						TIM3_Ch2_DCVAL += 20;
					}

				} else {
					if (TIM3_Ch2_DCVAL > 2500) {
						TIM3_Ch2_DCVAL = 2500;
					} else if (TIM3_Ch2_DCVAL < 500) {
						TIM3_Ch2_DCVAL = 500;
					}
				}
				//set the values of the server
				TIM3->CCR1 = TIM3_Ch1_DCVAL;
				TIM3->CCR2 = TIM3_Ch2_DCVAL;
//			HAL_Delay(200);

//GET THE UART SENSOR VALUES AND PRINT TO PUTTY
//				HAL_UART_Receive_IT(&huart1, &byte, 2);
//				HAL_UART_Transmit(&huart1, &test, 1, 500);
//				while (rcv_intpt_flag == (00)) {
//				};
//				distance = (byte[0] << 8) + byte[1];
//				sprintf((char) msg_buffer, "\r\n Distance Sensed (mm)= %d",
//						distance);
//				HAL_UART_Transmit(&huart6, msg_buffer,
//						strlen((char) msg_buffer), 500);
//				rcv_intpt_flag = 00;

			};

		}

	}

}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 20000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500 - 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 2500 - 1;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LASERn_GPIO_Port, LASERn_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, BLUE_Pin | GREEN_Pin | RED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LASERn_Pin BLUE_Pin GREEN_Pin RED_Pin */
	GPIO_InitStruct.Pin = LASERn_Pin | BLUE_Pin | GREEN_Pin | RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		rcv_intpt_flag = 1;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

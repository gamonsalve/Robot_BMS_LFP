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
#include "RC_Model_KF_Vout_Vcb_for_MCU.h"  /* Simulink Model header file */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TESTING 0
#define NUMBER_OF_SAMPLES 15
#define NUMBER_OF_MEASURES 3
#define ADC_BUFFER_LEN NUMBER_OF_SAMPLES*NUMBER_OF_MEASURES
#define V_I_REF 1.647
#define V_GAIN 1.008856088
#define I_GAIN 1.006723716
#define R1 100000
#define ROUT 12000
#define I2C_RX_BUFFER_SIZE 20
#define I2C_RESPONSE_HEADER 'B'
#define I2C_MEASURES 6
#define SAMPLING_TIME_MS 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* UART Variables */
uint8_t rx_buffer[1]; //UART input byte
char input_data[50]; // String build byte by byte
char output_data[50]; // UART output string
char *token; // Variable for string split
/* Elapsed time Variables */
uint32_t t0 = 0; //Sampling time timer
uint32_t contactor_timer = 0;
uint16_t start_time = 0;
uint16_t elapsed_time = 0; // soc estimation elapsed time
uint8_t process_data = 0; //Flag to start processing data
uint32_t beat = 0; // Variable for heartbeat;
/* ADC Variables */
uint16_t adc_buffer[ADC_BUFFER_LEN]; // adc buffer
uint8_t adc_dma_complete = 0;
/* SoC  Variables */
float current = 0;
float voltage = 0;
float temperature = 0;
/* I2C Variables */
uint8_t i2c_rx_buffer[I2C_RX_BUFFER_SIZE];
char i2c_tx_buffer[I2C_RX_BUFFER_SIZE];
uint8_t master_reads = 0;
uint8_t master_writes = 0;
int i2c_counter = 0;
// I2C Bytes to send and structure (12);
typedef struct soc_measurement {
	uint8_t MSB, LSB;
} soc_measurement;
soc_measurement i2c_measurements[I2C_MEASURES];
/* Contactor Variables*/
typedef enum {
	off, precharge, positive, on
} CONTACTOR_STATES;
typedef enum {
	idle, i2c_start_request, precharge_cplt, positive_cplt
} CONTACTOR_EVENTS;
CONTACTOR_STATES contactor_state = off;
CONTACTOR_EVENTS contactor_transition = idle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void process_adc_buffer();
void create_i2c_buffer();
soc_measurement int_to_byte(float value);
soc_measurement uint_to_byte(uint16_t value);
void HIL_simulation();
void soc_estimator();
void contactors_control();
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
  RC_Model_KF_Vout_Vcb_for_MCU_initialize(); //Simulink Model initialization
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // ADC self-calibration routine, before starting ADC
	HAL_UART_Receive_IT(&huart2, rx_buffer, 1); //Wait for UART communication
	HAL_I2C_EnableListen_IT(&hi2c1);
	; //Enable the address listen mode in slave mode
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, ADC_BUFFER_LEN); // Start ADC conversion with transfer by DMA
	HAL_TIM_Base_Start(&htim6); //start timer 6 with interruptions
	create_i2c_buffer(); // Initialize the buffer to sent with 0
	t0 = HAL_GetTick(); // reset sampling time timer
	//Close both contactors
	HAL_GPIO_WritePin(PRECHARGE_RELAY_GPIO_Port, PRECHARGE_RELAY_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(POSITIVE_RELAY_GPIO_Port, POSITIVE_RELAY_Pin,
			GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		Heartbeat();
		// I2C Request has been received.
		if (master_writes == 1) {
			master_writes = 0; // clean flag
			// Master requested to write/transmit, then slave receive
			HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, i2c_rx_buffer,
			I2C_RX_BUFFER_SIZE, I2C_FIRST_FRAME);
		} else if (master_reads == 1) {
			master_reads = 0; // clean flag
			//Master requested to read/receive, then the slave transmit
			//1. Build data to send.
			create_i2c_buffer();
			// 2. Slave transmit buffer with data
			HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t*) i2c_tx_buffer,
			I2C_RX_BUFFER_SIZE, I2C_FIRST_FRAME);
			// Let's check if contactors are connected
			if (contactor_state == off && i2c_rx_buffer[0] == 'B') {
				// contactors are off let's set the flag to start driver connection
				contactor_transition = i2c_start_request;
			}
		}

		contactors_control();

		// Proccess adc data when dma transfer is complete
		if (adc_dma_complete == 1) {
			process_adc_buffer();
			adc_dma_complete = 0;
		}

		// Estimate SoC every 100ms
		if (HAL_GetTick() - t0 >= SAMPLING_TIME_MS) {
			t0 = HAL_GetTick();
			soc_estimator();
		}

//		HIL_simulation();

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 46;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 80-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65536-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_USART2_UART_Init(void)
{

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, POSITIVE_RELAY_Pin|PRECHARGE_RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : POSITIVE_RELAY_Pin PRECHARGE_RELAY_Pin */
  GPIO_InitStruct.Pin = POSITIVE_RELAY_Pin|PRECHARGE_RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (rx_buffer[0] != '\r') {
		// Read the byte/char and add it to the data variable if there is no \r
		strcat(input_data, (char*) rx_buffer);
		HAL_UART_Receive_IT(&huart2, rx_buffer, 1); //we are still waiting for more data, we will continuo reading
	} else {
		//We get the line terminator, then process the data and send
		process_data = 1;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	process_data = 0; // the input data has been processed and send
	HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

void Heartbeat() {
	if (HAL_GetTick() - beat > 250) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		beat = HAL_GetTick();
	}
}

// DMA callback functions
// Called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {

}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adc_dma_complete = 1;
}

/* Process ADC Buffer Function */
float voltage_temp = 0; // For debug purposes
float current_temp = 0; // for debug purposes
float temperature_temp = 0; // for debug purposes
void process_adc_buffer() {
	float sum_current = 0;
	float sum_voltage = 0;
	float sum_temperature = 0;
	float Rtemp = 0;
	for (int i = 0; i < NUMBER_OF_MEASURES * NUMBER_OF_SAMPLES; i = i + 3) {
		sum_voltage += adc_buffer[i] * 3.312 / 4096.0;
		sum_current += adc_buffer[i + 1] * 3.312 / 4096.0;
		sum_temperature += adc_buffer[i + 2] * 3.312 / 4096.0;
	}
	current_temp = (sum_current / NUMBER_OF_SAMPLES)*I_GAIN;
	current_temp = -(((sum_current / NUMBER_OF_SAMPLES)*(I_GAIN))-V_I_REF)*40;//for estimation output current must be negative
	if(current_temp>-0.1 && current_temp<0.1){
		current = 0;
	}else{
		current = current_temp;
	}
	voltage_temp = (sum_voltage / NUMBER_OF_SAMPLES)*V_GAIN;
	voltage = (sum_voltage / NUMBER_OF_SAMPLES)*V_GAIN*(R1+ROUT)/ROUT;
	sum_temperature = sum_temperature / NUMBER_OF_SAMPLES;
	temperature_temp = sum_temperature;
	Rtemp = (R1*(sum_temperature))/(4.98-sum_temperature); // Get Sensor Resistance
	temperature = -21.71*log(Rtemp)+227.31; //Temperature = -21.71*ln(Rtemp)+227.31
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
		uint16_t AddrMatchCode) {
	//Check if the master is reading or writing.
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
		master_writes = 1;
	} else if (TransferDirection == I2C_DIRECTION_RECEIVE) {
		master_reads = 1;
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {

}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
}

/* I2C processing functions */
void create_i2c_buffer() {
// Create an array with the variables
	float measurements_temp[6] = { rtU.current, rtU.voltage, temperature/10,
			rtY.soc_estimated, rtY.soc_measured, elapsed_time };

//filling the array of soc_measurment
	for (int i = 0; i < I2C_MEASURES; i++) {
		if (i != 5) {
			i2c_measurements[i] = int_to_byte(measurements_temp[i]);
			i2c_tx_buffer[i * 2] = i2c_measurements[i].MSB;
			i2c_tx_buffer[(i * 2) + 1] = i2c_measurements[i].LSB;
		} else {
			// elapsed_time is a uint16, then convertion is different;
			i2c_measurements[i] = uint_to_byte(measurements_temp[i]);
			i2c_tx_buffer[i * 2] = i2c_measurements[i].MSB;
			i2c_tx_buffer[(i * 2) + 1] = i2c_measurements[i].LSB;
		}
	}
}

soc_measurement int_to_byte(float value) {
	int16_t temp_int = value * 1000;
	soc_measurement measure;
	measure.MSB = temp_int >> 8; // Most Significant Byte
	measure.LSB = temp_int & 0xFF; // Less significant Byte
	return measure;
}

soc_measurement uint_to_byte(uint16_t value) {
	uint16_t temp_int = value;
	soc_measurement measure;
	measure.MSB = temp_int >> 8; // Most Significant Byte
	measure.LSB = temp_int & 0xFF; // Less significant Byte
	return measure;
}

/* Hardware In the Loop simulation */
void HIL_simulation() {
	// This block of code send the data to the serial port when the current and voltage signals are received via serial.
	if (process_data == 1) {
		// UART input data is ready to be processed
		if (TESTING) {
			HAL_UART_Transmit_IT(&huart2, (uint8_t*) input_data,
					sizeof(input_data));
		}
		start_time = __HAL_TIM_GET_COUNTER(&htim6);
		//time_start = __HAL_TIM_GET_COUNTER(&htim6); // Timer 1us precision
		token = strtok(input_data, ";");
		rtU.current = atof(token);
		while (token != NULL) {
			token = strtok(NULL, " ");
			if (token != NULL) {
				rtU.voltage = atof(token);
			}
		}
		RC_Model_KF_Vout_Vcb_for_MCU_step();
		elapsed_time = __HAL_TIM_GET_COUNTER(&htim6) - start_time;
		sprintf(output_data, "%d;%.4f;%.4f;%.4f;%.4f\r", elapsed_time,
				rtY.soc_estimated, rtY.voltage_estimated[0], rtU.current,
				rtU.voltage); //voltage_estimated[0]=Vt voltage_estimated[1]=Vcb
		process_data = 0; //The input data has been processed
		HAL_UART_Transmit_IT(&huart2, (uint8_t*) output_data,
				sizeof(output_data));
		memset(input_data, 0, strlen(input_data)); //clean input data
		process_data = 0; // We have processed the data, we have to wait for the arrival of new data.
	}
}

void soc_estimator() {
	rtU.current = current;
	rtU.voltage = voltage;
	start_time = __HAL_TIM_GET_COUNTER(&htim6); // Get current time
//	HAL_Delay(9);
	RC_Model_KF_Vout_Vcb_for_MCU_step();
	elapsed_time = __HAL_TIM_GET_COUNTER(&htim6) - start_time;
}

void contactors_control() {
	GPIO_PinState precharge_pin_state = HAL_GPIO_ReadPin(
			PRECHARGE_RELAY_GPIO_Port, PRECHARGE_RELAY_Pin);
	GPIO_PinState positive_pin_state = HAL_GPIO_ReadPin(
			POSITIVE_RELAY_GPIO_Port, POSITIVE_RELAY_Pin);
	//Transitions
	if (contactor_state == precharge
			&& (HAL_GetTick() - contactor_timer) >= 1000 && current < 3
			&& precharge_pin_state == 1 && positive_pin_state == 0) {
		//Driver has been precharged
		contactor_transition = precharge_cplt;
		contactor_timer = HAL_GetTick(); //Set timer to wait 1s;
	}

	if (contactor_state == positive && (HAL_GetTick() - contactor_timer) >= 250
			&& precharge_pin_state == 1 && positive_pin_state == 1) {
		contactor_transition = positive_cplt;
	}
	// Contactors' control
	switch (contactor_state) {
	case off:
		if (contactor_transition == i2c_start_request) {
			contactor_state = precharge;
			contactor_timer = HAL_GetTick(); //Set timer to wait 1s;
		}
		break;
	case precharge:
		//Let's close the precharge contactor and let the positive contactor open
		HAL_GPIO_WritePin(PRECHARGE_RELAY_GPIO_Port, PRECHARGE_RELAY_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(POSITIVE_RELAY_GPIO_Port, POSITIVE_RELAY_Pin,
				GPIO_PIN_RESET);
		if (contactor_transition == precharge_cplt) {
			contactor_state = positive;
		}
		break;
	case positive:
		HAL_GPIO_WritePin(PRECHARGE_RELAY_GPIO_Port, PRECHARGE_RELAY_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(POSITIVE_RELAY_GPIO_Port, POSITIVE_RELAY_Pin,
				GPIO_PIN_SET);
		if (contactor_transition == positive_cplt) {
			contactor_state = on;
		}
		break;
	case on:
		HAL_GPIO_WritePin(PRECHARGE_RELAY_GPIO_Port, PRECHARGE_RELAY_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(POSITIVE_RELAY_GPIO_Port, POSITIVE_RELAY_Pin,
				GPIO_PIN_SET);
		break;
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

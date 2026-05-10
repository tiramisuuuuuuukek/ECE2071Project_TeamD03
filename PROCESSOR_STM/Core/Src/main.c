/* USER CODE BEGIN Header */
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// *** Task 3 variables for Audio improving initialisation (start) *** //
// Task 3 filter settings
#define OUTLIER_THRESHOLD 140 //test from 40,60,80,100,120 and 140 gives best output (140= amplitude)

// 16-bit SPI frame received from Sampling STM
uint16_t raw_adc_val = 0;

// Simple outlier rejection + moving average variables
uint16_t prev_sample = 512;     // start near midpoint for 10-bit ADC (save 1st value for Moving Average filter of 3)
uint16_t prev_sample2 = 512;	// save 2nd value for Moving Average filter of 3
uint16_t filtered_sample = 512;		// initial value near midpoint instead of 0 is better for average


// Downsampling by 2
uint8_t sample_toggle = 0;
uint16_t downsam_sum = 0;
uint8_t downsam_count = 0;

// UART output byte
uint8_t output_val = 0;

// *** Task 3 variables for Audio  improving initialisation (end) *** //

// ------------------------------------------------------- /

// *** User Interface and Modes (start) *** //
#define MODE_IDLE 'I'			// new mode added because recording audio would prematurely start
#define MODE_MANUAL 'M'
#define MODE_DISTANCE 'D'
uint8_t mode = MODE_IDLE;		// default mode is idle
uint8_t mode_distance = 0;		// enabled = 1 ; disabled = 0
uint8_t recording_enabler = 0;	// enabled = 1 ; disabled = 0
float min_distance_cm = 10;
volatile float distance = 6767;	// include volatile since the variable's value may change unexpectedly at any time
uint32_t count1 = 0;
uint32_t count2 = 0;
uint8_t flag = 0;
uint8_t rx_cmd = 0;
volatile uint8_t distance_valid = 0;

// *** User Interface and Modes (end) *** //

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static void transmit_8bit_audio(uint8_t uart_output);
void delay_uS(uint16_t delay);
void ultrasonic_trig();
static void handle_uart_command(void);

static void reset_audio_filter_state(void);

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start(&htim16);					// timer for counting us delay (note: place base_start above IT)
//  HAL_TIM_Base_Start_IT(&htim7);				// Timer to periodically enable ultrasonic sensor
//  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);	// Echo input capture interrupt/ timer to measure distance
  __HAL_SPI_ENABLE(&hspi1);						// audio SPI receive pollign

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  handle_uart_command();

	  if (mode == MODE_MANUAL)
	  {
		  recording_enabler = 1;		// transmit
	  }
	  transmit_8bit_audio(recording_enabler);		// transmit when enabled
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 32000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 31;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.BaudRate = 921600;
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
  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

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

//uses tim16
void delay_uS(uint16_t delay)
{
	// function to delay in microseconds
    __HAL_TIM_SET_COUNTER(&htim16 , 0);
    while (__HAL_TIM_GET_COUNTER(&htim16) < delay)
    {
    	// wait until delay fully clears
    }
}

void ultrasonic_trig()
{
	// function to trigger ultrasonic sequence of Reset -> set -> reset
	HAL_GPIO_WritePin(Trigger_GPIO_Port,Trigger_Pin, GPIO_PIN_RESET);
	delay_uS(2); //time for us sensor to completely reset
    HAL_GPIO_WritePin(Trigger_GPIO_Port,Trigger_Pin, GPIO_PIN_SET);
    delay_uS(10); //waits for 10us
    HAL_GPIO_WritePin(Trigger_GPIO_Port,Trigger_Pin, GPIO_PIN_RESET);
}

static void reset_audio_filter_state(void)
{
	// Simple function to reset variable values after switching between modes (flushes old data out when doing new recording)
    prev_sample = 512;
    prev_sample2 = 512;
    filtered_sample = 512;
    sample_toggle = 0;
    output_val = 0;
    downsam_sum = 0;
    downsam_count = 0;
}

static void handle_uart_command(void)
{
	// This function is used to read commands from Python.
	// It switches between modes such as IDLE , MANUAL , DISTANCE
	// it should read values of 'I' , 'D' , 'M' and the distance threshold sent
    static uint8_t reading_distance = 0;		// used for 'D', 1 = start reading digit , 0 = idle
    static uint16_t pending_distance = 0;		// 1 = start reading digit , 0 = idle
    // e.g if 10 is typed on CMD -> 1 is transmitted then only 0 is transmitted, so it has to save the first byte (1) then append 0 to the back to get 10

    // reads 1 byte at a time from USART2; 0 means non-blocking -> no byte available. immediately exits to prevent bottlenecking transmissionr ate
    while (HAL_UART_Receive(&huart2, &rx_cmd, 1, 0) == HAL_OK)
    {
    	// if mode is in IDLE mode
        if (rx_cmd == 'I')
        {
        	// IDLE mode must ensure audio is NOT being sent to Python
        	// keep spi audio draining in the background
            mode = MODE_IDLE;
            mode_distance = 0;			// Disable distance mode
            recording_enabler = 0;		// stop audio being transmitted

            reset_audio_filter_state();		// reset memory

        	// Reset old distance readings
            reading_distance = 0;
            pending_distance = 0;

            // stop ultrasonic timers
            HAL_TIM_Base_Stop_IT(&htim7);
            HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);

            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1,
                                           TIM_CHANNEL_1,
                                           TIM_INPUTCHANNELPOLARITY_RISING);
            flag = 0;
            distance = 999;			// 999 is used as "Too Far away value"
            distance_valid = 0;
        }
        // IF mode is in Manual mode
        else if (rx_cmd == 'M')
        {
            mode = MODE_MANUAL;
            // bypass ultrasonic sensor
            mode_distance = 0; //does not trigger distance mode
            // straight away enable the recording and transmitting
            recording_enabler = 1;
            reset_audio_filter_state();
            reading_distance = 0;
            pending_distance = 0;

            // stop ultrasonic timers
            HAL_TIM_Base_Stop_IT(&htim7);
            HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);

            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1,
                                           TIM_CHANNEL_1,
                                           TIM_INPUTCHANNELPOLARITY_RISING);
            flag = 0;
            distance = 999;
        }
        // if mode is in Distance mode
        else if (rx_cmd == 'D')
        {
            /*
             * Do not immediately start distance mode. This is to prevent premature audio sampling.
             * Wait until Python sends the distance threshold.
             */
        	// temporary idel mode
            mode = MODE_IDLE;
            mode_distance = 0;
            recording_enabler = 0;


            reading_distance = 1;		// start reading distance number
            pending_distance = 0;

            HAL_TIM_Base_Stop_IT(&htim7);
            HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1,
                                           TIM_CHANNEL_1,
                                           TIM_INPUTCHANNELPOLARITY_RISING);
            flag = 0;
            distance = 999;
        }
        else if (reading_distance)
        {
        	// builts the minimum distance sent through python
            if (rx_cmd >= '0' && rx_cmd <= '9') //command received from python
            {
            	// converts ASCII characters to integer numbers
                pending_distance = pending_distance * 10 + (rx_cmd - '0');
            }
            else if (rx_cmd == '\n' || rx_cmd == '\r')	// when newline is received, the distance number is finished
            {
                if (pending_distance > 0 && pending_distance < 400)
                {
                    min_distance_cm = pending_distance;
                }
                // enables distance mode logic
                mode = MODE_DISTANCE;
                mode_distance = 1;
                recording_enabler = 0;

                flag = 0;
                distance = 999;

                // reset timers
                __HAL_TIM_SET_COUNTER(&htim1, 0);
                __HAL_TIM_SET_COUNTER(&htim7, 0);

                // start ultrasonic timers
                HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);		// tim1 measures ultrasonic echo pulse
                HAL_TIM_Base_Start_IT(&htim7);					// tim7 periodically triggers the ultrasonic sensor, instead of always turning it on

                reading_distance = 0;
                pending_distance = 0;
            }
        }
    }
}
//to trigger us sensor
//uses tim7
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        if (mode_distance)
        {
            if (distance > 1 && distance < min_distance_cm && distance_valid)
            {
                recording_enabler = 1;
                // set Green light ON when minimum distance is reached
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
            }
            else
            {
                recording_enabler = 0;
                // set Green light OFF when minimum distance is reached
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
            }
            distance_valid = 0;
            // runs every 100 ms, start next ultraspnic measurement at the end
        	ultrasonic_trig();

        }
    }
}

//measures pulse width: to calculate distance of us sensor
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM1) &&
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) &&
        (mode_distance == 1))
    {
        if (flag == 0)
        {
            count1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);

            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1,
                                           TIM_CHANNEL_1,
                                           TIM_INPUTCHANNELPOLARITY_FALLING);
            flag = 1;
        }
        else
        {
            count2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);

            uint32_t pulse_width;

            if (count2 >= count1)
            {
                pulse_width = count2 - count1;
            }
            else
            {
                pulse_width = (65535 - count1) + count2;
            }

            distance = pulse_width / 58.0f;
            distance_valid = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1,
                                           TIM_CHANNEL_1,
                                           TIM_INPUTCHANNELPOLARITY_RISING);
            flag = 0;
        }
    }
}

static void transmit_8bit_audio(uint8_t uart_output)
{
    // RXNE = Receive buffer not empty.
    // If no SPI sample has arrived, return immediately, instead of computing
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET)
    {
        return;
    }

    // Read one 16-bit SPI frame from the SPI data register.
    // Only the lower 10 bits are useful ADC data.
    uint16_t raw_sample = (uint16_t)(hspi1.Instance->DR);
    raw_sample &= 0x03FF;			// use AND operand to extract the first 10 bits out of the 16 bits since adc value is 10-bit

    // Clear overrun if it happened.
    // This prevents SPI from getting stuck after missed samples.
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_OVR) != RESET)
    {
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    }

    // Simple outlier rejection:
    // If the new sample difference between the previous accepted sample is too much to the OUTLIER THRESHOLD we tuned,
    // limit the jump instead of replacing the value.

    int16_t diff = (int16_t)raw_sample - (int16_t)prev_sample;

    if (diff > OUTLIER_THRESHOLD)
    {
        raw_sample = prev_sample + OUTLIER_THRESHOLD; //uses previous sample + threshold
    } else if (diff < -OUTLIER_THRESHOLD)
    {
    	raw_sample = prev_sample - OUTLIER_THRESHOLD;
    }

    //Optimised moving average filter of length 3:
    // average current accepted sample with previous two accepted sample.
    filtered_sample = (raw_sample + prev_sample + prev_sample2)/3;

    // shife sample history for next incoming sample
    prev_sample2 = prev_sample;
    prev_sample = raw_sample;

    // Sampling STM sends about 44.1 ksps. For Processing SMT to send about 22.05ksps
    // Downsample by 2 to meet the requirement
    // Using prac 4 week 7, Concept 2:
    // Instead of Dropping first samples and only forwarding every second sample, half the information is thrown away.
    // Use of SAMPLE AVERAGING
    downsam_sum += filtered_sample;
    downsam_count++;

    if (downsam_count >= 2)
    {
    	uint16_t downsam_avg_val = downsam_sum >> 1;		// divide sum by 2
    	downsam_sum = 0;
    	downsam_count = 0;

    	if (uart_output)
    	{
    		output_val = (uint8_t)(downsam_avg_val >> 2);		// shift by 2 bits to the right to remove 2 LSB(most right bits)
    		HAL_UART_Transmit(&huart2, &output_val, 1, 1);
    	}
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
